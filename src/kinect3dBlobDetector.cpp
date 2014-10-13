#include "kinect3dBlobDetector.h"

#include "ofxCv.h"

namespace garmentAugmentation {

	static double fx_d = 1.0 / 5.9421434211923247e+02;
	static double fy_d = 1.0 / 5.9104053696870778e+02;
	static float cx_d = 3.3930780975300314e+02;
	static float cy_d = 2.4273913761751615e+02;
	static int clockwiseBacktracking[] = { 6, 0, 0, 2, 2, 4, 4, 6 }; // newStart = clockwiseBacktraking[previous_backtrack]
	static int clockwiseX[] = { -1, -1, 0, 1, 1, 1, 0, -1, -1, -1, 0, 1, 1, 1, 0, -1 };
	static int clockwiseY[] = { 0, -1, -1, -1, 0, 1, 1, 1, 0, -1, -1, -1, 0, 1, 1, 1 };

	// ***************************************************************************
	//                                CONSTRUCTORS
	// ***************************************************************************
	kinect3dBlobDetector::kinect3dBlobDetector() {
		m_pointCloud = NULL;
		kinectPtr = NULL;
		kWidth = 640;
		kHeight = 480;
		setResolution(BF_MEDIUM_RES);
		nullPoint = ofVec3f(0, 0, 0);
		bAllocatedMap = false;
	}

	kinect3dBlobDetector::~kinect3dBlobDetector() {
	}

	// ***************************************************************************
	//                                INIT
	// ***************************************************************************
	void kinect3dBlobDetector::init(ofxKinectCommonBridge *newKinect) {
		kinectPtr = newKinect;
		kWidth = kinectPtr->getColorPixelsRef().getWidth();
		kHeight = kinectPtr->getColorPixelsRef().getHeight();
		kNPix = kWidth*kHeight;
		bFinderInited = setResolution(BF_MEDIUM_RES);
	}

	// ***************************************************************************
	//                                FIND BLOBS

	//  maskPixels : for background subtraction
	// neighbour search range
	// http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

	// ***************************************************************************

	bool kinect3dBlobDetector::findBlobs(ofImage * maskImage,
		const ofVec3f thresh3D, const int thresh2D,
		const float minVol, const float maxVol,
		const int minPoints, const unsigned int maxBlobs)
	{
		if (!bFinderInited) {
			ofLog(OF_LOG_WARNING, "ofxKinectBlobFinder: findBlobs - must init finder first");
			return false;
		}
		if (!maskImage->bAllocated() || (maskImage->getWidth() != kWidth) || (maskImage->getHeight() != kHeight) ||
			(maskImage->bpp != 8)) {
			ofLog(OF_LOG_WARNING, "ofxKinectBlobFinder: findBlobs - mask image mismatch");
			return false;
		}
		if (!createCloud(maskImage->getPixels())) {
			ofLog(OF_LOG_WARNING, "ofxKinectBlobFinder: findBlobs - could not create pointcloud");
			return false;
		}

		int queueIndex = 0;
		int lastQueued = 0;
		int pixIndex = 0;

		int pixelsToProcess = nPix;
		int *queue = new int[nPix];

		blobs.clear();
		int numBlobs = 0;

		float minX, minY, minZ, maxX, maxY, maxZ;

		ofVec3f *minXPos, *minYPos, *minZPos, *maxXPos, *maxYPos, *maxZPos;
		std::vector<simple3dBlob * > tempBlobs;

		while ((pixIndex < nPix) && (pixelsToProcess > 0) &&
			(tempBlobs.size() < maxBlobs)) {
			queueIndex = 0;
			lastQueued = 0;
			minX = minY = minZ = 100;
			maxX = maxY = maxZ = -100;
			//search for next unprocessed pixel
			while ((pixIndex < nPix) &&
				(m_pointCloud[pixIndex].flag != FLAG_IDLE)) pixIndex++;

			if (pixIndex == nPix) break;
			else queue[0] = pixIndex;

			UCHAR* maskPixels;

			int queueIndexPix = queue[queueIndex];
			// while not end of queue
			while ((lastQueued < nPix) && (queueIndex <= lastQueued) &&
				(queueIndexPix >= 0) &&
				(queueIndexPix < nPix) && (pixelsToProcess > 0)) {

				pixelsToProcess--;

				cloudPoint * p_cloudPoint = &m_pointCloud[queueIndexPix];
				(*p_cloudPoint).flag = FLAG_PROCESSED;

				ofVec3f * posPtr = &((*p_cloudPoint).pos);
				float pointX = (*posPtr).x;
				float pointY = (*posPtr).y;
				float pointZ = (*posPtr).z;

				if (pointX < minX) { minX = pointX; minXPos = posPtr; }
				else if (pointX > maxX) { maxX = pointX; maxXPos = posPtr; }
				if (pointY < minY) { minY = pointY; minYPos = posPtr; }
				else if (pointY > maxY) { maxY = pointY; maxYPos = posPtr; }
				if (pointZ < minZ) { minZ = pointZ; minZPos = posPtr; }
				else if (pointZ > maxZ) { maxZ = pointZ; maxZPos = posPtr; }

				int i = queueIndexPix % width;
				int j = queueIndexPix / width;

				for (int u = i - thresh2D; u <= i + thresh2D; u++) {
					for (int v = j - thresh2D; v <= j + thresh2D; v++) {
						int neighbour = u + v*width;

						if ((neighbour >= 0) && (neighbour < nPix) && (m_pointCloud[neighbour].flag == FLAG_IDLE)) {
							ofVec3f nPoint = m_pointCloud[neighbour].pos;
							if ((abs(pointX - nPoint.x) <= thresh3D.x) &&
								(abs(pointY - nPoint.y) <= thresh3D.y) &&
								(abs(pointZ - nPoint.z) <= thresh3D.z)) {
								lastQueued++;
								if (lastQueued < nPix) {
									queue[lastQueued] = neighbour;
									m_pointCloud[neighbour].flag = FLAG_QUEUED;
								}
							}
						}
					}
				}

				queueIndex++;
				queueIndexPix = queue[queueIndex];
			}


			if (lastQueued > minPoints) {
				ofPoint blobDim = ofPoint(abs(maxX - minX), abs(maxY - minY), abs(maxZ - minZ));
				float blobVol = blobDim.x*blobDim.y*blobDim.z;

				// minimum number of pixels to be considered as a blob
				if ((blobVol >= minVol) && (blobVol <= maxVol)) {
					ofVec3f newMassCenter = ofVec3f(0.0f, 0.0f, 0.0f);
					simple3dBlob* newBlob = new simple3dBlob();

					for (int i = 0; i < lastQueued; i++) {
						m_pointCloud[queue[i]].flag = numBlobs;
						newMassCenter += m_pointCloud[queue[i]].pos;//*pointWeight;
					}

					newBlob->boundingBoxMin = ofPoint(minX, minY, minZ);
					newBlob->boundingBoxMax = ofPoint(maxX, maxY, maxZ);
					newBlob->minX = *minXPos;    newBlob->maxX = *maxXPos;
					newBlob->minY = *minYPos;    newBlob->maxY = *maxYPos;
					newBlob->minZ = *minZPos;    newBlob->maxZ = *maxZPos;
					newBlob->dimensions = blobDim;
					newBlob->volume = blobVol;
					newBlob->massCenter = newMassCenter / lastQueued;
					newBlob->centroid = newBlob->boundingBoxMin.getMiddle(newBlob->boundingBoxMax);
					newBlob->idx = numBlobs;
					newBlob->nbPoints = lastQueued;

					// Mark the 2d contour of the blob
					while ((m_pointCloud[pixIndex].flag != numBlobs) && (pixIndex < nPix)) pixIndex++;
					m_pointCloud[pixIndex].boundary = true;
					newBlob->contourIndices2d.push_back(pixIndex);

					int i = pixIndex % width;
					int j = pixIndex / width;
					int x, y;
					bool isContourClosed = false;
					int k = 0;
					while (!isContourClosed) {
						x = i + clockwiseX[k]; y = j + clockwiseY[k];

						if (x > 0 && x < width && y > 0 && y < height){
							int neighbour = x + y * width;

							if (m_pointCloud[neighbour].flag == numBlobs) {
								m_pointCloud[neighbour].boundary = true;
								newBlob->contourIndices2d.push_back(neighbour);
								k = clockwiseBacktracking[(k - 1) % 8];
								i = neighbour % width;
								j = neighbour / width;

								if (pixIndex == neighbour)
									isContourClosed = true;
							}
							else
								k++;
						}
					}

					tempBlobs.push_back(newBlob);
					numBlobs++;
				}
			}
		}

		for (int i = 0; i < blobs.size(); ++i)
			delete blobs[i];
		blobs = tempBlobs;
		nBlobs = numBlobs;
		tempBlobs.clear();
		delete[] queue;

		return true;
	}

	// ************************************************************
	//                CREATE POINT CLOUD AND SCALE IMAGES
	// ************************************************************

	bool kinect3dBlobDetector::createCloud(unsigned char * maskPix) {
		ofVec3f thePos;
		cloudPoint * p_cloudPoint = &m_pointCloud[0];
		ofShortPixels distances = kinectPtr->getDepthPixels();

		int row_incr = kWidth*(resolution - 1);

		for (int j = 0; j < kHeight; j += resolution) {
			for (int i = 0; i < kWidth; i += resolution) {
				if (*maskPix == 0 || distances[j*kWidth + i] == 0){
					(*p_cloudPoint).flag = FLAG_BACKGROUND;
					(*p_cloudPoint).pos = nullPoint;
				}
				else {
					(*p_cloudPoint).flag = FLAG_IDLE;
					(*p_cloudPoint).pos = kinectPtr->getWorldCoordinates(i, j, distances[j*kWidth + i]) * scale;
				}
				p_cloudPoint++;
				maskPix += resolution;
			}
			maskPix += row_incr;
		}
		return true;
	}

	// ************************************************************
	//                SET POINT CLOUD SCALE
	// ************************************************************
	void kinect3dBlobDetector::setScale(const ofVec3f newScale) {
		scale = newScale;
	}

	// ************************************************************
	//                GET POINT CLOUD SCALE
	// ************************************************************
	ofVec3f kinect3dBlobDetector::getScale() const {
		return scale;
	};

	// ************************************************************
	//                SET ANALYSIS/POINTCLOUD RESOLUTON
	// ************************************************************
	bool kinect3dBlobDetector::setResolution(kinectBlobDetectorResolution newResolution) {
		if ((m_pointCloud == NULL) || (resolution != newResolution)) {
			resolution = newResolution;
			width = kWidth / resolution;
			height = kHeight / resolution;
			nPix = width*height;
			if (m_pointCloud != NULL) delete[] m_pointCloud;
			m_pointCloud = new cloudPoint[nPix];
			if (m_pointCloud == NULL) {
				ofLog(OF_LOG_WARNING, "ofxKinectBlobFinder: setResolution - error allocating memory for pointcloud ");
				return false;
			}
		}
		return true;
	}

	// ************************************************************
	//                GET ANALYSIS/POINTCLOUD RESOLUTON
	// ************************************************************
	enum kinectBlobDetectorResolution kinect3dBlobDetector::getResolution() {
		return resolution;
	}

	// ************************************************************
	//
	// ************************************************************
	bool kinect3dBlobDetector::isInited() {
		return bFinderInited;
	}

};