#include "ofApp.h"

#include <stdlib.h>
#include "cvHelper.h"
#include "lscm.h"

//--------------------------------------------------------------
void ofApp::setup() {
	// Kinect
	mKcbKinect = mOfxKinect.getHandle();
	mKinectColorImgWidth = 640;
	mKinectColorImgHeight = 480;
	mKinectDepthImgWidth = 640;
	mKinectDepthImgHeight = 480;

	bool initSensor = mOfxKinect.initSensor();
	if (!initSensor) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's sensor";
		exit();
	}

	bool initStreams = mOfxKinect.initColorStream(mKinectColorImgWidth, mKinectColorImgHeight) && mOfxKinect.initDepthStream(mKinectDepthImgWidth, mKinectDepthImgHeight, false, true);
	if (!initStreams) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's color and/or depth streams";
		exit();
	}

	bool initSkeletonStream = mOfxKinect.initSkeletonStream(false);
	if (!initSkeletonStream) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's skeleton stream";
		exit();
	}

	bool kinectStarted = mOfxKinect.start();
	if (!kinectStarted) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't start the kinect";
		exit();
	}

	// Images allocation
	mOfSegmentedImg.allocate(mKinectColorImgWidth, mKinectColorImgHeight, OF_IMAGE_COLOR);
	mCvSegmentedImg = ofxCv::toCv(mOfSegmentedImg);
	mOfGarmentMask.allocate(mKinectColorImgWidth, mKinectColorImgHeight, OF_IMAGE_GRAYSCALE);
	mCvGarmentMask = ofxCv::toCv(mOfGarmentMask);

	// Computer vision tools
	mContourFinder.setAutoThreshold(false);
	mContourFinder.setFindHoles(false);
	mContourFinder.setInvert(false);
	mContourFinder.setSimplify(true);
	mContourFinder.setSortBySize(true);
	mContourFinder.setUseTargetColor(false);

	// Textures
	mChessboardImage.loadImage("chessboard.png");

	// OpenGL
	ofDisableAlphaBlending();
	ofSetFrameRate(30);

	// GUI-
	mGui.setup();
	mGui.add(mGarmentSegmentationLowH.setup("low hue thresh", 70, 0, 179));
	mGui.add(mGarmentSegmentationLowS.setup("low saturation thresh", 0, 0, 255));
	mGui.add(mGarmentSegmentationLowV.setup("low value thresh", 40, 0, 255));
	mGui.add(mGarmentSegmentationHighH.setup("high hue thresh", 120, 0, 179));
	mGui.add(mGarmentSegmentationHighS.setup("high saturation thresh", 95, 0, 255));
	mGui.add(mGarmentSegmentationHighV.setup("high value thresh", 255, 0, 255));
	mGui.add(mOpenKernelSize.setup("opening kernel size", 5, 1, 9));
	mGui.add(mCloseKernelSize.setup("closing kernel size", 9, 1, 9));
	mGui.add(mMorphoUseEllipse.setup("ellipse for morpho operations",false));
	mGui.add(mGarmentBodyPercent.setup("percent of clothing on body", 20, 0, 100)); // 30% is a good value for a t-shirt
	mGui.add(mMeshGenerationStep.setup("undersampling the polygonal mesh", 7, 1, 20)); // in pixels
	
	// Keys
	mPause = false;
	mSaveMesh = false;
}

//--------------------------------------------------------------
void ofApp::update() {
	if (mSaveMesh) {
		mGarmentGeneratedMesh.save("export.ply");
		ofLog() << "Exported the 3D mesh";
		mSaveMesh = false;
	}
	if (mPause)
		return;

	mOfxKinect.update();

	// Model ROI
	mModelRoi = ofRectangle(mOfSegmentedImg.width, mOfSegmentedImg.height, 0, 0);
	int modelRoiXBottomRight = 0, modelRoiYBottomRight = 0;
	int modelBodyPixelsCount = 0; // Used to discriminate contours, more robust than bounding box if the model extend their arms, etc.

	// Segmentation from the skeleton
	if (mOfxKinect.isNewSkeleton()) {
		ofShortPixels depthPlayerPixel = mOfxKinect.getDepthPlayerPixelsRef(); // The kinect segmentation map

		for (int x = 0; x < mOfSegmentedImg.width; ++x) {
			for (int y = 0; y < mOfSegmentedImg.height; ++y) {
				USHORT playerId = depthPlayerPixel[x + y*mOfSegmentedImg.width];

				if (playerId != 0) {
					ofColor bgra = mOfxKinect.getColorPixelsRef().getColor(x, y);
					mOfSegmentedImg.setColor(x, y, ofColor(bgra.b, bgra.g, bgra.r));

					// Update the ROI
					mModelRoi.x = (x < mModelRoi.x) ? x : mModelRoi.x;
					mModelRoi.y = (y < mModelRoi.y) ? y : mModelRoi.y;
					modelRoiXBottomRight = (x > modelRoiXBottomRight) ? x : modelRoiXBottomRight;
					modelRoiYBottomRight = (y > modelRoiYBottomRight) ? y : modelRoiYBottomRight;
					modelBodyPixelsCount++;
				}
				else {
					mOfSegmentedImg.setColor(x, y, ofColor::black);
				}
			}
		}
	
		// Cloth segmentation and contour acquisition
		if (modelBodyPixelsCount != 0) {
			// Update the ROI information of the model
			mModelRoi.width = modelRoiXBottomRight - mModelRoi.x;
			mModelRoi.height = modelRoiYBottomRight - mModelRoi.y;
			cv::Mat cvSegmentedImgRoi = mCvSegmentedImg(ofxCv::toCv(mModelRoi));
			cv::Mat cvGarmentMaskRoi = mCvGarmentMask(ofxCv::toCv(mModelRoi));

			// OPENCV
			// Color segmentation
			cv::Mat cvSegmentedImgHsv;
			cv::cvtColor(cvSegmentedImgRoi, cvSegmentedImgHsv, CV_RGB2HSV);
			cv::inRange(cvSegmentedImgHsv, cv::Scalar(mGarmentSegmentationLowH, mGarmentSegmentationLowS, mGarmentSegmentationLowV), cv::Scalar(mGarmentSegmentationHighH, mGarmentSegmentationHighS, mGarmentSegmentationHighV), cvGarmentMaskRoi);

			// Morphological opening (remove small objects) and closing (fill small holes)
			cv::Mat openingOperator = cv::getStructuringElement(mMorphoUseEllipse ? cv::MORPH_ELLIPSE : cv::MORPH_RECT, cv::Size(mOpenKernelSize, mOpenKernelSize));
			cv::Mat closingOperator = cv::getStructuringElement(mMorphoUseEllipse ? cv::MORPH_ELLIPSE : cv::MORPH_RECT, cv::Size(mCloseKernelSize, mCloseKernelSize));
			cv::morphologyEx(cvGarmentMaskRoi, cvGarmentMaskRoi, cv::MORPH_OPEN, openingOperator);
			cv::morphologyEx(cvGarmentMaskRoi, cvGarmentMaskRoi, cv::MORPH_CLOSE, closingOperator);
			// !OPENCV

			// Find the biggest correct contour
			mContourFinder.setMinArea(modelBodyPixelsCount*mGarmentBodyPercent / 100.f);
			mContourFinder.findContours(cvGarmentMaskRoi);
			if (mContourFinder.getContours().size() != 0) {
				mCvGarmentContourModelRoiRel = mContourFinder.getContour(0);
				mOfGarmentContourModelRoiRel = mContourFinder.getPolyline(0);
				mOfGarmentMask.setColor(ofColor::black); // reset the mask as opencv only draw on the ROI
				ofxCv::fillPoly(mCvGarmentContourModelRoiRel, cvGarmentMaskRoi);
			}
			else {
				ofLog(OF_LOG_ERROR) << "Couldn't retrieve big enough contour";
				mCvGarmentContourModelRoiRel.clear();
				mOfGarmentContourModelRoiRel.clear();
			}
		}
		else {
			ofLog(OF_LOG_ERROR) << "Found skeleton but couldn't detect garment";
			mCvGarmentContourModelRoiRel.clear();
			mOfGarmentContourModelRoiRel.clear();
		}

		// Generate the point of cloud
		if (mOfGarmentContourModelRoiRel.size() != 0) {
			// Update the ROI information of the garment
			mGarmentRoi = mOfGarmentContourModelRoiRel.getBoundingBox();
			mGarmentRoi.translate(mModelRoi.getTopLeft());
			generateMesh(mCvGarmentMask(ofxCv::toCv(mGarmentRoi)), mGarmentGeneratedMesh, mMeshGenerationStep, mGarmentRoi.getTopLeft());
			//computeNormals(mGarmentGeneratedMesh, true);
			meshParameterizationLSCM(mGarmentGeneratedMesh);
		}
		mOfSegmentedImg.update();
		mOfGarmentMask.update();
	} // mOfxKinect.isNewSkeleton()
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (mOfxKinect.isFrameNew()) {
		ofBackground(ofColor::blue);

		// Top Left
		mOfxKinect.draw(0, 0);

		ofPushMatrix();
		ofSetColor(ofColor::red);
		ofTranslate(mModelRoi.getTopLeft());
		mOfGarmentContourModelRoiRel.draw();
		ofPopMatrix();

		ofSetColor(ofColor::white);

		// Top Right
		mOfxKinect.drawDepth(mKinectColorImgWidth, 0);

		// Bottom Left
		//mOfSegmentedImg.draw(0, mKinectColorImgHeight);
		mOfGarmentMask.draw(0, mKinectColorImgHeight);

		// Bottom Right
		//mOfGarmentMask.draw(mKinectColorImgWidth, mKinectColorImgHeight);
		if (mGarmentGeneratedMesh.hasVertices()) {
			glPointSize(1);

			mEasyCam.begin();
			if (!mPause) {
				mEasyCam.setTarget(mGarmentGeneratedMesh.getCentroid());
			}
			ofPushMatrix();
				ofEnableDepthTest();
				mChessboardImage.bind();
				mGarmentGeneratedMesh.draw();
				mChessboardImage.unbind();
				ofDisableDepthTest();
			ofPopMatrix();
			mEasyCam.end();
		}
	}
	mGui.draw();
}

//--------------------------------------------------------------
void ofApp::exit() {
	mOfxKinect.stop();
}

void ofApp::mousePressed(int x, int y, int button) {
	if (button == OF_MOUSE_BUTTON_MIDDLE) {
		mSaveMesh = true;
	}
}

void ofApp::keyPressed(int key) {
	if (key == ' ') {
		mPause = !mPause;
	}
}

void ofApp::generateMesh(cv::Mat& maskImage, ofMesh& mesh, int step, ofPoint offset){
	int width = maskImage.cols;
	int height = maskImage.rows;
	int meshRows = height / step;
	int meshCols = width / step;

	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
	mesh.enableIndices();

	// Generate a lying mesh on the contour
	cv::Mat meshIndices;
	meshIndices.create(meshRows, meshCols, CV_32S);
	int index = -1;
	cv::Point2i a, b, c, d;
	for (int row = 0; row < meshRows; row++) {
		for (int col = 0; col < meshCols; col++) {
			a = cv::Point2i(col*step, row*step);
			b = cv::Point2i((col + 1)*step, row*step);
			c = cv::Point2i((col + 1)*step, (row + 1)*step);
			d = cv::Point2i(col*step, (row + 1)*step);

			if (cvHelper::zeroAt(maskImage, a.y, a.x)) { // Snapp the point to the contour if one of the square vertice is in the contour mask
				cv::Point2i snapDirection = cv::Point2i(0, 0);

				if (b.y > 0  && b.x > 0 && b.y < height && b.x < width && !cvHelper::zeroAt(maskImage, b.y, b.x)) {
					snapDirection.x += 1;
				}
				if (d.y > 0 && d.x > 0 && d.y < height && d.x < width && !cvHelper::zeroAt(maskImage, d.y, d.x)) {
					snapDirection.y += 1;
				}
				if (c.y > 0 && c.x > 0 && c.y < height && c.x < width && !cvHelper::zeroAt(maskImage, c.y, c.x)) {
					snapDirection.x += 1;
					snapDirection.y += 1;
				}

				if (snapDirection != cv::Point2i(0, 0)) {
					snapDirection = (snapDirection == cv::Point2i(2, 2)) ? cv::Point2i(1, 1) : snapDirection;

					while (cvHelper::zeroAt(maskImage, a.y, a.x)) {
						a += snapDirection;
					}

					ofVec3f worldCoordinates = mOfxKinect.getWorldCoordinates(a.x + offset.x, a.y + offset.y) * ofVec3f(-1, -1, 1);
					if (abs(worldCoordinates.z) > 0.1) {
						mesh.addVertex(worldCoordinates);
					}
					else {
						mesh.addVertex(ofVec3f(a.x + offset.x, a.y + offset.y, 0));
					}
					*(meshIndices.ptr<int>(row)+col) = ++index;
				}
				else{
					*(meshIndices.ptr<int>(row)+col) = -1; // Discontinuity in the contour
				}
			}
			else {
				ofVec3f worldCoordinates = mOfxKinect.getWorldCoordinates(a.x + offset.x, a.y + offset.y) * ofVec3f(-1, -1, 1);
				if (abs(worldCoordinates.z) > 0.1) {
					mesh.addVertex(worldCoordinates);
				}
				else {
					mesh.addVertex(ofVec3f(a.x + offset.x, a.y + offset.y, 0));
				}
				*(meshIndices.ptr<int>(row)+col) = ++index;
			}
		}
	}


	// Link the points in a mesh
	bool* indicesUsed = new bool[mesh.getNumVertices()];
	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		indicesUsed[i] = false;
	}
	for (int row = 0; row < meshRows - 1; row++) {
		for (int col = 0; col < meshCols - 1; col++) {
			int pointAIdx = *(meshIndices.ptr<int>(row) + col);
			int pointBIdx = *(meshIndices.ptr<int>(row) + col + 1);
			int pointCIdx = *(meshIndices.ptr<int>(row + 1) + col + 1);
			int pointDIdx = *(meshIndices.ptr<int>(row + 1) + col);

			if (pointAIdx != -1 && pointBIdx != -1 && pointDIdx != -1) {
				mesh.addTriangle(pointAIdx, pointBIdx, pointDIdx);
				indicesUsed[pointAIdx] = true;
				indicesUsed[pointBIdx] = true;
				indicesUsed[pointDIdx] = true;
			}
			if (pointBIdx != -1 && pointCIdx != -1 && pointDIdx != -1) {
				mesh.addTriangle(pointBIdx, pointCIdx, pointDIdx);
				indicesUsed[pointBIdx] = true;
				indicesUsed[pointCIdx] = true;
				indicesUsed[pointDIdx] = true;
			}
		}
	}


	// Remove the eventual unused vertices
	for (int i = mesh.getNumVertices(); i >= 0; --i) {
		if (!indicesUsed[i]) {
			mesh.removeVertex(i);

			for (int g = 0; g < mesh.getNumIndices(); ++g) {
				if (mesh.getIndex(g) > i) {
					mesh.setIndex(g, mesh.getIndex(g) - 1);
				}
			}
		}
	}

	// Average the missing depths
	ofShortPixels& depthPixels = mOfxKinect.getDepthPixelsRef();
	ofRectangle roi(offset.x, offset.y, width, height);
	cv::Mat cvDepthPixels = ofxCv::toCv(depthPixels);
	cvDepthPixels = cvDepthPixels(ofxCv::toCv(roi));

	cv::Mat cvDepthPixelsInPainted = cvDepthPixels.clone();
	cv::Mat cvZeroMask;
	cvZeroMask.create(cvDepthPixelsInPainted.rows, cvDepthPixelsInPainted.cols, cvDepthPixelsInPainted.type());

	for (int row = 0; row < cvDepthPixelsInPainted.rows; ++row) {
		USHORT* pRow = cvDepthPixelsInPainted.ptr<USHORT>(row);
		for (int col = 0; col < cvDepthPixelsInPainted.cols; ++col) {
			if (*(pRow + col) == 0) {
				*(cvZeroMask.ptr<USHORT>(row)+col) = 0;
			}
			else if (cvHelper::zeroAt(maskImage, row, col)) {
				*(cvZeroMask.ptr<USHORT>(row)+col) = 0;
				*(pRow + col) = 0;
			}
			else {
				*(cvZeroMask.ptr<USHORT>(row)+col) = 65535;
			}
		}
	}

	int depthInpaintingKernelSize = 1;
	bool unknownValue = true;
	while (unknownValue) {
		depthInpaintingKernelSize += 2;
		cv::GaussianBlur(cvZeroMask, cvZeroMask, cv::Size(depthInpaintingKernelSize, depthInpaintingKernelSize), 1);
		cv::GaussianBlur(cvDepthPixelsInPainted, cvDepthPixelsInPainted, cv::Size(depthInpaintingKernelSize, depthInpaintingKernelSize), 1);
		cvDepthPixelsInPainted = 65535 * (cvDepthPixelsInPainted / cvZeroMask);

		ofVec3f meshPoint;
		unknownValue = false;
		for (int i = 0; i < mesh.getNumVertices(); ++i) {
			meshPoint = mesh.getVertex(i);
			if (abs(meshPoint.z) < 0.1) {
				meshPoint = mOfxKinect.getWorldCoordinates(meshPoint.x, meshPoint.y, *(cvDepthPixelsInPainted.ptr<USHORT>(meshPoint.y - offset.y) + (USHORT)(meshPoint.x - offset.x))) * ofVec3f(-1, -1, 1);
				
				if (abs(meshPoint.z) < 0.1 && depthInpaintingKernelSize < 9) {
					unknownValue = true;
				}
				else if (depthInpaintingKernelSize > 9) {
					ofLogError("ofApp::generateMesh") << "Couldn't inpaint the mesh";
					break;
				}
				else {
					mesh.setVertex(i, meshPoint);
				}
			}
		}
	}
}

void ofApp::computeNormals(ofMesh& mesh, bool bNormalize){
	for (int i = 0; i < mesh.getVertices().size(); i++) mesh.addNormal(ofPoint(0, 0, 0));

	for (int i = 0; i < mesh.getIndices().size(); i += 3){
		const int ia = mesh.getIndices()[i];
		const int ib = mesh.getIndices()[i + 1];
		const int ic = mesh.getIndices()[i + 2];
		ofVec3f a = mesh.getVertices()[ia];
		ofVec3f b = mesh.getVertices()[ib];
		ofVec3f c = mesh.getVertices()[ic];

		ofVec3f e1 = a - b;
		ofVec3f e2 = c - b;
		ofVec3f no = e2;
		no.cross(e1);

		mesh.getNormals()[ia] += no;
		mesh.getNormals()[ib] += no;
		mesh.getNormals()[ic] += no;
	}

	if (bNormalize)
	for (int i = 0; i < mesh.getNormals().size(); i++) {
		mesh.getNormals()[i].normalize();
	}
}

void ofApp::meshParameterizationLSCM(ofMesh& mesh) {
	mesh.enableTextures();
	ofVec2f* texCoords = new ofVec2f[mesh.getNumVertices()];
	mesh.addTexCoords(texCoords, mesh.getNumVertices());
	delete[] texCoords;

	LSCM lscm(mesh);
	lscm.apply();
	float uRange = std::abs(lscm.umax - lscm.umin);
	float vRange = std::abs(lscm.vmax - lscm.vmin);
	float maxRange = std::max(uRange, vRange);
	int textureSize = mChessboardImage.getTextureReference().getWidth();
	ofVec2f outputRange(textureSize*uRange / maxRange, textureSize*vRange / maxRange);

	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		mesh.setTexCoord(i, mapVec2f(mesh.getTexCoord(i), ofVec2f(lscm.umin, lscm.vmin), ofVec2f(lscm.umax, lscm.vmax), ofVec2f(0, 0), outputRange));
	}
}

ofVec2f ofApp::mapVec2f(ofVec2f value, ofVec2f inputMin, ofVec2f inputMax, ofVec2f outputMin, ofVec2f outputMax, bool clamp) {
	float x = ofMap(value.x, inputMin.x, inputMax.x, outputMin.x, outputMax.x, clamp);
	float y = ofMap(value.y, inputMin.y, inputMax.y, outputMin.y, outputMax.y, clamp);

	return ofVec2f(x, y);
}