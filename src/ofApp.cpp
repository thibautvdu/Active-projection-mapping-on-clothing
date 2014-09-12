#include "ofApp.h"

#include <stdlib.h>
#include "cvHelper.h"
#include "lscm.h"

/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::setup() {
	// HARDWARE INIT	/	/	/	/	/	/	/	/	/	/	/	/

	// Kinect
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

	// Projector
	mKinectProjectorToolkit.loadCalibration("calibration.xml");
	mProjectorWindow = ofUtilities::ofVirtualWindow(1920, 0, 1024, 768);

	// HARDWARE INIT	-	-	-	-	-	-	-	-	-	-	-	-


	// OPEN GL	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	ofDisableAlphaBlending();
	ofSetFrameRate(30);

	// OPEN GL	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// KINECT SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/	/

	// Background segmentation
	mOfSegmentedImg.setUseTexture(false); // Not meant to be displayed
	mOfSegmentedImg.allocate(mKinectColorImgWidth, mKinectColorImgHeight, OF_IMAGE_COLOR);
	mCvSegmentedImg = ofxCv::toCv(mOfSegmentedImg);
	mOfGarmentMask.allocate(mKinectColorImgWidth, mKinectColorImgHeight, OF_IMAGE_GRAYSCALE);
	mCvGarmentMask = ofxCv::toCv(mOfGarmentMask);

	// Cloth segmentation and contour detection
	mContourFinder.setAutoThreshold(false);
	mContourFinder.setFindHoles(false);
	mContourFinder.setInvert(false);
	mContourFinder.setSimplify(true);
	mContourFinder.setSortBySize(true);
	mContourFinder.setUseTargetColor(false);

	// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

	// Mesh
	mGarmentGeneratedMesh = ofDeformationTracking::ofSemiImplicitActiveMesh(12, 12);

	// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

	// Textures
	mChessboardImage.loadImage("chessboard.png");

	// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


	// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

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
	mGui.add(mMeshBoundaryWeight.setup("tracking mesh boundary weight", 0.8, 0.1, 2));
	mGui.add(mMeshDepthWeight.setup("tracking mesh depth weight", 0.6, 0.001, 2));
	mGui.add(mMeshAdaptationRate.setup("tracking mesh adaptation rate", 2, 0.1, 4));
	
	// Keys
	mPause = false;
	mSaveMesh = false;
	mAskRegeneration = false;

	// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-

	mOfGarmentMask.loadImage("mask.tif");
	mOfGarmentMask.setImageType(OF_IMAGE_GRAYSCALE);
}

void ofApp::update() {
	// EVENTS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	if (mSaveMesh) {
		mGarmentGeneratedMesh.save("export.ply");
		ofLog() << "Exported the 3D mesh";
		mSaveMesh = false;
	}
	if (mPause)
		return;
	if (abs(mGarmentGeneratedMesh.getBoundaryWeight() - mMeshBoundaryWeight) > 0.001) {
		mGarmentGeneratedMesh.setBoundaryWeight(mMeshBoundaryWeight);
	}
	if (abs(mGarmentGeneratedMesh.getDepthWeight() - mMeshDepthWeight) > 0.001) {
		mGarmentGeneratedMesh.setDepthWeight(mMeshDepthWeight);
	}
	if (abs(mGarmentGeneratedMesh.getAdaptationRate() - mMeshAdaptationRate) > 0.001) {
		mGarmentGeneratedMesh.setAdaptationRate(mMeshAdaptationRate);
	}

	// EVENTS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// HARDWARE	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	mOfxKinect.update();

	// HARDWARE	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// Model ROI
	mModelRoi = ofRectangle(mOfSegmentedImg.width, mOfSegmentedImg.height, 0, 0);
	int modelRoiXBottomRight = 0, modelRoiYBottomRight = 0;
	int modelBodyPixelsCount = 0; // Used to discriminate contours, more robust than bounding box if the model extend their arms, etc.

	// Segmentation from the skeleton
	if (/*mOfxKinect.isNewSkeleton()*/ 1) {
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
		if (/*modelBodyPixelsCount != 0*/ 1) {
			// Update the ROI information of the model
			//mModelRoi.width = modelRoiXBottomRight - mModelRoi.x;
			//mModelRoi.height = modelRoiYBottomRight - mModelRoi.y;
			//cv::Mat cvSegmentedImgRoi = mCvSegmentedImg(ofxCv::toCv(mModelRoi));
			//cv::Mat cvGarmentMaskRoi = mCvGarmentMask(ofxCv::toCv(mModelRoi));

			// OPENCV
			/*
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
			*/

			// Find the biggest correct contour
			mContourFinder.setMinArea(modelBodyPixelsCount*mGarmentBodyPercent / 100.f);
			//mContourFinder.findContours(cvGarmentMaskRoi);
			mContourFinder.findContours(mCvGarmentMask);
			if (mContourFinder.getContours().size() != 0) {
				mOfGarmentContour = mContourFinder.getPolyline(0);
				/*
				for (int i = 0; i < mOfGarmentContour.size(); ++i) {
					mOfGarmentContour[i] += mModelRoi.getTopLeft();
				}*/

				mOfGarmentMask.setColor(ofColor::black); // reset the mask as opencv only draw on the ROI
				//ofxCv::fillPoly(mContourFinder.getContour(0), cvGarmentMaskRoi);
				ofxCv::fillPoly(mContourFinder.getContour(0), mCvGarmentMask);

				// Update the ROI information of the garment
				mGarmentRoi = mOfGarmentContour.getBoundingBox();

				drawProjectorImage();
			}
			else {
				ofLog(OF_LOG_ERROR) << "Couldn't retrieve big enough contour";
				mOfGarmentContour.clear();
			}
		}
		else {
			ofLog(OF_LOG_ERROR) << "Found skeleton but couldn't detect garment";
			mOfGarmentContour.clear();
		}

		// Generate the mesh
		if (mOfGarmentContour.size() != 0) {
			if (!mGarmentGeneratedMesh.isGenerated() && mAskRegeneration) {
				//mGarmentGeneratedMesh.generateMesh(mOfGarmentContourModel);
				//computeNormals(mGarmentGeneratedMesh, true);
				//meshParameterizationLSCM(mGarmentGeneratedMesh);
				//mAskRegeneration = false;
			}
			else if (mGarmentGeneratedMesh.isGenerated()) {
				//mGarmentGeneratedMesh.updateMesh(mOfGarmentContourModel);
			}
		}
		mOfSegmentedImg.update();
		mOfGarmentMask.update();
	} // mOfxKinect.isNewSkeleton()
}

void ofApp::draw() {
	if (mOfxKinect.isFrameNew()) {

		// COMPUTER SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/

		ofBackground(ofColor::white);
		mProjectorWindow.background(ofColor::black);

		// Top Left
		mOfxKinect.draw(0, 0);

		ofSetColor(ofColor::blue);
		mOfGarmentContour.draw();
		ofSetColor(255);

		// Top Right
		mOfxKinect.drawDepth(mKinectColorImgWidth, 0);

		// Bottom Left
		mOfGarmentMask.draw(0, mKinectColorImgHeight);

		// Bottom Right
		/*
		if (mGarmentGeneratedMesh.hasVertices()) {
			glPointSize(1);

			//mEasyCam.begin();
			if (!mPause) {
				//mEasyCam.setTarget(mGarmentGeneratedMesh.getCentroid());
			}
			ofPushMatrix();
				ofEnableDepthTest();
				//mChessboardImage.bind();
				
				mGarmentGeneratedMesh.drawWireframe();
				//mChessboardImage.unbind();
				ofDisableDepthTest();
			ofPopMatrix();
			//mEasyCam.end();

		}*/

		// GUI
		mGui.draw();

		// COMPUTER SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-

		 
		// PROJECTOR SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/

		if (mOfGarmentContour.size() != 0) {
			mProjectorWindow.begin();
				ofPath projectorGarmentPath = ofUtilities::polylineToPath(projectorGarmentContour);
				projectorGarmentPath.setFilled(true);
				projectorGarmentPath.setFillColor(ofColor::red);
				projectorGarmentPath.draw();
			mProjectorWindow.end();
		}

		// PROJECTOR SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-
	}
}

/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

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
	else if (key == 'g') {
		mAskRegeneration = true;
	}
}

/* OF ROUTINES	-	-	-	-	-	-	-	-	-	-	-	-	-	*/


/* METHODS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::drawProjectorImage() {
	projectorGarmentContour.clear();
	ofVec2f projectorCoordinates;

	std::vector<ofVec3f> worldPoints;
	ofVec3f worldPoint;
	float averageDepth = 0;
	for (int i = 0; i < mOfGarmentContour.size(); ++i) {
		worldPoint = mOfxKinect.getWorldCoordinates(mOfGarmentContour[i].x, mOfGarmentContour[i].y);

		if (abs(worldPoint.z) > 0.1) {
			worldPoints.push_back(worldPoint);
			averageDepth += worldPoint.z;
		}
	}
	averageDepth /= worldPoints.size();

	for (int i = 0; i < worldPoints.size(); ++i) {
		if (abs(worldPoints[i].z - averageDepth) < 500) {
			projectorCoordinates = mKinectProjectorToolkit.getProjectedPoint(worldPoints[i]);

			projectorCoordinates.x = ofMap(projectorCoordinates.x, 0, 1, 0, mProjectorWindow.getWidth());
			projectorCoordinates.y = ofMap(projectorCoordinates.y, 0, 1, 0, mProjectorWindow.getHeight());

			projectorGarmentContour.addVertex(projectorCoordinates);
		}
	}
	projectorGarmentContour.close();
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
		mesh.setTexCoord(i, ofUtilities::mapVec2f(mesh.getTexCoord(i), ofVec2f(lscm.umin, lscm.vmin), ofVec2f(lscm.umax, lscm.vmax), ofVec2f(0, 0), outputRange));
	}
}

/* METHODS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	*/