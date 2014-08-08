#include <stdlib.h>
#include "ofApp.h"
#include "cvHelper.h"

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

	// Allocation
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

	// OpenGL
	ofDisableAlphaBlending();
	ofSetFrameRate(30);

	// GUI
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
}

//--------------------------------------------------------------
void ofApp::update() {
	mOfxKinect.update();

	// Model ROI
	ofRectangle modelRoi(mOfSegmentedImg.width, mOfSegmentedImg.height, 0, 0);
	int modelRoiXBottomRight = 0, modelRoiYBottomRight = 0;
	int modelBodyPixelsCount = 0; // Used to discriminate contours, more robust than bounding box if the model extend their arms, etc.

	// Segmentation from the skeleton
	if (mOfxKinect.isNewSkeleton()) {
		NUI_DEPTH_IMAGE_PIXEL * pNuiDepthPixel = mOfxKinect.getNuiMappedDepthPixelsRef(); // The kinect segmentation map

		for (int x = 0; x < mOfSegmentedImg.width; ++x) {
			for (int y = 0; y < mOfSegmentedImg.height; ++y) {
				USHORT playerId = (pNuiDepthPixel + x + y*mOfSegmentedImg.width)->playerIndex;

				if (playerId != 0) {
					ofColor bgra = mOfxKinect.getColorPixelsRef().getColor(x, y);
					mOfSegmentedImg.setColor(x, y, ofColor(bgra.b, bgra.g, bgra.r));

					// Update the ROI
					modelRoi.x = (x < modelRoi.x) ? x : modelRoi.x;
					modelRoi.y = (y < modelRoi.y) ? y : modelRoi.y;
					modelRoiXBottomRight = (x > modelRoiXBottomRight) ? x : modelRoiXBottomRight;
					modelRoiYBottomRight = (y > modelRoiYBottomRight) ? y : modelRoiYBottomRight;
					modelBodyPixelsCount++;
				}
				else {
					mOfSegmentedImg.setColor(x, y, ofColor::black);
				}
			}
		}
	}
	
	// Cloth segmentation and contour acquisition
	if (modelBodyPixelsCount != 0) {
		// Update the ROI information
		modelRoi.width = modelRoiXBottomRight - modelRoi.x;
		modelRoi.height = modelRoiYBottomRight - modelRoi.y;
		mOfGarmentMask.setColor(ofColor::black); // reset the mask as opencv only draw on the ROI
		cv::Mat cvSegmentedImgRoi = mCvSegmentedImg(ofxCv::toCv(modelRoi));
		cv::Mat cvGarmentMaskRoi = mCvGarmentMask(ofxCv::toCv(modelRoi));

		// OPENCV
		// Color segmentation
		cv::Mat mCvSegmentedImgHsv;
		cv::cvtColor(cvSegmentedImgRoi, mCvSegmentedImgHsv, CV_RGB2HSV);
		cv::inRange(mCvSegmentedImgHsv, cv::Scalar(mGarmentSegmentationLowH, mGarmentSegmentationLowS, mGarmentSegmentationLowV), cv::Scalar(mGarmentSegmentationHighH, mGarmentSegmentationHighS, mGarmentSegmentationHighV), cvGarmentMaskRoi);

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
			std::vector<cv::Point> clothContour = mContourFinder.getContour(0);
			//cv::fillPoly(mCvGarmentMask, clothContour, cv::Scalar(100, 0, 100));
		}
		else {
			ofLog(OF_LOG_ERROR) << "Couldn't retrieve contour of the garment";
		}
	}
	mOfSegmentedImg.update();
	mOfGarmentMask.update();
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (mOfxKinect.isFrameNew()) {
		ofBackground(0);

		mOfxKinect.draw(0, 0);
		mOfxKinect.drawDepth(640, 0);
		mOfSegmentedImg.draw(0, 480);
		if (mOfGarmentMask.isAllocated()) {
			mOfGarmentMask.draw(640, 480);
		}
	}
	mGui.draw();
}

//--------------------------------------------------------------
void ofApp::exit() {
	mOfxKinect.stop();
}