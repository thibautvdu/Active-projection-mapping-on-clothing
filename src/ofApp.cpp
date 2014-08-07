#include <stdlib.h>
#include "ofApp.h"
#include "cvHelper.h"

//--------------------------------------------------------------
void ofApp::setup() {
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

	mOfSegmentedImg.allocate(mKinectColorImgWidth, mKinectColorImgHeight, OF_IMAGE_COLOR);
	mCvSegmentedImg = ofxCv::toCv(mOfSegmentedImg);
	mOfClothMask.allocate(mKinectColorImgWidth, mKinectColorImgHeight, OF_IMAGE_GRAYSCALE);
	mCvClothMask = ofxCv::toCv(mOfClothMask);

	ofDisableAlphaBlending();
	ofSetFrameRate(30);

	// GUI
	mGui.setup();
	mGui.add(mClothSegmentationLowH.setup("low hue thresh", 70, 0, 179));
	mGui.add(mClothSegmentationLowS.setup("low saturation thresh", 0, 0, 255));
	mGui.add(mClothSegmentationLowV.setup("low value thresh", 120, 0, 255));
	mGui.add(mClothSegmentationHighH.setup("high hue thresh", 120, 0, 179));
	mGui.add(mClothSegmentationHighS.setup("high saturation thresh", 95, 0, 255));
	mGui.add(mClothSegmentationHighV.setup("high value thresh", 255, 0, 255));
}

//--------------------------------------------------------------
void ofApp::update() {
	mOfxKinect.update();

	if (mOfxKinect.isNewSkeleton()) {
		NUI_DEPTH_IMAGE_PIXEL * pNuiDepthPixel = mOfxKinect.getNuiMappedDepthPixelsRef(); // The kinect segmentation map
		
		int modelPixelCount = 0;
		ofPoint modelRoiPointA(mOfSegmentedImg.width, mOfSegmentedImg.height);
		ofPoint modelRoiPointD(0, 0);
		for (int x = 0; x < mOfSegmentedImg.width; ++x) {
			for (int y = 0; y < mOfSegmentedImg.height; ++y) {
				USHORT playerId = (pNuiDepthPixel + x + y*mOfSegmentedImg.width)->playerIndex;

				if (playerId != 0) {
					ofColor bgra = mOfxKinect.getColorPixelsRef().getColor(x, y);
					mOfSegmentedImg.setColor(x, y, ofColor(bgra.b,bgra.g,bgra.r));
					modelPixelCount++;
					// Update the ROI
					modelRoiPointA.x = (x < modelRoiPointA.x) ? x : modelRoiPointA.x;
					modelRoiPointA.y = (y < modelRoiPointA.y) ? y : modelRoiPointA.y;
					modelRoiPointD.x = (x > modelRoiPointD.x) ? x : modelRoiPointD.x;
					modelRoiPointD.y = (y > modelRoiPointD.y) ? y : modelRoiPointD.y;
				}
				else {
					mOfSegmentedImg.setColor(x, y, ofColor::black);
				}
			}
		}

		// OPENCV
		/*if (mCvSegmentedImage.type() != CV_8UC4) {
		ofLog(OF_LOG_FATAL_ERROR) << "Expected the kinect's color image to be 8 bytes x 4 channels";
		exit();
		}*/
		// Color segmentation
		cv::Mat mCvSegmentedImgHsv;
		cv::cvtColor(mCvSegmentedImg, mCvSegmentedImgHsv, CV_RGB2HSV);
		cv::inRange(mCvSegmentedImgHsv, cv::Scalar(mClothSegmentationLowH, mClothSegmentationLowS, mClothSegmentationLowV), cv::Scalar(mClothSegmentationHighH, mClothSegmentationHighS, mClothSegmentationHighV), mCvClothMask);

		//morphological opening (remove small objects from the foreground)
		cv::erode(mCvClothMask, mCvClothMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(mCvClothMask, mCvClothMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		cv::dilate(mCvClothMask, mCvClothMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::erode(mCvClothMask, mCvClothMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	}
	mOfSegmentedImg.update();
	mOfClothMask.update();
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (mOfxKinect.isFrameNew()) {
		ofBackground(0);

		mOfxKinect.draw(0, 0);
		mOfxKinect.drawDepth(640, 0);
		mOfSegmentedImg.draw(0, 480);
		mOfClothMask.draw(640, 480);
	}
	mGui.draw();
}

//--------------------------------------------------------------
void ofApp::exit() {
	mOfxKinect.stop();
}