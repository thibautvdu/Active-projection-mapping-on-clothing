#include <stdlib.h>
#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	mKcbKinect = mOfxKinect.getHandle();

	bool initSensor = mOfxKinect.initSensor();
	if (!initSensor) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's sensor";
		exit();
	}

	bool initStreams = mOfxKinect.initColorStream(640, 480) && mOfxKinect.initDepthStream(640, 480, false, true);
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

	mOfSegmentedImage.allocate(640, 480, OF_IMAGE_COLOR);

	ofDisableAlphaBlending();
	ofSetFrameRate(30);
}

//--------------------------------------------------------------
void ofApp::update() {
	mOfxKinect.update();
	
	if (mOfxKinect.isNewSkeleton()) {
		NUI_DEPTH_IMAGE_PIXEL * pNuiDepthPixel = mOfxKinect.getNuiMappedDepthPixelsRef();

		ofPixels ofxKinectColorPixels = mOfxKinect.getColorPixelsRef();
		mOfSegmentedImage.setColor(ofColor::black);
		ofPixels segmentedImagePixels = mOfSegmentedImage.getPixelsRef();
		for (int x = 0; x < 640; x++) {
			for (int y = 0; y < 480; y++) {
				USHORT playerId = (pNuiDepthPixel + x + y*640)->playerIndex;

				if (playerId != 0) {
					segmentedImagePixels.setColor(x, y, ofxKinectColorPixels.getColor(x,y));
				}
			}
		}
		mOfSegmentedImage.setFromPixels(segmentedImagePixels);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (mOfxKinect.isFrameNew()) {
		ofBackground(0);

		mOfxKinect.draw(0, 0);
		mOfxKinect.drawDepth(640, 0);
		mOfSegmentedImage.draw(0, 480);
	}
}

//--------------------------------------------------------------
void ofApp::exit() {

}