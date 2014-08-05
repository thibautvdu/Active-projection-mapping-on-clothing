#include <stdlib.h>
#include "ofApp.h"
#include "KinectBackgroundRemoval.h"

//--------------------------------------------------------------
void ofApp::setup() {
	bool initSensor = mOfxKinect.initSensor();
	if (!initSensor) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's sensor";
		exit();
	}

	bool initStreams = mOfxKinect.initColorStream(640,480) && mOfxKinect.initDepthStream(640,480,true);
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

	ofDisableAlphaBlending();
	ofSetFrameRate(30);
}

//--------------------------------------------------------------
void ofApp::update() {
	mOfxKinect.update();

	if (mOfxKinect.isNewSkeleton()) {
		// Search for the first non-empty skeleton
		vector<Skeleton> skeletons = mOfxKinect.getSkeletons();
		for (int i = 0; i < skeletons.size(); ++i) {
			if (skeletons[i].find(NUI_SKELETON_POSITION_HEAD) != skeletons[i].end()) {
				/*
				INuiSensor* nuiKinect = &mOfxKinect.getNuiSensor();
				INuiBackgroundRemovedColorStream* backgroundLessColorStream;
				NuiCreateBackgroundRemovedColorStream(nuiKinect, &backgroundLessColorStream);
				*/
				
				// Construct an openCV convex hull from the skeleton
				Skeleton modelSkeleton = skeletons[i];
				std::vector<cv::Point> modelImgBonePoints;
				for (Skeleton::iterator it = modelSkeleton.begin(); it != modelSkeleton.end(); ++it) {
					modelImgBonePoints.push_back(cv::Point(it->second.getScreenPosition().x, it->second.getScreenPosition().y));
				}
				std::vector<int> hullIndices;
				cv::convexHull(modelImgBonePoints, hullIndices);

				mModelHullOfInterest.clear();
				for (int i = 0; i < hullIndices.size(); ++i) {
					mModelHullOfInterest.push_back(modelImgBonePoints[hullIndices[i]]);
				}
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (mOfxKinect.isFrameNew()) {
		ofBackground(0);

		mOfxKinect.draw(0, 0);

		ofMesh vertexData;
		for (int i = 0; i < mModelHullOfInterest.size(); ++i) {
			vertexData.addVertex(ofVec3f(mModelHullOfInterest[i].x, mModelHullOfInterest[i].y, 0));
		}

		ofSetColor(255, 0, 0);
		vertexData.setMode(OF_PRIMITIVE_LINE_LOOP);
		vertexData.draw();
		ofSetColor(255);

		mOfxKinect.drawDepth(640, 0);
	}
}

//--------------------------------------------------------------
void ofApp::exit() {

}