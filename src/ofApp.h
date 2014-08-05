#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinectCommonBridge.h"

class ofApp : public ofBaseApp {

	public:
		void setup();
		void update();
		void draw();
		void exit();

		ofxKinectCommonBridge mOfxKinect;
		std::vector<cv::Point> mModelHullOfInterest;
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
