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
		KCBHANDLE mKcbKinect;
		int mKinectColorImgWidth, mKinectColorImgHeight;
		int mKinectDepthImgWidth, mKinectDepthImgHeight;

		ofImage mOfSegmentedImg;
		cv::Mat mCvSegmentedImg;
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
