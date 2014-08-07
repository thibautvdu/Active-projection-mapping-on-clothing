#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinectCommonBridge.h"
#include "ofxGui.h"

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
		ofImage mOfClothMask;
		cv::Mat mCvClothMask;

		// GUI
		ofxPanel mGui;
		ofxIntSlider mClothSegmentationLowH, mClothSegmentationLowS, mClothSegmentationLowV; // Cloth color segmentation low thresh
		ofxIntSlider mClothSegmentationHighH, mClothSegmentationHighS, mClothSegmentationHighV; // Cloth color segmentation high thresh
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
