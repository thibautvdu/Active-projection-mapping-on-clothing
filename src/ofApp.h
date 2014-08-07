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
		ofImage mOfGarmentMask;
		cv::Mat mCvGarmentMask;

		ofxCv::ContourFinder mContourFinder;

		// GUI
		ofxPanel mGui;
		ofxIntSlider mGarmentSegmentationLowH, mGarmentSegmentationLowS, mGarmentSegmentationLowV; // Cloth color segmentation low thresh
		ofxIntSlider mGarmentSegmentationHighH, mGarmentSegmentationHighS, mGarmentSegmentationHighV; // Cloth color segmentation high thresh
		ofxIntSlider mOpenKernelSize, mCloseKernelSize;
		ofxToggle mMorphoUseEllipse;
		ofxIntSlider mGarmentBodyPercent;
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
