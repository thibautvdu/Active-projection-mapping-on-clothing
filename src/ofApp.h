#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include "ofMain.h"

#include <stdlib.h>
#include "ofxCv.h"
#include "ofxKinectCommonBridge.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {

	public:
		void setup();
		void update();
		void draw();
		void exit();

		// Kinect sensor
		ofxKinectCommonBridge mOfxKinect;
		KCBHANDLE mKcbKinect;
		int mKinectColorImgWidth, mKinectColorImgHeight;
		int mKinectDepthImgWidth, mKinectDepthImgHeight;

		// Background segmentation
		ofImage mOfSegmentedImg;
		cv::Mat mCvSegmentedImg;
		ofRectangle mModelRoi;
		
		// Cloth segmentation and contour detection
		ofImage mOfGarmentMask;
		cv::Mat mCvGarmentMask;
		ofxCv::ContourFinder mContourFinder;
		std::vector<cv::Point> mCvGarmentContour;
		ofPolyline mOfGarmentContour;
		ofRectangle mGarmentRoi;

		// Mesh generation
		ofMesh mGarmentPointOfCloud;

		// GUI
		ofxPanel mGui;
		ofxIntSlider mGarmentSegmentationLowH, mGarmentSegmentationLowS, mGarmentSegmentationLowV; // Cloth color segmentation low thresh
		ofxIntSlider mGarmentSegmentationHighH, mGarmentSegmentationHighS, mGarmentSegmentationHighV; // Cloth color segmentation high thresh
		ofxIntSlider mOpenKernelSize, mCloseKernelSize;
		ofxToggle mMorphoUseEllipse;
		ofxIntSlider mGarmentBodyPercent;
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
