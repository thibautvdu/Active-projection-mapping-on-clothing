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

		void mousePressed(int x, int y, int button);

		void generateMesh(cv::Mat& maskImage, const vector<cv::Point>& contour, ofMesh& mesh, int step = 1, ofPoint offset = ofPoint(0,0));
		void ofApp::computeNormals(ofMesh& mesh, bool bNormalize);

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
		std::vector<cv::Point> mCvGarmentContourModelRoiRel;
		ofPolyline mOfGarmentContourModelRoiRel;
		ofRectangle mGarmentRoi;

		// Mesh generation
		ofMesh mGarmentGeneratedMesh;

		// GUI
		ofxPanel mGui;
		ofxIntSlider mGarmentSegmentationLowH, mGarmentSegmentationLowS, mGarmentSegmentationLowV; // Cloth color segmentation low thresh
		ofxIntSlider mGarmentSegmentationHighH, mGarmentSegmentationHighS, mGarmentSegmentationHighV; // Cloth color segmentation high thresh
		ofxIntSlider mOpenKernelSize, mCloseKernelSize; // Morphological operators
		ofxToggle mMorphoUseEllipse; // Morphological operators
		ofxIntSlider mGarmentBodyPercent; // Contour 
		ofxIntSlider mDepthSmoothingKernelSize;
		ofxIntSlider mDepthSmoothingSigma;
		ofxIntSlider mMeshGenerationStep;
		ofEasyCam mEasyCam;
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
