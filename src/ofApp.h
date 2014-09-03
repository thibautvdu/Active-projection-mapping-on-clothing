#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include "ofMain.h"

#include <stdlib.h>
#include "ofxCv.h"
#include "ofxKinectCommonBridge.h"
#include "ofxGui.h"
#include "ofSemiImplicitActiveMesh.h"

class ofApp : public ofBaseApp {

	public:
		void setup();
		void update();
		void draw();
		void exit();

		void mousePressed(int x, int y, int button);
		void keyPressed(int key);

		void computeNormals(ofMesh& mesh, bool bNormalize);
		void meshParameterizationLSCM(ofMesh& mesh);
		ofVec2f ofApp::mapVec2f(ofVec2f value, ofVec2f inputMin, ofVec2f inputMax, ofVec2f outputMin, ofVec2f outputMax, bool clamp = false);

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
		ofPolyline mOfGarmentContourModel;
		ofRectangle mGarmentRoi;

		// Mesh generation
		ofDeformationTracking::ofSemiImplicitActiveMesh mGarmentGeneratedMesh;
		bool mAskRegeneration;

		// Textures
		ofImage mChessboardImage;

		// GUI
		ofxPanel mGui;
		ofxIntSlider mGarmentSegmentationLowH, mGarmentSegmentationLowS, mGarmentSegmentationLowV; // Cloth color segmentation low thresh
		ofxIntSlider mGarmentSegmentationHighH, mGarmentSegmentationHighS, mGarmentSegmentationHighV; // Cloth color segmentation high thresh
		ofxIntSlider mOpenKernelSize, mCloseKernelSize; // Morphological operators
		ofxToggle mMorphoUseEllipse; // Morphological operators
		ofxIntSlider mGarmentBodyPercent; // Contour 

		ofxFloatSlider mMeshBoundaryWeight;
		ofxFloatSlider mMeshDepthWeight;
		ofxFloatSlider mMeshAdaptationRate;

		ofEasyCam mEasyCam;

		// Keys
		bool mPause;
		bool mSaveMesh;
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
