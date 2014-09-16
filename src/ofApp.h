#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include "ofMain.h"

#include <stdlib.h>
#include "ofxCv.h"
#include "ofxKinectCommonBridge.h"
#include "ofxGui.h"
#include "ofxKinectProjectorToolkit.h"
#include "ofSemiImplicitActiveMesh.h"
#include "ofUtilities.h"

class ofApp : public ofBaseApp {

	public:

		/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/
		void setup();
		void update();
		void draw();
		void exit();

		/* OF INPUTS	/	/	/	/	/	/	/	/	/	/	/	/	/	*/
		void mousePressed(int x, int y, int button);
		void keyPressed(int key);

		/* METHODS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	*/
		void drawProjectorImage();
		void ofApp::toProjectorSpace(ofMesh& mesh);
		void meshParameterizationLSCM(ofMesh& mesh);


		/* VARIABLES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

		// HARDWARE HANDLERS	/	/	/	/	/	/	/	/	/	/	/	/

		ofxKinectCommonBridge mOfxKinect;
		int mKinectColorImgWidth, mKinectColorImgHeight;
		int mKinectDepthImgWidth, mKinectDepthImgHeight;

		ofxKinectProjectorToolkit mKinectProjectorToolkit;
		ofUtilities::ofVirtualWindow mProjectorWindow;

		// HARDWARE HANDLERS	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Background segmentation
		ofImage mOfSegmentedImg;
		cv::Mat mCvSegmentedImg;
		ofRectangle mModelRoi;

		// Cloth segmentation and contour detection
		ofImage mOfGarmentMask;
		cv::Mat mCvGarmentMask;
		ofRectangle mGarmentRoi;

		ofxCv::ContourFinder mContourFinder;
		ofPolyline mOfGarmentContour;

		// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Mesh generation
		ofDeformationTracking::ofSemiImplicitActiveMesh mGarmentGeneratedMesh;

		// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

		ofPolyline projectorGarmentContour;

		// Textures
		ofImage mChessboardImage;

		// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


		// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

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
		bool mAskRegeneration;

		// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
