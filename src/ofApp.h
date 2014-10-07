#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include "ofMain.h"

#include <stdlib.h>
#include "ofxCv.h"
#include "ofxKinectCommonBridge.h"
#include "ofxGui.h"

#include "ofxKinectProjectorToolkit.h"
#include "ofUtilities.h"
#include "kinect3dBlobDetector.h"
#include "interactiveGarment.h"

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
		void toProjectorSpace(ofMesh& mesh);
		void meshParameterizationLSCM(ofMesh& mesh, int textureSize);
		void markFolds();


		/* VARIABLES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

		// HARDWARE HANDLERS	/	/	/	/	/	/	/	/	/	/	/	/

		ofxKinectCommonBridge m_ofxKinect;
		int m_kinectWidth, m_kinectHeight;

		ofxKinectProjectorToolkit m_kinectProjectorToolkit;
		ofUtilities::ofVirtualWindow m_projectorWindow;

		ofShader m_shader;

		// HARDWARE HANDLERS	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Background learning
		ofxIntSlider m_bgLearningCycleGui; int m_bgLearningCycleCount; bool m_learntBg;
		ofShortPixels m_depthBg;
		cv::Mat m_cvDepthBg;

		// Background segmentation
		ofImage m_bgMask;
		cv::Mat m_cvBgMask;

		// Folds detection
		ofImage m_normalsImg;
		cv::Mat m_cvNormalsImg;
		ofxIntSlider m_cannyThresh1;
		ofxIntSlider m_cannyThresh2;
		bool m_askFoldComputation;
		ofPolyline m_foldAxis;

		// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Point cloud
		ofxFloatSlider m_nearClipGui;
		ofxFloatSlider m_farClipGui;

		// Blob finder and tracker
		garmentAugmentation::kinect3dBlobDetector m_blobFinder;
		garmentAugmentation::simple3dBlob * m_modelBlob;
		bool m_blobFound;

		// Garment object
		garmentAugmentation::interactiveGarment garment_;

		// Tracking mesh
		//garmentAugmentation::ofSemiImplicitActiveMesh m_blobMesh;
		//ofxFloatSlider m_adaptationRate, m_boundaryWeight, m_depthWeight;

		// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

		ofImage m_chessboardImage;

		// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


		// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

		ofxPanel m_gui;
		ofEasyCam m_easyCam;

		// Keys
		bool m_askPause;
		bool m_askSaveAssets;
		bool m_askBgLearning;
		bool m_askBgExport;

		// FPS
		ofxLabel m_fpsGui;

		// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
