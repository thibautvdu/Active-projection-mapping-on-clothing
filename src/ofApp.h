#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include <stdlib.h>

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxUbo.h"

#include "ofxKinectCommonBridge.h"
#include "ofxKinectProjectorToolkit.h"
#include "of_utilities.h"
#include "kinect3dBlobDetector.h"
#include "interactive_garment.h"

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
		void meshParameterizationLSCM(const int textureSize, ofMesh& mesh);
		void detectFolds();


		/* VARIABLES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

		// HARDWARE HANDLERS	/	/	/	/	/	/	/	/	/	/	/	/

		ofxKinectCommonBridge ofxKinect_;
		static const int kinectWidth_, kinectHeight_;

		static const int screenWidth_, screenHeight_, projectorWidth_, projectorHeight_;
		ofxKinectProjectorToolkit kinectProjectorToolkit_;
		of_utilities::VirtualWindow projectorWindow_;

		// HARDWARE HANDLERS	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Background learning
		ofxIntSlider bgLearningCycleGui_; int bgLearningCycleCount_; bool learntBg_;
		ofShortPixels depthBg_;
		cv::Mat cvDepthBg_;

		// Background segmentation
		ofImage bgMask_;
		cv::Mat cvBgMask_;

		// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Scaling
		static const float toWorldUnits_;

		// Blob finder and tracker
		garment_augmentation::blob_detection::kinect3dBlobDetector blobFinder_;
		bool blobFound_;

		// Garment object
		garment_augmentation::garment::InteractiveGarment garment_;

		// Folds detection
		ofxFloatSlider deformationThres_;
		bool askFoldComputation_;
		uchar numFolds_;

		// Tracking mesh
		//garment_augmentation::ofSemiImplicitActiveMesh m_blobMesh;
		//ofxFloatSlider m_adaptationRate, m_boundaryWeight, m_depthWeight;

		// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

		ofImage chessboardImage_;

		// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


		// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

		ofxPanel gui_;
		ofEasyCam easyCam_;

		// Keys
		bool askPause_;
		bool askSaveAssets_;
		bool askBgLearning_;
		bool askBgExport_;

		// FPS
		ofxLabel fpsGui_;

		// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
