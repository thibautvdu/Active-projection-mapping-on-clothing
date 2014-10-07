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

		ofxKinectCommonBridge ofxKinect_;
		int kinectWidth_, kinectHeight_;

		ofxKinectProjectorToolkit kinectProjectorToolkit_;
		ofUtilities::ofVirtualWindow projectorWindow_;

		ofShader shader_;

		// HARDWARE HANDLERS	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Background learning
		ofxIntSlider bgLearningCycleGui_; int bgLearningCycleCount_; bool learntBg_;
		ofShortPixels depthBg_;
		cv::Mat cvDepthBg_;

		// Background segmentation
		ofImage bgMask_;
		cv::Mat cvBgMask_;

		// Folds detection
		ofImage normalsImg_;
		cv::Mat cvNormalsImg_;
		ofxIntSlider cannyThresh1_;
		ofxIntSlider cannyThresh2_;
		bool askFoldComputation_;
		ofPolyline foldAxis_;

		// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Point cloud
		ofxFloatSlider nearClipGui_;
		ofxFloatSlider farClipGui_;

		// Blob finder and tracker
		garmentAugmentation::kinect3dBlobDetector blobFinder_;
		bool blobFound_;

		// Garment object
		garmentAugmentation::interactiveGarment garment_;

		// Tracking mesh
		//garmentAugmentation::ofSemiImplicitActiveMesh m_blobMesh;
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
