#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include "ofMain.h"

#include <stdlib.h>
#include "ofxCv.h"
#include "ofxKinectCommonBridge.h"
#include "ofxGui.h"
#include "ofxKinectProjectorToolkit.h"
#include "ofxKinectBlobFinder.h"
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
		void toProjectorSpace(ofMesh& mesh);
		void meshParameterizationLSCM(ofMesh& mesh, int textureSize);


		/* VARIABLES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

		// HARDWARE HANDLERS	/	/	/	/	/	/	/	/	/	/	/	/

		ofxKinectCommonBridge mOfxKinect;
		int m_kinectWidth, m_kinectHeight;

		ofxKinectProjectorToolkit mKinectProjectorToolkit;
		ofUtilities::ofVirtualWindow mProjectorWindow;

		// HARDWARE HANDLERS	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Background learning
		ofxIntSlider m_bgLearningCycleGui; int m_bgLearningCycleCount; bool m_learntBg;
		ofShortPixels m_depthBg;
		cv::Mat m_cvDepthBg;

		// Background segmentation
		ofImage m_bgMask;
		cv::Mat m_cvBgMask;

		// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Point cloud
		ofxFloatSlider m_nearClipGui;
		ofxFloatSlider m_farClipGui;

		// Blob finder and tracker
		ofxKinectBlobFinder m_blobFinder;
		ofxKinectBlob m_modelBlob;
		bool m_blobFound;

		// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

		ofPolyline m_projectorGarmentContour;

		// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


		// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

		ofxPanel m_gui;

		// Keys
		bool m_askPause;
		bool m_askSaveMesh;
		bool m_askBgLearning;
		bool m_askBgExport;

		// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
