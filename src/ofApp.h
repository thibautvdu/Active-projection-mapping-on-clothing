#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include <stdlib.h>

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"

#include "ofxKinectCommonBridge.h"
#include "ofxKinectProjectorToolkit.h"
#include "of_utilities.h"
#include "kinect3dBlobDetector.h"
#include "interactive_garment.h"
#include "ransac_kalman_segments.h"
#include "threaded_deformation_detector.h"
#include "ofSemiImplicitActiveMesh.h"

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

		// Smoothing
		ofxIntSlider gaussian_size_;
		ofxFloatSlider gaussian_sigma_;

		// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


		// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

		// Scaling
		static const float k_to_world_units_;

		// Blob finder and tracker
		garment_augmentation::blob_detection::kinect3dBlobDetector blobFinder_;
		bool blobFound_;

		// Garment object
		garment_augmentation::garment::InteractiveGarment garment_;

		// Folds detection
		ofxFloatSlider fold_deformation_thresh_;
		ofxFloatSlider fold_deformation_thresh_2_;

		bool askFoldComputation_;
		garment_augmentation::garment::ThreadedDeformationDetector threaded_deformation_detector_;
		garment_augmentation::math::RansacKalman3dSegmentTracker ransac_kalman_tracker_;
		uchar numFolds_;
		ofxFloatSlider fold_distance_thresh_;
		ofxIntSlider fold_points_num_thresh_;
		ofxFloatSlider fold_width_;
		ofxIntSlider deformation_detector_num_threads_;

		ofxFloatSlider kalman_process_noise_;
		ofxFloatSlider kalman_measurement_noise_;
		ofxFloatSlider kalman_post_error_;
		ofxToggle using_velocity_;

		// Tracking mesh
		garment_augmentation::surface_tracking::ofSemiImplicitActiveMesh active_mesh_;
		ofxFloatSlider active_mesh_adaptation_rate_, active_mesh_boundary_weight_, active_mesh_depth_weight_;

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
		bool askedGeneration_;

		// FPS
		ofxLabel fpsGui_;

		// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
