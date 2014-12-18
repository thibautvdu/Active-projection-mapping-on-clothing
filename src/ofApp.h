// This is the openframework's application file. It manages the view and contain
// the main logic of the program, that is the equivalent to the process diagram
// available in the shared google doc drawing. Process are limited to simple
// operations (background removal, etc.) and call to other modules containing
// the most of the work and algorithms

#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include <stdlib.h>

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "opencv2/cuda.hpp"

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
		void keyPressed(int key);

		/* OF ROUTINES	-	-	-	-	-	-	-	-	-	-	-	-	-	*/



		/* METHODS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

		// Approximate an UV map for the given mesh and given texture size
		void ParameterizationLSCM(const int textureSize, ofMesh& mesh);

		// Compute the folds at a time t for the current point cloud and update 
		// the interactive garment object consequently
		void DetectFolds();

		/* METHODS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	*/



		/* VARIABLES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

		// HARDWARE HANDLERS //

		ofxKinectCommonBridge ofxKinect_;
		static const int k_kinect_width_, k_kinect_height_;

		static const int k_screen_width_, k_screen_height_, k_projector_width_, k_projector_height_;
		of_utilities::VirtualWindow projectorWindow_;
		ofxKinectProjectorToolkit kinectProjectorToolkit_;

		// !HARDWARE HANDLERS //


		// KINECT SCREEN SPACE //

		// Background learning
		ofxIntSlider gui_bg_learning_cycle_;
		int bgLearningCycleCount_;
		bool learntBg_;
		ofShortPixels of_depth_bg_; cv::Mat cv_depth_bg_; cv::cuda::GpuMat cv_depth_bg_gpu_;

		// Background segmentation
		ofImage of_fg_mask_; cv::Mat cv_fg_mask_; cv::cuda::GpuMat cv_fg_mask_gpu_;

		// Smoothing on the depth map
		ofxIntSlider gui_gaussian_size_;
		ofxFloatSlider gui_gaussian_sigma_;

		// !KINECT SCREEN SPACE //


		// KINECT WORLD SPACE //

		static const float k_to_world_units_; // scaling between the kinect values (mm) and world values (m)

		// Euclidian cluster segmentation
		garment_augmentation::blob_detection::kinect3dBlobDetector euclidian_cluster_segmentation_;
		bool blobFound_;

		// Interactive garment object, containing the mesh, the folds, the animations, etc.
		garment_augmentation::garment::InteractiveGarment garment_;

		// Folds detection
		ofxFloatSlider gui_crater_deformation_thresh_;
		bool askFoldComputation_;
		//garment_augmentation::garment::ThreadedDeformationDetector threaded_deformation_detector_; // non-cuda multithread detection
		//ofxIntSlider gui_detector_num_threads_; // non-cuda multithread detection
		garment_augmentation::math::RansacKalman3dSegmentTracker ransac_kalman_tracker_; // the folds tracking (after the craters detection)
		ofxFloatSlider gui_ransac_distance_thresh_;
		ofxIntSlider gui_ransac_points_num_thresh_;
		ofxFloatSlider gui_fold_width_;

		ofxFloatSlider gui_kalman_process_noise_;
		ofxFloatSlider gui_kalman_measurement_noise_;
		ofxFloatSlider gui_kalman_post_error_;
		ofxToggle gui_using_velocity_;

		// Tracking mesh
		garment_augmentation::surface_tracking::ofSemiImplicitActiveMesh active_mesh_;
		ofxFloatSlider gui_active_mesh_adaptation_rate_, gui_active_mesh_boundary_weight_, gui_active_mesh_depth_weight_;

		// !KINECT WORLD SPACE //


		// PROJECTOR SCREEN SPACE //

		ofImage chessboardImage_; // Used to test the retexturing when using the energy minimization mesh and LSCM

		// !PROJECTOR SCREEN SPACE //


		// GUI/CONTROLS //

		ofxPanel gui_;
		ofEasyCam easyCam_;

		// Keys
		bool askPause_;
		bool askSaveAssets_;
		bool askBgLearning_;
		bool askBgExport_;
		bool askedGeneration_;

		// FPS
		ofxLabel gui_fps_;

		// !GUI/CONTROLS //

		/* VARIABLES	-	-	-	-	-	-	-	-	-	-	-	-	-	*/
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
