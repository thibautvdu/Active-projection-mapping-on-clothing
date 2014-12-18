#include "ofApp.h"

#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/core/cuda_stream_accessor.hpp"

#include "cuda_accelerated_deformation_detection.h"
#include "lscm.h"
#include "flying_lights.h"
#include "contour_vector_field.h"
#include "fold_video_texture.h"


/* CONSTANTS	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

// HARDWARE HANDLERS //

const int ofApp::k_screen_width_ = 1920, ofApp::k_screen_height_ = 1080;
const int ofApp::k_projector_width_ = 1024, ofApp::k_projector_height_ = 768;
const int ofApp::k_kinect_width_ = 640, ofApp::k_kinect_height_ = 480;

// !HARDWARE HANDLERS //


// KINECT WORLD SPACE //

const float ofApp::k_to_world_units_ = 0.001; // mm to meters

// !KINECT WORLD SPACE //

/* CONSTANTS	-	-	-	-	-	-	-	-	-	-	-	-	-	*/



/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::setup() {
	// HARDWARE INIT //

	// Kinect
	bool initSensor = ofxKinect_.initSensor();
	if (!initSensor) {
		ofLogFatalError("ofApp::setup") << "Couldn't init the kinect's sensor";
		exit();
	}

	bool initStreams = ofxKinect_.initColorStream(k_kinect_width_, k_kinect_height_) && ofxKinect_.initDepthStream(k_kinect_width_, k_kinect_height_, false, true);
	if (!initStreams) {
		ofLogFatalError("ofApp::setup") << "Couldn't init the kinect's color and/or depth streams";
		exit();
	}

	bool kinectStarted = ofxKinect_.start();
	if (!kinectStarted) {
		ofLogFatalError("ofApp::setup") << "Couldn't start the kinect";
		exit();
	}

	// Projector
	kinectProjectorToolkit_.loadCalibration("cal.xml", k_projector_width_, k_projector_height_);
	projectorWindow_ = of_utilities::VirtualWindow(k_screen_width_, 0, k_projector_width_, k_projector_height_);

	// !HARDWARE INIT //


	// OPENGL //

	ofDisableAlphaBlending();
	ofSetFrameRate(50);

	// !OPENGL //


	// KINECT SCREEN SPACE //

	// Background learning
	bgLearningCycleCount_ = 0;
	askBgLearning_ = false;
	learntBg_ = false;

	// Background segmentation
	of_fg_mask_.allocate(k_kinect_width_,k_kinect_height_,OF_IMAGE_GRAYSCALE);
	cv_fg_mask_ = ofxCv::toCv(of_fg_mask_);

	// !KINECT SCREEN SPACE //


	// KINECT WORLD SPACE //

	// Euclidian cluster segmentation
	blobFound_ = false;
	euclidian_cluster_segmentation_.init(&ofxKinect_, k_kinect_width_, k_kinect_height_); // standarized coordinate system: z in the direction of gravity
	//euclidian_cluster_segmentation_.setResolution(garment_augmentation::blob_detection::BF_HIGH_RES); // it is possible to use higher resolution, but too slow
	euclidian_cluster_segmentation_.setScale(ofVec3f(k_to_world_units_));

	// Fold processing
	//threaded_deformation_detector_.SetTargetGarment(&garment_); // if using the non CUDA algorithm
	ransac_kalman_tracker_.set_segments_life_time(0.5); // life time of the folds
	askFoldComputation_ = false;

	// Animations
	std::unique_ptr<garment_augmentation::garment::Animation> lightsEffect(new garment_augmentation::garment::FlyingLights());
	garment_.AddAnimation(std::move(lightsEffect));
	//std::unique_ptr<garment_augmentation::garment::Animation> contourEffect(new garment_augmentation::garment::ContourVectorField());
	//garment_.AddAnimation(std::move(contourEffect));
	//std::unique_ptr<garment_augmentation::garment::Animation> videoEffect(new garment_augmentation::garment::FoldVideoTexture("videos/halluc_edit.mp4"));
	//garment_.AddAnimation(std::move(videoEffect));

	// !KINECT WORLD SPACE //


	// PROJECTOR SCREEN SPACE //

	chessboardImage_.loadImage("chessboard.png");

	// !PROJECTOR SCREEN SPACE //


	// GUI/CONTROLS //

	gui_.setup();
	gui_.add(new ofxLabel(std::string("BACKGROUND LEARNING")));
	gui_.add(gui_bg_learning_cycle_.setup("nb iterations", 10, 1, 50));
	gui_.add(new ofxLabel(std::string("GAUSSIAN SMOOTHING")));
	gui_.add(gui_gaussian_size_.setup("size", 5, 3, 9));
	gui_.add(gui_gaussian_sigma_.setup("sigma", 2.5, 0, 3));
	gui_.add(new ofxLabel(std::string("DEFORMATION DETECTION")));
	// gui_.add(gui_detector_num_threads_.setup("num of threads", 6, 1, 12)); // non-cuda multithreads detection
	gui_.add(gui_crater_deformation_thresh_.setup("crater thresh", 0.015, 0.00, 0.03));
	gui_.add(new ofxLabel(std::string("FOLD TRACKING")));
	gui_.add(gui_ransac_distance_thresh_.setup("distance thresh", 0.05, 0.01, 0.08));
	gui_.add(gui_ransac_points_num_thresh_.setup("nb points thresh", 7, 5, 20));
	gui_.add(gui_fold_width_.setup("fold's width",0.055,0.0,0.1));
	gui_.add(gui_kalman_process_noise_.setup("process noise cov", 0.0004, 0.0, 0.01));
	gui_.add(gui_kalman_measurement_noise_.setup("measurement noise cov", 0.007, 0.0, 0.01));
	gui_.add(gui_kalman_post_error_.setup("post error cov", 0.025, 0.0, 0.1));
	gui_.add(gui_using_velocity_.setup("use velocity", true));

	// Mesh
	//gui_.add(active_mesh_adaptation_rate_.setup("adaptation rate", 2, 0, 10));
	//gui_.add(active_mesh_boundary_weight_.setup("boundary weight", 0, 0, 4));
	//gui_.add(active_mesh_depth_weight_.setup("depth weight", 2, 0, 4));

	// Keys
	askPause_ = false;
	askSaveAssets_ = false;
	askBgExport_ = false;
	askedGeneration_ = false;

	// FPS
	gui_.add(gui_fps_.setup(std::string("fps :")));

	// !GUI/CONTROLS //
}

void ofApp::update() {
	// CONTROLS //

	if (askSaveAssets_) {
		garment_.mesh_ref().save("garment_raw_mesh_export.ply");
		ofSaveImage(ofxKinect_.getColorPixelsRef(), "kinect_color_export.tif");
		ofSaveImage(ofxKinect_.getDepthPixelsRef(), "kinect_depth_export.tif");
		ofSaveImage(of_fg_mask_, "kinect_fg_mask.tif");
		ofLogNotice("ofApp::update") << "Exported the 3D mesh, IR/color image, depth image and foreground mask";
		askSaveAssets_ = false;
	}

	if (askBgExport_) {
		if (learntBg_) {
			ofSaveImage(of_depth_bg_, "background_export.tif");
			ofLogNotice("ofApp::update") << "Exported the learnt background";
		}
		else {
			ofLogNotice("ofApp::update") << "Can't export the background, press b to learn background";
		}
		askBgExport_ = false;
	}

	if (askPause_)
		return;

	// !CONTROLS //


	// HARDWARE UPDATE //

	ofxKinect_.update();

	// !HARDWARE UPDATE //


	// MAIN PROCESS LOOP //
	if (ofxKinect_.isFrameNewDepth()) {

		// Background learning
		if (askBgLearning_) {
			bgLearningCycleCount_ = gui_bg_learning_cycle_;
			learntBg_ = false;
			askBgLearning_ = false;

			ofLogNotice("ofApp::update()") << "Learning background . . .";
			of_depth_bg_ = ofxKinect_.getDepthPixels();
			cv_depth_bg_ = ofxCv::toCv(of_depth_bg_);
			--bgLearningCycleCount_;

			if (bgLearningCycleCount_ == 0) {
				cv_depth_bg_gpu_.upload(cv_depth_bg_);
				learntBg_ = true;
				ofLogNotice("ofApp::update()") << "Background learning complete";
			}
		}
		else if (bgLearningCycleCount_ != 0) {
			for (int i = 0; i < of_depth_bg_.getWidth()*of_depth_bg_.getHeight(); ++i) {
				if (0 == of_depth_bg_[i]) {
					of_depth_bg_[i] = ofxKinect_.getDepthPixelsRef()[i];
				}
			}

			--bgLearningCycleCount_;
			if (bgLearningCycleCount_ == 0) {
				cv_depth_bg_gpu_.upload(cv_depth_bg_);
				learntBg_ = true;
				ofLogNotice("ofApp::update()") << "Background learning complete";
			}
		}
		
		if (!learntBg_) {
			bool importedPixels = ofLoadImage(of_depth_bg_, "background_export.tif");
			if (importedPixels) {
				of_depth_bg_.setNumChannels(1);
				cv_depth_bg_ = ofxCv::toCv(of_depth_bg_);
				cv_depth_bg_gpu_.upload(cv_depth_bg_);
				learntBg_ = true;
				ofLogNotice("ofApp::update()") << "Imported background";
			}
		}

		// Once the background is learnt/imported, start the real work
		if (learntBg_) {
			// CUDA background removal
			cv::cuda::GpuMat diff_gpu;
			cv::cuda::GpuMat &current_depth_gpu = ofxKinect_.getDepthPixelsGpuRef();

			cv::cuda::absdiff(cv_depth_bg_gpu_, current_depth_gpu, diff_gpu);

			cv::cuda::threshold(current_depth_gpu, cv_fg_mask_gpu_, 0, 1, CV_THRESH_BINARY); // Put every non zero value to one
			
			cv::cuda::GpuMat temp;
			cv::cuda::multiply(diff_gpu, cv_fg_mask_gpu_, temp, 0.1); // Then multiply to the difference to get the mask and switch to cm with scaling
			temp.convertTo(cv_fg_mask_gpu_, CV_8UC1);

			cv::Mat kernel = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(11, 11));
			cv::Ptr<cv::cuda::Filter> erode = cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, kernel);
			cv::Ptr<cv::cuda::Filter> dilate = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel);
			erode->apply(cv_fg_mask_gpu_, cv_fg_mask_gpu_);
			dilate->apply(cv_fg_mask_gpu_, cv_fg_mask_gpu_);

			cv::cuda::threshold(cv_fg_mask_gpu_, cv_fg_mask_gpu_, 20, 255, CV_THRESH_BINARY);
			cv_fg_mask_gpu_.download(cv_fg_mask_);
			of_fg_mask_.update();

			// Euclidian cluster segmentation
			float finderRes = euclidian_cluster_segmentation_.getResolution();
			finderRes *= finderRes;

			euclidian_cluster_segmentation_.findBlobs(&of_fg_mask_,
				ofVec3f(0.05, 0.05, 0.1), 1, 0.06, 1.2, (int)(0.001*k_kinect_height_*k_kinect_width_ / finderRes), 1);

			blobFound_ = false;
			if (euclidian_cluster_segmentation_.nBlobs != 0)
				blobFound_ = true;

			// The model is found
			if (blobFound_) {
				garment_augmentation::Simple3dblob *blob = euclidian_cluster_segmentation_.blobs[0];  // Take the first blob, which is the biggest one

				// Update the foreground mask to the blob only
				int pcW = euclidian_cluster_segmentation_.getWidth();
				int pcH = euclidian_cluster_segmentation_.getHeight();
				uchar *p_cv_fg_mask;
				for (int y = 0; y < pcH; y++){
					p_cv_fg_mask = cv_fg_mask_.ptr<uchar>(y);
					for (int x = 0; x < pcW; ++x) {
						if (euclidian_cluster_segmentation_.getCloudPoint(y*pcW + x).flag_ != blob->idx)
							p_cv_fg_mask[x] = 0;
					}
				}

				// Eventual smoothing and cropping on the depth map !!! DESTRUCTIVE OPERATION ON THE DEPTH !!!
				// this is a vertical smoothing, assuming that the folds are verticals, and crop the depth image
				// to the contour of the model
				if (gui_gaussian_size_ >= 3 && gui_gaussian_sigma_ > 0) {
					if (gui_gaussian_size_ % 2 == 0)
						gui_gaussian_size_ = gui_gaussian_size_ + 1;
					ofxKinect_.CropAndSmooth(cv_fg_mask_gpu_, gui_gaussian_size_, gui_gaussian_sigma_);
				}

				// Update the garment object with the blob (generate the mesh, etc.
				garment_.Update(euclidian_cluster_segmentation_, *blob);

				// Detect and update the folds
				DetectFolds();

				// Possibly use the tracking mesh by energy minimization too
				/*if (!active_mesh_.isGenerated() && askedGeneration_) {
					active_mesh_.generateMesh(garment_);
				}
				else if (askedGeneration_){
					active_mesh_.updateMesh(ofxKinect_, garment_, euclidian_cluster_segmentation_.getResolution());
				}*/

				// Update the animations
				garment_.UpdateAnimations();
			}
		}
	}
	// !MAIN PROCESS LOOP //
}

void ofApp::draw() {
	if (ofxKinect_.isFrameNew()) {
		// COMPUTER SCREEN //

		ofBackground(ofColor::grey);
		projectorWindow_.background(ofColor::black);

		// Top Left : color video
		ofxKinect_.draw(0, 0);

		// Top Right : depth map
		ofxKinect_.drawDepth(k_kinect_width_, 0);

		// Bottom Left : foreground mask
		of_fg_mask_.draw(0, k_kinect_height_);

		// Bottom Right : mesh with folds and animations in the world coordinates
		if (blobFound_) {
			if (askPause_) // if the application is in pause, switch to a CAD-like view
				easyCam_.begin();

			ofPushMatrix();
				ofTranslate(k_kinect_width_, k_kinect_height_);
				ofScale(1 / k_to_world_units_, 1 / k_to_world_units_, 1 / k_to_world_units_);
				ofTranslate(0, 0, -garment_.blob().maxZ.z - 1);
				garment_.DrawMesh();
				garment_.DrawFolds();
				garment_.DrawAnimations();
				//active_mesh_.drawWireframe();
			ofPopMatrix();

			if (askPause_)
				easyCam_.end();
		}

		// !COMPUTER SCREEN //

		 
		// PROJECTOR SCREEN //

		if (blobFound_) {
			projectorWindow_.Begin();
				ofMultMatrix(ofMatrix4x4::getTransposedOf(kinectProjectorToolkit_.getTransformMatrix()));
				ofScale(1 / k_to_world_units_, 1 / k_to_world_units_, 1 / k_to_world_units_);
				//garment_.DrawMesh();
				garment_.DrawAnimations();
			projectorWindow_.End();
		}

		// !PROJECTOR SCREEN //
	}

	// GUI //

	gui_fps_.setup(std::string("fps : ") + std::to_string(ofGetFrameRate()));
	gui_.draw();

	// !GUI //
}

void ofApp::exit() {
	ofxKinect_.stop(); // Clean stop of the kinect sensor
}

void ofApp::keyPressed(int key) {
	if (key == ' ') { // Pause the application for close up to the mesh
		askPause_ = !askPause_;
	}
	else if (key == 's') { // Save the various images and mesh
		askSaveAssets_ = true;
	}
	else if (key == 'b') { // Ask to learn the background from the current scene
		askBgLearning_ = true;
	}
	else if (key == 'i') { // Export the learnt background. It will be automatically loaded next time
		askBgExport_ = true;
	}
	else if (key == 'c') { // Launch the fold detection and tracking
		askFoldComputation_ = true;
	}
	else if (key == 'g') { // Generate the tracking mesh if enabled
		askedGeneration_ = true;
	}
}

/* OF ROUTINES	-	-	-	-	-	-	-	-	-	-	-	-	-	*/



/* METHODS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::ParameterizationLSCM(const int textureSize, ofMesh& mesh) {
	mesh.enableTextures();
	ofVec2f* texCoords = new ofVec2f[mesh.getNumVertices()];
	mesh.addTexCoords(texCoords, mesh.getNumVertices());
	delete[] texCoords;

	LSCM lscm(mesh);
	lscm.apply();
	float uRange = std::abs(lscm.umax - lscm.umin);
	float vRange = std::abs(lscm.vmax - lscm.vmin);
	float maxRange = std::max(uRange, vRange);
	ofVec2f outputRange(textureSize*uRange / maxRange, textureSize*vRange / maxRange);

	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		mesh.setTexCoord(i, of_utilities::MapVec2f(mesh.getTexCoord(i), ofVec2f(lscm.umin, lscm.vmin), ofVec2f(lscm.umax, lscm.vmax), ofVec2f(0, 0), outputRange));
	}
}

void ofApp::DetectFolds() {
	const vector<vector<int>> &garment_mesh_2dview = garment_.mesh2d_view();
	ofFastMesh &garment_mesh = garment_.mesh_ref();

	// Convert the fold widths from meters (GUI) to pixels (on the depth map)
	float meters_per_pixel_horizontal = garment_.blob().massCenter.z * ofxKinect_.HORIZONTAL_FOCAL_LENGTH_INV;
	int fold_pixels_width = gui_fold_width_ / meters_per_pixel_horizontal;
	fold_pixels_width /= euclidian_cluster_segmentation_.getResolution();

	// Construct the GPU 3D coordinates map
	cv::cuda::GpuMat world_coordinates_gpu;
	cv::Mat world_coordinates;

	ofRectangle model_2d_roi = garment_.blob().bounding_box_2d;
	int top_left_x = model_2d_roi.getTopLeft().x;
	int top_left_y = model_2d_roi.getTopLeft().y;

	world_coordinates.create(model_2d_roi.height, model_2d_roi.width, CV_32FC3); // 4th channel is indicating whether there is a valid value

	float *p_world_coord_row;
	ofVec3f *vertex;
	for (int y = 0; y < world_coordinates.rows; ++y) {
		p_world_coord_row = world_coordinates.ptr<float>(y);
		for (int x = 0; x < world_coordinates.cols; ++x) {
			if (garment_mesh_2dview[top_left_x + x][top_left_y + y] != -1) {
				vertex = &garment_mesh.getVertex(garment_mesh_2dview[top_left_x + x][top_left_y + y]);
				p_world_coord_row[x * 3] = vertex->x;
				p_world_coord_row[x * 3 + 1] = vertex->y;
				p_world_coord_row[x * 3 + 2] = vertex->z;
			}
			else {
				p_world_coord_row[x * 3 + 2] = std::numeric_limits<float>::infinity();
			}
		}
	}

	world_coordinates_gpu.upload(world_coordinates);

	// CUDA craters detection
	cv::cuda::GpuMat detected_deformations_gpu;
	detected_deformations_gpu.create(world_coordinates_gpu.size(), CV_8UC1);
	garment_augmentation::cuda_optimization::AcceleratedDeformationsDetection(world_coordinates_gpu, gui_crater_deformation_thresh_, fold_pixels_width, detected_deformations_gpu);

	// Get the result back from the graphic card
	cv::Mat output_points;
	detected_deformations_gpu.download(output_points);
	std::vector<ofVec3f> points;
	points.reserve(100);

	uchar *output_points_row_ptr;
	for (int y = 0; y < output_points.rows; ++y) {
		output_points_row_ptr = output_points.ptr<uchar>(y);
		for (int x = 0; x < output_points.cols; ++x) {
			if (output_points_row_ptr[x] != 0) {
				points.push_back(garment_mesh.getVertex(garment_mesh_2dview[top_left_x + x][top_left_y + y]));

				// Color back the mesh
				for (int yy = top_left_y + y - (fold_pixels_width - 1) / 2; yy <= top_left_y + y + (fold_pixels_width - 1) / 2; ++yy) {
					for (int xx = top_left_x + x - (fold_pixels_width - 1) / 2; xx <= top_left_x + x + (fold_pixels_width - 1) / 2; ++xx) {
						if (garment_mesh_2dview[xx][yy] != -1)
							garment_mesh.setColor(garment_mesh_2dview[xx][yy], ofColor::red);
					}
				}
			}
		}
	}

	// Compute folds from craters
	if (askFoldComputation_) {
		// Tune the kalman parameters according the GUI inputs
		ransac_kalman_tracker_.TuneKalmanCovariances(gui_kalman_process_noise_, gui_kalman_measurement_noise_, gui_kalman_post_error_);
		ransac_kalman_tracker_.SetVelocityUse(gui_using_velocity_);

		// Run the tracker and retrieve the updated or new segments with their lifetime
		std::vector<std::pair<garment_augmentation::math::Of3dsegmentOrientation, float> > tracked_segments;
		ransac_kalman_tracker_.Track3Dsegments(points, gui_ransac_distance_thresh_, gui_ransac_points_num_thresh_, tracked_segments);

		// Send the folds' update to the interactive garment object
		garment_.UpdateFolds(tracked_segments);
	}
}

/* METHODS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	*/