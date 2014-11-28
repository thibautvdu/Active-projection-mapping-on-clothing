#include "ofApp.h"

#include <stdlib.h>
#include "opencv2/cuda.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaarithm.hpp"

#include "cv_helper.h"
#include "lscm.h"
#include "deformation_tracker.h"
#include "flying_lights.h"
#include "contour_vector_field.h"
//#include "fold_video_texture.h"
#include "gl_shader.h"

#include "cuda_test.h"

/* CONSTANTS	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

// HARDWARE HANDLERS	/	/	/	/	/	/	/	/	/	/	/	/

const int ofApp::screenWidth_ = 1920, ofApp::screenHeight_ = 1080;
const int ofApp::projectorWidth_ = 1024, ofApp::projectorHeight_ = 768;
const int ofApp::kinectWidth_ = 640, ofApp::kinectHeight_ = 480;

// HARDWARE HANDLERS	-	-	-	-	-	-	-	-	-	-	-	-

// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

const float ofApp::k_to_world_units_ = 0.001; // mm to meters

// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-

/* CONSTANTS	/	/	/	/	/	/	/	/	/	/	/	/	/	*/


/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::setup() {
	// HARDWARE INIT	/	/	/	/	/	/	/	/	/	/	/	/

	bool initSensor = ofxKinect_.initSensor();
	if (!initSensor) {
		ofLogFatalError("ofApp::setup") << "Couldn't init the kinect's sensor";
		exit();
	}

	bool initStreams = ofxKinect_.initColorStream(kinectWidth_, kinectHeight_) && ofxKinect_.initDepthStream(kinectWidth_, kinectHeight_, false, true);
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
	kinectProjectorToolkit_.loadCalibration("cal.xml", projectorWidth_, projectorHeight_);
	projectorWindow_ = of_utilities::VirtualWindow(screenWidth_, 0, projectorWidth_, projectorHeight_);

	// HARDWARE INIT	-	-	-	-	-	-	-	-	-	-	-	-


	// OPEN GL	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	ofDisableAlphaBlending();
	ofSetFrameRate(50);

	// OPEN GL	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// KINECT SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/	/

	// Background learning
	bgLearningCycleCount_ = 0;
	askBgLearning_ = false;
	learntBg_ = false;

	// Background segmentation
	bgMask_.allocate(kinectWidth_,kinectHeight_,OF_IMAGE_GRAYSCALE);
	cvBgMask_ = ofxCv::toCv(bgMask_);

	// Blob detection
	blobFound_ = false;

	// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

	// Blob finder and tracker
	blobFinder_.init(&ofxKinect_, kinectWidth_, kinectHeight_); // standarized coordinate system: z in the direction of gravity
	//blobFinder_.setResolution(garment_augmentation::blob_detection::BF_HIGH_RES);
	blobFinder_.setScale(ofVec3f(k_to_world_units_)); // mm to meters

	// Fold processing
	threaded_deformation_detector_.SetTargetGarment(&garment_);
	ransac_kalman_tracker_.set_segments_life_time(0.5); // life time of the folds
	askFoldComputation_ = false;
	numFolds_ = 1;

	// Physic animations
	std::unique_ptr<garment_augmentation::garment::Animation> lightsEffect(new garment_augmentation::garment::FlyingLights());
	//garment_.AddAnimation(std::move(lightsEffect));
	std::unique_ptr<garment_augmentation::garment::Animation> contourEffect(new garment_augmentation::garment::ContourVectorField());
	//garment_.AddAnimation(std::move(contourEffect));
	//std::unique_ptr<garment_augmentation::garment::Animation> videoEffect(new garment_augmentation::garment::FoldVideoTexture("videos/halluc_edit.mp4"));
	//garment_.AddAnimation(std::move(videoEffect));

	// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

	chessboardImage_.loadImage("chessboard.png");

	// CUDA TEST //
	first_test.loadImage("first_test.jpg");
	cv::Mat first_test_cv = ofxCv::toCv(first_test);
	ofLog() << cv_helper::GetImageType(first_test_cv);
	cv::cuda::GpuMat first_test_cv_gpu;
	first_test_cv_gpu.upload(first_test_cv);
	garment_augmentation::garment::cuda_test(first_test_cv_gpu);
	first_test_cv_gpu.download(first_test_cv);

	// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


	// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	gui_.setup();
	gui_.add(new ofxLabel(std::string("BACKGROUND LEARNING")));
	gui_.add(bgLearningCycleGui_.setup("nb iterations", 10, 1, 50));
	gui_.add(new ofxLabel(std::string("GAUSSIAN SMOOTHING")));
	gui_.add(gaussian_size_.setup("size", 5, 3, 9));
	gui_.add(gaussian_sigma_.setup("sigma", 2.5, 0, 3));
	gui_.add(new ofxLabel(std::string("DEFORMATION DETECTION")));
	gui_.add(deformation_detector_num_threads_.setup("num of threads", 6, 1, 12));
	gui_.add(fold_deformation_thresh_.setup("deformation thresh", 0.015, 0.00, 0.03));
	gui_.add(fold_deformation_thresh_2_.setup("deformation thresh 2", 50, 0.00, 100));
	gui_.add(new ofxLabel(std::string("FOLD TRACKING")));
	gui_.add(fold_distance_thresh_.setup("distance thresh", 0.05, 0.01, 0.08));
	gui_.add(fold_points_num_thresh_.setup("nb points thresh", 7, 5, 20));
	gui_.add(fold_width_.setup("fold's width",0.045,0.0,0.1));
	gui_.add(kalman_process_noise_.setup("process noise cov", 0.0004, 0.0, 0.01));
	gui_.add(kalman_measurement_noise_.setup("measurement noise cov", 0.007, 0.0, 0.01));
	gui_.add(kalman_post_error_.setup("post error cov", 0.025, 0.0, 0.1));
	gui_.add(using_velocity_.setup("use velocity", true));

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
	gui_.add(fpsGui_.setup(std::string("fps :")));

	// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
}

void ofApp::update() {
	// EVENTS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	if (askSaveAssets_) {
		garment_.mesh_ref().save("mesh_export.ply");
		ofSaveImage(ofxKinect_.getColorPixelsRef(), "color_ir_export.tif");
		ofSaveImage(ofxKinect_.getDepthPixelsRef(), "depth_export.tif");
		ofSaveImage(bgMask_, "bg_mask.tif");
		ofLogNotice("ofApp::update") << "Exported the 3D mesh, IR/color and depth image";
		askSaveAssets_ = false;
	}
	if (askBgExport_) {
		if (learntBg_) {
			ofSaveImage(depthBg_, "background_export.tif");
			ofLogNotice("ofApp::update") << "Exported the learnt background";
		}
		else {
			ofLogNotice("ofApp::update") << "Can't export the background, press b to learn background";
		}
		askBgExport_ = false;
	}
	if (askPause_)
		return;

	//active_mesh_.setAdaptationRate(active_mesh_adaptation_rate_);
	//active_mesh_.setBoundaryWeight(active_mesh_boundary_weight_);
	//active_mesh_.setDepthWeight(active_mesh_depth_weight_);

	// EVENTS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// HARDWARE	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	ofxKinect_.update();

	// HARDWARE	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// PROCESS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/
	if (ofxKinect_.isFrameNewDepth()) {
		// Background learning
		
		if (askBgLearning_) {
			bgLearningCycleCount_ = bgLearningCycleGui_;
			learntBg_ = false;
			askBgLearning_ = false;

			ofLogNotice("ofApp::update()") << "Learning background . . .";
			depthBg_ = ofxKinect_.getDepthPixels();
			cvDepthBg_ = ofxCv::toCv(depthBg_);
			--bgLearningCycleCount_;

			if (bgLearningCycleCount_ == 0) {
				cv_depth_bg_gpu_.upload(cvDepthBg_);
				learntBg_ = true;
				ofLogNotice("ofApp::update()") << "Background learning complete";
			}
		}
		else if (bgLearningCycleCount_ != 0) {
			for (int i = 0; i < depthBg_.getWidth()*depthBg_.getHeight(); ++i) {
				if (0 == depthBg_[i]) {
					depthBg_[i] = ofxKinect_.getDepthPixelsRef()[i];
				}
			}

			--bgLearningCycleCount_;
			if (bgLearningCycleCount_ == 0) {
				cv_depth_bg_gpu_.upload(cvDepthBg_);
				learntBg_ = true;
				ofLogNotice("ofApp::update()") << "Background learning complete";
			}
		}
		
		if (!learntBg_) {
			bool importedPixels = ofLoadImage(depthBg_, "background_export.tif");
			if (importedPixels) {
				depthBg_.setNumChannels(1);
				cvDepthBg_ = ofxCv::toCv(depthBg_);
				cv_depth_bg_gpu_.upload(cvDepthBg_);
				learntBg_ = true;
				ofLogNotice("ofApp::update()") << "Imported background";
			}
		}

		// Point cloud processing
		if (learntBg_) {
			// Background removal
			cv::cuda::GpuMat diff_gpu;
			cv::cuda::GpuMat &current_depth_gpu = ofxKinect_.getDepthPixelsGpuRef();

			cv::cuda::absdiff(cv_depth_bg_gpu_, current_depth_gpu, diff_gpu);

			// Put every non zero value to one
			cv::cuda::threshold(current_depth_gpu, cv_bg_mask_gpu_, 0, 1, CV_THRESH_BINARY);
			// Multiply it to the diff to get the mask and switch to cm
			cv::cuda::GpuMat temp;
			cv::cuda::multiply(diff_gpu, cv_bg_mask_gpu_, temp,0.1);
			temp.convertTo(cv_bg_mask_gpu_, CV_8UC1);

			/*
			UCHAR *p_maskRow;
			USHORT *p_diffRow;
			USHORT *p_depth;
			for (int row = 0; row < kinectHeight_; ++row) {
				p_maskRow = cvBgMask_.ptr<UCHAR>(row);
				p_diffRow = diff.ptr<USHORT>(row);
				p_depth = current_depth.ptr<USHORT>(row);
				for (int col = 0; col < kinectWidth_; ++col) {
					if (*(p_depth + col) != 0)
						*(p_maskRow + col) = *(p_diffRow + col) / 10; // to cm
					else
						*(p_maskRow + col) = 0;
				}
			}*/

			cv::Mat kernel = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(11, 11));
			cv::Ptr<cv::cuda::Filter> erode = cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, kernel);
			cv::Ptr<cv::cuda::Filter> dilate = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel);
			erode->apply(cv_bg_mask_gpu_, cv_bg_mask_gpu_);
			dilate->apply(cv_bg_mask_gpu_, cv_bg_mask_gpu_);

			cv::cuda::threshold(cv_bg_mask_gpu_, cv_bg_mask_gpu_, 20, 255, CV_THRESH_BINARY);
			cv_bg_mask_gpu_.download(cvBgMask_);
			bgMask_.update();

			// Eventual smoothing
			if (gaussian_size_ >= 3 && gaussian_sigma_ > 0) {
				if (gaussian_size_ % 2 == 0)
					gaussian_size_ = gaussian_size_ + 1;
				ofxKinect_.SelectiveSmoothing(cv_bg_mask_gpu_, gaussian_size_, gaussian_sigma_);
			}

			// Blob detection
			float finderRes = blobFinder_.getResolution();
			finderRes *= finderRes;

			blobFinder_.findBlobs(&bgMask_,
				ofVec3f(0.05, 0.05, 0.1), 1, 0.06, 1.2, (int)(0.001*kinectHeight_*kinectWidth_ / finderRes), 1);

			blobFound_ = false;
			if (blobFinder_.nBlobs != 0)
				blobFound_ = true;

			if (blobFound_) {
				garment_.Update(blobFinder_, *blobFinder_.blobs[0]);
				detectFolds();
				garment_.UpdateAnimations();

				/*if (!active_mesh_.isGenerated() && askedGeneration_) {
					active_mesh_.generateMesh(garment_);
				}
				else if (askedGeneration_){
					active_mesh_.updateMesh(ofxKinect_, garment_, blobFinder_.getResolution());
				}*/
			}
		}
	}
	// PROCESS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
}

void ofApp::draw() {
	if (ofxKinect_.isFrameNew()) {
		// COMPUTER SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/

		ofBackground(ofColor::grey);
		projectorWindow_.background(ofColor::black);

		// Top Left
		//ofxKinect_.draw(0, 0);
		first_test.update();
		first_test.draw(0, 0);

		// Top Right
		ofxKinect_.drawDepth(kinectWidth_, 0);

		// Bottom Left
		bgMask_.draw(0, kinectHeight_);

		// Bottom Right
		if (blobFound_) {
			if (askPause_)
				easyCam_.begin();

			ofPushMatrix();
				ofTranslate(kinectWidth_, kinectHeight_);
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

		// COMPUTER SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-

		 
		// PROJECTOR SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/

		if (blobFound_) {
			projectorWindow_.Begin();
				ofMultMatrix(ofMatrix4x4::getTransposedOf(kinectProjectorToolkit_.getTransformMatrix()));
				ofScale(1 / k_to_world_units_, 1 / k_to_world_units_, 1 / k_to_world_units_);
				//garment_.DrawMesh();
				garment_.DrawAnimations();
			projectorWindow_.End();
		}

		// PROJECTOR SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-
	}

	// GUI
	fpsGui_.setup(std::string("fps : ") + std::to_string(ofGetFrameRate()));
	gui_.draw();
}

/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::exit() {
	ofxKinect_.stop();
}

void ofApp::mousePressed(int x, int y, int button) {

}

void ofApp::keyPressed(int key) {
	if (key == ' ') {
		askPause_ = !askPause_;
	}
	else if (key == 's') {
		askSaveAssets_ = true;
	}
	else if (key == 'b') {
		askBgLearning_ = true;
	}
	else if (key == 'i') {
		askBgExport_ = true;
	}
	else if (key == 'c') {
		askFoldComputation_ = true;
	}
	else if (key == 'g') {
		askedGeneration_ = true;
	}
}

/* OF ROUTINES	-	-	-	-	-	-	-	-	-	-	-	-	-	*/


/* METHODS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::meshParameterizationLSCM(const int textureSize, ofMesh& mesh) {
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

void ofApp::detectFolds() {
	// Convert the seeked fold width from meters to pixels
	float meters_per_pixel_horizontal = garment_.blob().massCenter.z * ofxKinect_.HORIZONTAL_FOCAL_LENGTH_INV;
	int fold_pixels_width = fold_width_ / meters_per_pixel_horizontal;
	fold_pixels_width /= blobFinder_.getResolution();

	threaded_deformation_detector_.SetNumThreads(deformation_detector_num_threads_);
	std::vector<ofVec3f> points = threaded_deformation_detector_.DetectDeformations(fold_pixels_width, 2, fold_deformation_thresh_);

	// Compute folds from deformaed areas
	if (askFoldComputation_) {
		// Tune the kalman parameters
		ransac_kalman_tracker_.TuneKalmanCovariances(kalman_process_noise_, kalman_measurement_noise_,kalman_post_error_);
		ransac_kalman_tracker_.SetVelocityUse(using_velocity_);

		// Run the tracker and retrieve the updated or new segments with their lifetime
		std::vector<std::pair<garment_augmentation::math::Of3dsegmentOrientation, float> > tracked_segments;
		ransac_kalman_tracker_.Track3Dsegments(points, fold_distance_thresh_, fold_points_num_thresh_, tracked_segments); // threshold : 5cm, minimum points : 10

		garment_.UpdateFolds(tracked_segments);
	}
}

/* METHODS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	*/