#ifndef FLEXIBLE_SURFACE_AUGMENTATION_THREADED_DEFORMATION_DETECTION_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_THREADED_DEFORMATION_DETECTION_H_

#include "interactive_garment.h"

#include "ofMain.h"

#include "deformation_tracker.h"

namespace garment_augmentation {
namespace garment {

	class ThreadedDeformationDetector {

		public:
			ThreadedDeformationDetector() : garment_(NULL) {
				num_threads_ = 4;
			}

			~ThreadedDeformationDetector() {
				for (int i = 0; i < threads_.size(); ++i)
					threads_[i]->stopThread();
			}

			void SetTargetGarment(InteractiveGarment *garment) {
				garment_ = garment;
			}

			void SetNumThreads(const int num_threads) {
				num_threads_ = num_threads;
			}

			std::vector<ofVec3f> DetectDeformations(const int min_fold_width, const int growing_range,const float deformation_thresh);

		private:
			class DetectorThread : public ofThread {
				public:
					DetectorThread(InteractiveGarment &garment, std::vector<ofVec3f> &out_detected) :
						detected_points_(out_detected), garment_(garment),
						tracker_(garment_augmentation::garment::DeformationTracker(&garment_, ofRectangle(0, 0, 0, 0))) {
						map_2d_width_ = garment_.mesh2d_view().size();

						if (map_2d_width_ != 0) {
							map_2d_height_ = garment_.mesh2d_view()[0].size();
						}
						else{
							ofLogFatalError("ThreadedDeformationDetector") << "The 2d map of the garment mesh is empty";
						}

						setting_done_ = false;
					}

					void Setting(const ofPoint top_left, const ofPoint bottom_right, const int min_fold_width, const int growing_range,
						const float deformation_thresh, std::shared_ptr<ofMutex> mutex_points, std::shared_ptr<ofMutex> mutex_garment_mesh) {
						top_left_corner_ = top_left;
						bottom_right_corner_ = bottom_right;
						min_fold_width_ = min_fold_width;
						growing_range_ = growing_range;
						deformation_thresh_ = deformation_thresh;
						detected_points_lock_ = mutex_points;
						garment_mesh_lock_ = mutex_garment_mesh;
						setting_done_ = true;
					}

					void threadedFunction();

				private:
					std::vector<ofVec3f> &detected_points_;
					std::shared_ptr<ofMutex> detected_points_lock_;
					InteractiveGarment &garment_;
					std::shared_ptr<ofMutex> garment_mesh_lock_;
					int map_2d_width_, map_2d_height_;
					ofPoint top_left_corner_, bottom_right_corner_;
					int min_fold_width_;
					int growing_range_;
					float deformation_thresh_;
					garment_augmentation::garment::DeformationTracker tracker_;

					bool setting_done_;
			};

			InteractiveGarment *garment_;
			std::vector<ofVec3f> detected_points_;
			std::vector<std::unique_ptr<DetectorThread>> threads_;
			int num_threads_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_THREADED_DEFORMATION_DETECTION_H_