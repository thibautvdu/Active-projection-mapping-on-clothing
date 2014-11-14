#include "threaded_deformation_detector.h"

namespace garment_augmentation {
namespace garment {

	std::vector<ofVec3f> ThreadedDeformationDetector::DetectDeformations(const int min_fold_width,
	const int growing_range, const float deformation_thresh) {
		ofRectangle model_2d_roi = garment_->blob().bounding_box_2d;

		detected_points_.clear();

		// Create necessary new threads
		while (threads_.size() < num_threads_) {
			threads_.push_back(std::unique_ptr<DetectorThread>(new DetectorThread(*garment_, detected_points_)));
		}

		if (threads_.size() > num_threads_) {
			threads_.resize(num_threads_);
		}

		// Set all the threads and start them
		std::shared_ptr<ofMutex> detected_points_lock( new ofMutex());
		std::shared_ptr<ofMutex> garment_mesh_lock(new ofMutex());
		int idx_thread = 0;
		int y_increment = std::ceilf(model_2d_roi.height / num_threads_);
		for (int y = model_2d_roi.getTopLeft().y; y < model_2d_roi.getBottomRight().y; y += y_increment) {
			threads_[idx_thread]->Setting(ofPoint(model_2d_roi.getTopLeft().x, y), ofPoint(model_2d_roi.getBottomRight().x, y + y_increment - 1),
											min_fold_width, growing_range, deformation_thresh, detected_points_lock, garment_mesh_lock);
			threads_[idx_thread]->startThread();
			++idx_thread;
		}

		for (int i = 0; i < threads_.size(); ++i)
			threads_[i]->waitForThread(true);
		
		return detected_points_;
	}

	void ThreadedDeformationDetector::DetectorThread::threadedFunction() {

		if (!setting_done_) {
			ofLogFatalError("ThreadedDeformationDetector::DetectorThread::threadedFunction") << "Attempt to run threaded function without settings";
			return;
		}

		// Detect deformed areas
		std::vector<std::pair<int, int>> points_coords;

		if (min_fold_width_ % 2 == 0)
			min_fold_width_ = min_fold_width_ + 1;

		// Correct the roi to avoid going out of the 2d map
		int max_half_fold_width = (min_fold_width_ + growing_range_ - 1) / 2;
		top_left_corner_.x = std::max(max_half_fold_width, static_cast<int>(top_left_corner_.x));
		top_left_corner_.y = std::max(max_half_fold_width, static_cast<int>(top_left_corner_.y));
		bottom_right_corner_.x = std::min(map_2d_width_ - min_fold_width_ - 1, static_cast<int>(bottom_right_corner_.x));
		bottom_right_corner_.y = std::min(map_2d_height_ - min_fold_width_ - 1, static_cast<int>(bottom_right_corner_.y));

		for (int y = top_left_corner_.y; y <= bottom_right_corner_.y; ++y) {
			int previous_center_x = -1;
			int previous_patch_size = -1;
			int previous_center_idx = -1;
			for (int x = top_left_corner_.x; x <= bottom_right_corner_.x; ++x) {
				tracker_.MoveCenterTo(x, y);

				// Test the location for various patch size
				bool got_positive_result = false;
				for (int patch_size = min_fold_width_ + growing_range_; patch_size >= min_fold_width_ && !got_positive_result; patch_size -= 2) {
					tracker_.Resize(min_fold_width_, min_fold_width_);

					if (tracker_.IsInsideMesh()) {
						float foldness = tracker_.GetFoldness();
						if (foldness  > deformation_thresh_) {
							// Test if the center is not contained in a previous valid patch horizontaly aligned
							if (previous_center_x != -1 && x - previous_center_x < previous_patch_size) {
								// average a new center, incorrect but there is not often more than two patches in the merging
								detected_points_lock_->lock();
								detected_points_[previous_center_idx] += tracker_.GetCenter();
								detected_points_[previous_center_idx] /= 2;
								detected_points_lock_->unlock();
							}
							else {
								detected_points_lock_->lock();
								previous_center_idx = detected_points_.size();
								detected_points_.push_back(tracker_.GetCenter());
								detected_points_lock_->unlock();
							}

							previous_patch_size = patch_size;
							previous_center_x = x;
							garment_mesh_lock_->lock();
							tracker_.ColorFill(ofColor::red);
							garment_mesh_lock_->unlock();
							got_positive_result = true;
						}
					}
				}
			}
		}
	}

}
}