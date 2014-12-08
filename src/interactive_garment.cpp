#include "interactive_garment.h"

namespace garment_augmentation {
namespace garment {

	void InteractiveGarment::Update(const blob_detection::kinect3dBlobDetector &detector, const Simple3dblob blob) {
		blob_ = blob;

		int pcW = detector.getWidth();
		int pcH = detector.getHeight();

		if (mesh_.getCapacity() != pcW*pcH) {
			mesh_.reserveCapacity(pcW*pcH);
			contour_.reserve(pcW*pcH);
			mesh2d_view_.resize(pcW, vector<int>(pcH));
		}

		mesh_.resize(blob_.nbPoints);
		contour_.clear();

		int vIdx = 0;
		for (int y = 0; y < pcH; y++){
			for (int x = 0; x < pcW; ++x) {
				if (detector.getCloudPoint(y*pcW + x).flag_ == blob_.idx) {
					mesh_.setVertex(vIdx, detector.getCloudPoint(y*pcW + x).pos_ - blob.massCenter);
					if (detector.getCloudPoint(y*pcW + x).boundary_) {
						contour_.push_back(detector.getCloudPoint(y*pcW + x).pos_);
						mesh_.setColor(vIdx, ofColor::blue);
					}
					else {
						mesh_.setColor(vIdx, ofColor::white);
					}
					mesh2d_view_[x][y] = vIdx;
					vIdx++;
				}
				else
					mesh2d_view_[x][y] = -1;
			}
		}
	}

	template <typename P_TYPE_A, typename P_TYPE_B>
	bool InteractiveGarment::ComparePairSecond(const std::pair<P_TYPE_A, P_TYPE_B> a, const std::pair<P_TYPE_A, P_TYPE_B> b) {
		return (a.second < b.second);
	}

	void InteractiveGarment::UpdateFolds(std::vector<Fold> &new_folds) {
		float threshold = 0.3;
		float orientation_weight = 0.7, proximity_weight = 0.3;

		// Compute the distances
		ofVec3f new_middle, new_director;
		float new_squared_length;
		for (int i = 0; i < new_folds.size(); ++i) {
			Fold &new_fold = new_folds[i];
			new_middle = new_fold.GetMiddle();
			new_director = new_fold.GetDirector();
			new_squared_length = new_fold.GetSquaredLength();
			new_fold.closests_.resize(folds_.size());
			for (int j = 0; j < folds_.size(); ++j) {
				float distance_score = (1 - new_director.dot(folds_[j].GetDirector())) * orientation_weight + ((new_middle - folds_[j].GetMiddle()).lengthSquared() / folds_[j].GetSquaredLength()) * proximity_weight;
				new_fold.closests_[j] = std::pair<int, float>(j, distance_score);
			}

			std::sort(new_fold.closests_.begin(), new_fold.closests_.end(), ComparePairSecond<int,float>);
		}

		// Compute all the relevant configurations
		folds_tracking_permutations_.clear();
		std::pair<std::vector<int>, float> compute_config(std::vector<int>(new_folds.size()), 0);
		FoldsPermutation(new_folds, compute_config, threshold, 0);

		// Find the optimal configuration
		vector<std::pair<std::vector<int>, float>>::iterator best_config_ite = std::min_element(folds_tracking_permutations_.begin(),
																			folds_tracking_permutations_.end(), 
																			ComparePairSecond<std::vector<int>, float>);
		if (folds_tracking_permutations_.end() != best_config_ite) {
			std::vector<int> &best_config = best_config_ite->first;
			// Update the folds
			for (int i = 0; i < best_config.size(); ++i) {
				if (best_config[i] == -1) { // new fold
					folds_.push_back(new_folds[i]);
				}
				else {
					folds_[best_config[i]].UpdatePosition(new_folds[i]);
				}
			}
			for (int i = 0; i < folds_.size(); ++i) {
				//folds_[i].Update();
				if (folds_[i].flag() == Fold::TO_SUPPRESS) {
					folds_.erase(folds_.begin() + i);
					--i;
				}
			}
		}
	}

	void InteractiveGarment::UpdateFolds(const std::vector<std::pair<math::Of3dsegmentOrientation, float>> &folds_update) {
		// Update existing folds
		for (int i = folds_.size() - 1; i >= 0; --i) {
			if (i >= folds_update.size() || 0 >= folds_update[i].second) {
				folds_[i].set_flag(Fold::foldState::TO_SUPPRESS);
			}
			else {
				folds_[i].UpdatePosition(folds_update[i].first);
			}
		}

		// Create new folds
		int first_new_fold_idx = folds_.size();
		for (int i = first_new_fold_idx; i < folds_update.size(); ++i) {
			folds_.push_back(Fold(folds_update[i].first));
		}

		// Suppress the flaged folds
		for (int i = folds_.size() - 1; i >= 0; --i) {
			if (folds_[i].flag() == Fold::foldState::TO_SUPPRESS) {
				folds_.erase(folds_.begin() + i);
			}
		}
	}

	void InteractiveGarment::FoldsPermutation(const std::vector<Fold> &new_folds, std::pair<std::vector<int>,float> &current_configuration,  const float threshold, int new_fold_idx) {
		if (new_fold_idx == new_folds.size()) { // STOP RECCURSION
			folds_tracking_permutations_.push_back(current_configuration);
		}
		else {
			Fold fold = new_folds[new_fold_idx];
			bool out_of_threshold = false;
			for (int i = 0; i < fold.closests_.size() && !out_of_threshold; ++i) {
				if (fold.closests_[i].second < threshold) {
					// Check we didn't assign this closest fold already in this configuration
					bool used = false;
					for (int j = 0; j < new_fold_idx && !used; ++j) {
						if (current_configuration.first[j] == fold.closests_[i].first)
							used = true;
					}

					if (!used) {
						current_configuration.first[new_fold_idx] = fold.closests_[i].first;
						current_configuration.second += fold.closests_[i].second;
						FoldsPermutation(new_folds, current_configuration, threshold, new_fold_idx+1);	// RECCURSION
					}
				}
				else { // There is no more closest element that are under the desired threshold
					out_of_threshold = true;
				}
			}

			// New fold
			if (new_folds.size() > folds_.size()) {
				// Check we didn't already create all the new folds
				int nb_new_folds = 0;
				for (int i = 0; i < new_fold_idx; ++i) {
					if (current_configuration.first[i] == -1) {
						++nb_new_folds;
					}
				}
				if (nb_new_folds < (new_folds.size() - folds_.size())) {
					current_configuration.first[new_fold_idx] = -1;
					FoldsPermutation(new_folds, current_configuration, threshold, new_fold_idx + 1); // RECCURSION
				}
			}
		}
	}


} // namespace garment
} // namespace garment_augmentation