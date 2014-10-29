#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_

#include "ofMain.h"

namespace garment_augmentation {
namespace garment {

	class Fold {
		public:
			enum foldState { IDLE = 0, NEW = 1, UPDATED = 2, TO_SUPPRESS = 3};

			inline Fold() {
				flag_ = NEW;
				life_time_ = k_default_life_time_; // 1 second
				random_col_ = ofColor(ofRandomf() * 255, ofRandomf() * 255, ofRandomf() * 255);
			}

			inline Fold(const ofVec3f a, const ofVec3f b) : a_pt_(a), b_pt_(b) {
				flag_ = NEW;
				life_time_ = k_default_life_time_; // 1 second
				random_col_ = ofColor(ofRandomf() * 255, ofRandomf() * 255, ofRandomf() * 255);
			}

			inline void Update() {
				if (IDLE == flag_) {
					life_time_ -= static_cast<int>(ofGetLastFrameTime() * 1000);
				}
				else if (UPDATED == flag_) {
					life_time_ = k_default_life_time_;
				}

				if (life_time_ <= 0)
					flag_ = TO_SUPPRESS;
				else
					flag_ = IDLE;
			}

			inline void UpdatePosition(const Fold &new_measure) {
				int elapsed_time = static_cast<int>(ofGetLastFrameTime() * 1000);
				if (elapsed_time > k_update_time_)
					elapsed_time = k_update_time_;

				a_pt_ = a_pt_ * (k_update_time_ - elapsed_time) / k_update_time_ + elapsed_time * new_measure.a_pt_ / k_update_time_;
				b_pt_ = b_pt_ * (k_update_time_ - elapsed_time) / k_update_time_ + elapsed_time * new_measure.b_pt_ / k_update_time_;
				flag_ = UPDATED;
			}

			inline void Draw() {
				ofSetColor(random_col_);
				ofLine(a_pt_, b_pt_);
				ofSetColor(255);
			}

			inline ofVec3f GetMiddle() const {
				return (a_pt_ + b_pt_) / 2;
			}

			inline ofVec3f GetDirector() const {
				return (b_pt_ - a_pt_).getNormalized();
			}

			inline float GetSquaredLength() const {
				return (b_pt_ - a_pt_).lengthSquared();
			}

			inline ofVec3f a_pt() const { 
				return a_pt_; 
			}

			inline ofVec3f b_pt() const { 
				return b_pt_; 
			}

			inline enum foldState flag() const {
				return flag_;
			}

			std::vector<std::pair<int, float>> closests_;

		private:
			static const int k_default_life_time_ = 500; // 1/2 second, if no position update are received in this time frame the fold is destroyed
			static const int k_update_time_ = 200; // 200ms, time frame on which the updates are summed and averaged

			ofVec3f a_pt_;
			ofVec3f b_pt_;
			enum foldState flag_;
			int life_time_;
			ofColor random_col_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_