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
				life_time_ = 10;
				random_col_ = ofColor(ofRandomf() * 255, ofRandomf() * 255, ofRandomf() * 255);
			}

			inline Fold(const ofVec3f a, const ofVec3f b) : a_pt_(a), b_pt_(b) {
				flag_ = NEW;
				life_time_ = 10;
				random_col_ = ofColor(ofRandomf() * 255, ofRandomf() * 255, ofRandomf() * 255);
			}

			inline void Update() {
				if (IDLE == flag_) {
					life_time_--;
				}
				else if (UPDATED == flag_) {
					life_time_ = 10;
				}

				if (life_time_ <= 0)
					flag_ = TO_SUPPRESS;
				else
					flag_ = IDLE;
			}

			inline void Update(const Fold &new_measure) {
				a_pt_ = (a_pt_ * 2 + new_measure.a_pt_) / 3;
				b_pt_ = (b_pt_ * 2 + new_measure.b_pt_) / 3;
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

			ofVec3f a_pt_;
			ofVec3f b_pt_;
			enum foldState flag_;
			int life_time_;
			ofColor random_col_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_