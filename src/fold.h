#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_

#include "ofMain.h"

namespace garment_augmentation {
namespace garment {

	class Fold {
		public:
			enum foldState { IDLE = 0, NEW = 1 };

			inline Fold() {
				flag_ = NEW;
			}

			inline Fold(const ofVec3f a, const ofVec3f b) : a_pt_(a), b_pt_(b) {
				Fold();
			}

			inline void Update() {
				flag_ = IDLE;
			}

			inline void Draw() {
				ofLine(a_pt_, b_pt_);
			}

			inline ofVec3f a_pt() const { 
				return a_pt_; 
			}

			inline ofVec3f b_pt() const { 
				return b_pt_; 
			}

		private:

			ofVec3f a_pt_;
			ofVec3f b_pt_;
			enum foldState flag_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_