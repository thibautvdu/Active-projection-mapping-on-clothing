#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_

#include "ofMain.h"

namespace garmentAugmentation {
namespace garment {

	enum foldState {IDLE = 0, NEW = 1};

	class fold {
		public:
			inline fold() {
				flag_ = NEW;
			}

			inline fold(ofVec3f a, ofVec3f b) : aPt_(a), bPt_(b) {
				fold();
			}

			inline void update() {
				flag_ = IDLE;
			}

			inline void draw() {
				ofLine(aPt_, bPt_);
			}

			inline ofVec3f getAPt() const { return aPt_; }
			inline ofVec3f getBPt() const { return bPt_; }

		private:
			ofVec3f aPt_;
			ofVec3f bPt_;
			enum foldState flag_;
	};

} // namespace garment
} // namespace garmentAugmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_H_