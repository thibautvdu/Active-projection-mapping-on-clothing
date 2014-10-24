#ifndef FLEXIBLE_SURFACE_AUGMENTATION_ANIMATION_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_ANIMATION_H_

#include "ofMain.h"

#include "fold.h"

namespace garment_augmentation {
namespace garment {

	class Animation {
		public:
			virtual void Update() = 0;
			virtual void Update(const std::vector<Fold> &folds) = 0;
			virtual void Draw() = 0;

		protected:
			static void RefreshTime() {
				last_frame_time_ = ofGetLastFrameTime();
			}
			inline static double last_frame_time() {
				return last_frame_time_;
			}

		private:
			static double last_frame_time_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_ANIMATION_H_