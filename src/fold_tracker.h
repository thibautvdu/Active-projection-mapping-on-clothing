#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_

#include "interactive_garment.h"

namespace garment_augmentation {
namespace garment {

	class FoldTracker {
		public:
			FoldTracker(InteractiveGarment *garment, const ofRectangle roi) : p_garment_(garment), roi_(roi) {
				need_computation_ = true;
			}


			inline void Move(const int x, const int y) {
				roi_.translate(x, y);
				need_computation_ = true;
			}

			inline void MoveTo(const ofRectangle roi) {
				roi_ = roi;
				need_computation_ = true;
			}

			inline float GetFoldness() {
				if (need_computation_)
					ComputeDeltaDepth();

				return delta_depth_;
			}

			std::vector<ofVec3f> GetPoints();
			void ColorFill(const ofColor color);

			bool IsInsideMesh();

		private:
			void ComputeDeltaDepth();

			InteractiveGarment *const p_garment_;
			ofRectangle roi_;
			bool need_computation_;
			float delta_depth_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_