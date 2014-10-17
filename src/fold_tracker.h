#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_

#include "interactive_garment.h"

namespace garment_augmentation {
namespace garment {

	class FoldTracker {
		public:
			FoldTracker(InteractiveGarment *garment, const ofRectangle roi) : p_garment_(garment), roi_(roi) {
				area_ = unfolded_area_ = -1;
				if (gaussian_values_.size() == 0)
					ComputeGaussianDistribution(1);
			}

			inline void Move(const int x, const int y) {
				roi_.translate(x, y);
				area_ = unfolded_area_ = -1;
			}

			inline void MoveTo(const ofRectangle roi) {
				roi_ = roi;
				area_ = unfolded_area_ = -1;
			}

			inline float area() {
				if (area_ == -1)
					ComputeAreas();

				return area_;
			}

			inline float unfolded_area() {
				if (unfolded_area_ == -1)
					ComputeAreas();

				return unfolded_area_;
			}

			inline float GetDeformationPercent() {
				return (area() - unfolded_area()) * 100 / unfolded_area();
			}

			std::vector<ofVec3f> GetPoints();
			void ColorFill(const ofColor color);

			bool IsInsideMesh();

		private:
			static std::vector<float> gaussian_values_;

			static void ComputeGaussianDistribution(const float sigma);
			void ComputeAreas();

			InteractiveGarment *const p_garment_;
			ofRectangle roi_;
			float area_;
			float unfolded_area_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_