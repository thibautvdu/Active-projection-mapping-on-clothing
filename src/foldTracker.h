#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_

#include "interactiveGarment.h"

namespace garmentAugmentation {
namespace garment {

	class foldTracker {
		public:
			foldTracker(interactiveGarment *garment, const ofRectangle roi) : pGarment_(garment), roi_(roi) {
				area_ = unfoldedArea_ = -1;
				if (kGaussianValues_.size() == 0)
					computeGaussianDist(1);
			}

			inline void move(const int x, const int y) {
				roi_.translate(x, y);
				area_ = unfoldedArea_ = -1;
			}

			inline void moveTo(const ofRectangle roi) {
				roi_ = roi;
				area_ = unfoldedArea_ = -1;
			}

			inline float getArea() {
				if (area_ == -1)
					computeAreas();

				return area_;
			}

			inline float getUnfoldedArea() {
				if (unfoldedArea_ == -1)
					computeAreas();

				return unfoldedArea_;
			}

			inline float getDeformationPercent() {
				return (getArea() - getUnfoldedArea()) * 100 / getUnfoldedArea();
			}

			int getMeshIndex(const int x, const int y) const;

			std::vector<ofVec3f> getPoints();
			void setColor(const ofColor color);

			bool insideMesh();

		private:
			static std::vector<float> kGaussianValues_;

			static void computeGaussianDist(const float sigma);
			void computeAreas();

			interactiveGarment *const pGarment_;
			ofRectangle roi_;
			float area_;
			float unfoldedArea_;
	};

} // namespace garment
} // namespace garmentAugmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_