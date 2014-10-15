#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_

#include "interactiveGarment.h"

namespace garmentAugmentation {
namespace garment {

	class foldTracker {
		public:
			foldTracker(interactiveGarment *garment, ofRectangle roi) : pGarment_(garment), m_roi(roi) {
				m_area = m_unfoldedArea = -1;
				if (gaussianValues_.size() == 0)
					computeGaussianDist(1);
			}

			inline void move(int x, int y) {
				m_roi.translate(x, y);
				m_area = m_unfoldedArea = -1;
			}

			inline void moveTo(ofRectangle roi) {
				m_roi = roi;
				m_area = m_unfoldedArea = -1;
			}

			inline float getArea() {
				if (m_area == -1)
					computeAreas();

				return m_area;
			}

			inline float getUnfoldedArea() {
				if (m_unfoldedArea == -1)
					computeAreas();

				return m_unfoldedArea;
			}

			inline float getDeformationPercent() {
				return (getArea() - getUnfoldedArea()) * 100 / getUnfoldedArea();
			}

			int getMeshIndex(int x, int y);

			std::vector<ofVec3f> getPoints();
			void setColor(ofColor color);

			bool insideMesh();

		private:
			interactiveGarment *const pGarment_;
			blobDetection::cloudPoint *mp_pointCloud;
			ofRectangle m_roi;
			float m_area;
			float m_unfoldedArea;

			void computeAreas();

			// ( cover 99.7 % of the gaussian distribution )
			static std::vector<float> gaussianValues_;
			static void computeGaussianDist(float sigma);
	};

} // namespace garment
} // namespace garmentAugmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_