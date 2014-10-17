#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_MRPT_3DSEGMENT_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_MRPT_3DSEGMENT_H_

#include <ofMain.h>
#include <mrpt/math.h>

namespace garment_augmentation {
namespace math {

	using mrpt::math::TPoint3D;
	using mrpt::math::TLine3D;
	using mrpt::math::TSegment3D;

	class OfEigen3dsegment {
		public :
			inline OfEigen3dsegment() {
				OfEigen3dsegment(0, 0, 0, 0, 0, 0);
			}

			inline OfEigen3dsegment(const Eigen::Vector3d a, const Eigen::Vector3d b) : a_double_(a), b_double_(b) {
			}

			inline OfEigen3dsegment(const double a_x, const double a_y, const double a_z, const double b_x, const double b_y, const double b_z) {
				a_double_ = Eigen::Vector3d(a_x, a_y, a_z);
				b_double_ = Eigen::Vector3d(b_x, b_y, b_z);
			}

			inline ofVec3f a() {
				return ofVec3f(a_double_.x(), a_double_.y(), a_double_.z());
			}

			inline ofVec3f b() {
				return ofVec3f(b_double_.x(), b_double_.y(), b_double_.z());
			}

			inline Eigen::Vector3d a_double() {
				return a_double_;
			}

			inline Eigen::Vector3d b_double() {
				return b_double_;
			}

		private :
			Eigen::Vector3d a_double_;
			Eigen::Vector3d b_double_;
	};

} // namespace math
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_MRPT_3DSEGMENT_H_