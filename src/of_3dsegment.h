#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_MRPT_3DSEGMENT_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_MRPT_3DSEGMENT_H_

#include <ofMain.h>
#include <mrpt/math.h>

namespace garment_augmentation {
namespace math {

	using mrpt::math::TPoint3D;
	using mrpt::math::TLine3D;
	using mrpt::math::TSegment3D;

	class Of3dsegment {
		public :
			inline Of3dsegment() {
				Of3dsegment(0, 0, 0, 0, 0, 0);
			}

			inline Of3dsegment(const ofVec3f a, const ofVec3f b) : a_(a), b_(b) {
			}

			inline Of3dsegment(const float a_x, const float a_y, const float a_z, const float b_x, const float b_y, const float b_z) {
				a_ = ofVec3f(a_x, a_y, a_z);
				b_ = ofVec3f(b_x, b_y, b_z);
			}

			inline ofVec3f a() {
				return a_;
			}

			inline ofVec3f b() {
				return b_;
			}

		private :
			ofVec3f a_;
			ofVec3f b_;
	};

} // namespace math
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_MRPT_3DSEGMENT_H_