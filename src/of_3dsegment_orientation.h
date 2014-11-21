#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_3DSEGMENT_ORIENTATION_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_3DSEGMENT_ORIENTATION_H_

#include <ofMain.h>
#include <mrpt/math.h>

using mrpt::math::TSegment3D;
using mrpt::math::TPoint3D;

namespace garment_augmentation {
	namespace math {

		class Of3dsegmentOrientation {
		public:
			inline Of3dsegmentOrientation() {}

			inline Of3dsegmentOrientation(ofVec3f orientation, ofVec3f center, float length) : orientation_(orientation), center_(center), length_(length) {
			}

			inline Of3dsegmentOrientation(Eigen::Vector3d orientation, Eigen::Vector3d center, double length){
				orientation_ = ofVec3f(orientation.x(), orientation.y(), orientation.z());
				center_ = ofVec3f(center.x(), center.y(), center.z());
				length_ = length;
			}

			inline Of3dsegmentOrientation(TSegment3D mrpt_segment){
				ofVec3f a(mrpt_segment.point1.x, mrpt_segment.point1.y, mrpt_segment.point1.z);
				ofVec3f b(mrpt_segment.point2.x, mrpt_segment.point2.y, mrpt_segment.point2.z);
				orientation_ = (b-a).getNormalized();
				center_ = (a+b) / 2;
				length_ = (b-a).length();
			}

			inline const ofVec3f &orientation() const {
				return orientation_;
			}

			inline const ofVec3f &center() const {
				return center_;
			}

			inline float length() const {
				return length_;
			}

			inline ofVec3f a() const {
				return (center_ + orientation_ * length_  * 0.5);
			}

			inline ofVec3f b() const {
				return (center_ - orientation_ * length_  * 0.5);
			}

			TSegment3D ToMrpt3dSegment() const {
				float half_length = length_ * 0.5;
				ofVec3f a = (center_ + orientation_*half_length);
				ofVec3f b = (center_ - orientation_*half_length);

				return TSegment3D(TPoint3D(a.x, a.y, a.z), TPoint3D(b.x, b.y, b.z));
			}

		private:
			ofVec3f orientation_;
			ofVec3f center_;
			float length_;
		};

	} // namespace math
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_3DSEGMENT_ORIENTATION_H_