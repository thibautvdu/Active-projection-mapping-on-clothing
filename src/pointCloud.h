#ifndef FLEXIBLE_SURFACE_AUGMENTATION_POINT_CLOUD_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_POINT_CLOUD_H_

#include "ofMain.h"

namespace garmentAugmentation {
namespace blobDetection {

	enum cloudPointFlag { FLAG_OFF_THRESHOLD = -5, FLAG_BACKGROUND = -4, FLAG_IDLE = -3, FLAG_QUEUED = -2, FLAG_PROCESSED = -1 };

	class cloudPoint {
		public:
			cloudPoint() {
				boundary_ = false;
			}

			int flag_;
			bool boundary_;
			ofVec3f pos_;
	};

} // namespace blobDetection
} // namespace garmentAugmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_POINT_CLOUD_H_