#ifndef FLEXIBLE_SURFACE_AUGMENTATION_POINT_CLOUD_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_POINT_CLOUD_H_

#include "ofMain.h"

namespace garmentAugmentation {
	enum cloudPointFlag { FLAG_OFF_THRESHOLD = -5, FLAG_BACKGROUND = -4, FLAG_IDLE = -3, FLAG_QUEUED = -2, FLAG_PROCESSED = -1 };

	class cloudPoint {
	public:
		cloudPoint() {
			boundary = false;
		}

		int flag;
		bool boundary;
		ofVec3f pos;
	};
};

#endif // !FLEXIBLE_SURFACE_AUGMENTATION_POINT_CLOUD_H_