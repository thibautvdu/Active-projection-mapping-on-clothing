#ifndef FLEXIBLE_SURFACE_AUGMENTATION_ANIMATION_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_ANIMATION_H_

#include "fold.h"

namespace garmentAugmentation {

	class animation {
		public:
			virtual void update() = 0;
			virtual void update(std::vector<fold> &folds) = 0;
			virtual void draw() = 0;

		private:

	};
};

#endif // !FLEXIBLE_SURFACE_AUGMENTATION_ANIMATION_H_