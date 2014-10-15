#ifndef FLEXIBLE_SURFACE_AUGMENTATION_ANIMATION_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_ANIMATION_H_

#include "fold.h"

namespace garmentAugmentation {
namespace garment {

	class animation {
		public:
			virtual void update() = 0;
			virtual void update(const std::vector<fold> &folds) = 0;
			virtual void draw() = 0;

	};

} // namespace garment
} // namespace garmentAugmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_ANIMATION_H_