#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_

#include "ofMain.h"

#include "animation.h"
#include "fold.h"

namespace garmentAugmentation {
namespace garment {

	class flyingLights : public animation {
		public:
			flyingLights();
			void update();
			void update(const std::vector<fold> &folds);
			void draw();

		private:
			class particle : public ofSpherePrimitive {
			public:
				ofVec3f vel, acc;
				ofVec3f source;
				float lifeTime;
				float life;

				particle(ofVec3f pos, float radius, float lifeDistance) {
					this->setRadius(ofRandomf() * radius);
					this->setPosition(pos);
					source = pos;
					lifeTime = lifeDistance*lifeDistance;
				}

				void update() {
					vel += acc;

					this->setPosition(this->getPosition() + vel);
					acc.set(0, 0, 0);

					life = 1 - (source.distanceSquared(this->getPosition()) / lifeTime);
				}
			};

		std::vector<particle> lights;
	};

} // namespace garment
} // namespace garmentAugmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_