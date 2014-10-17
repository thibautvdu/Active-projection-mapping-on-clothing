#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_

#include "ofMain.h"

#include "animation.h"
#include "fold.h"

namespace garment_augmentation {
namespace garment {

	class FlyingLights : public Animation {
		public:

			FlyingLights();

			void Update();
			void Update(const std::vector<Fold> &folds);
			void Draw();

		private:
			class Particle : public ofSpherePrimitive {
				public:
					ofVec3f vel_, acc_;
					ofVec3f source_;
					float life_time_;
					float life_;

					inline Particle(const ofVec3f pos, const float radius, const float life_distance) {
						this->setRadius(ofRandomf() * radius);
						this->setPosition(pos);
						source_ = pos;
						life_time_ = life_distance*life_distance;
					}

					inline void Update() {
						vel_ += acc_;

						this->setPosition(this->getPosition() + vel_);
						acc_.set(0, 0, 0);

						life_ = 1 - (source_.distanceSquared(this->getPosition()) / life_time_);
					}
			};

			std::vector<Particle> lights_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_