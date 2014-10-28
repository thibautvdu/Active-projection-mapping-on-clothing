#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_

#include "animation.h"
#include "fold.h"

namespace garment_augmentation {
namespace garment {

	class FlyingLights : public Animation {
		public:

			FlyingLights();

			void Update();
			void Update(const std::vector<Fold> &folds);
			void Update(const std::vector<ofVec3f> &contour_points) {};
			void Draw();

			inline void set_light_radius(float radius) {
				light_radius_ = radius;
			}

			inline void set_light_speed(float speed) {
				light_speed_ = speed;
			}

			inline void set_light_acceleration(float acc) {
				light_acceleration_ = acc;
			}

		private:
			class Particle : public ofSpherePrimitive {
				public:
					ofVec3f vel_, acc_;
					ofVec3f source_;
					float life_distance_;
					float remaining_life_distance_;

					inline Particle(const ofVec3f pos, const float radius, const float life_distance) {
						this->setRadius(radius);
						this->setPosition(pos);
						source_ = pos;
						life_distance_ = life_distance*life_distance;
					}

					inline void Update() {
						vel_ += acc_;

						this->setPosition(this->getPosition() + vel_ * Animation::last_frame_time());

						remaining_life_distance_ = 1 - (source_.distanceSquared(this->getPosition()) / life_distance_);
					}
			};

			std::vector<Particle> lights_;
			float light_radius_;
			float light_speed_;
			float light_acceleration_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_