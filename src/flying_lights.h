#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_

#include "animation.h"
#include "fold.h"

namespace garment_augmentation {
namespace garment {

	class FlyingLights : public Animation {
		public:

			FlyingLights();

			void Update() {};
			void Update(const std::vector<Fold> &folds);
			void Update(const std::vector<ofVec3f> &contour_points) {};
			void Draw();

			inline bool RelativeToModel() { return true; }

			inline void set_light_radius(float radius) {
				light_radius_ = radius;
			}

			inline void set_light_speed(float speed) {
				light_speed_ = speed;
			}

		private:
			class Particle : public ofSpherePrimitive {
				public:

					Particle() {
						this->setRadius(1);
						vel_ = ofVec3f(0, 0, 0);
						life_time_ = 1;
					}

					Particle(ofVec3f init_pos) {
						this->setPosition(init_pos);
						this->setRadius(1);
						vel_ = ofVec3f(0, 0, 0);
						life_time_ = 1;
					}

					Particle(ofVec3f init_pos, float radius) {
						this->setPosition(init_pos);
						this->setRadius(radius);
						vel_ = ofVec3f(0, 0, 0);
						life_time_ = 1;
					}

					inline void set_life_time(float life_time) {
						life_time_ = life_time;
					}

					inline float life_time() {
						return life_time_;
					}

					inline void ResetForce() {
						force_ = ofVec3f::zero();
					}

					inline void AddForce(float x, float y, float z) {
						force_ += ofVec3f(x, y, z);
					}

					inline void AddForce(ofVec3f f) {
						force_ += f;
					}

					inline void AddDampingForce() {
					}

					inline void AddAttractionForce(float x, float y, float z, float radius, float strength) {
						ofVec3f diff = ofVec3f(x, y, z) - this->getPosition();
						float length = diff.length();

						if (length < radius) {
							float pct = 1 - (length / radius);
							diff /= length;
							force_ += diff * pct * strength;
						}
					}

					inline void AddRepulsionForce(float x, float y, float z, float radius, float strength) {
						ofVec3f diff = ofVec3f(x, y, z) - this->getPosition();
						float length = diff.length();

						if (length < radius) {
							float pct = 1 - (length / radius);
							diff /= length;
							force_ -= diff * pct * strength;
						}
					}

					inline void AddRepulsionForce(const Particle &p, float radius, float strength) {
						ofVec3f diff = p.getPosition() - this->getPosition();
						float length = diff.length();

						if (length < radius) {
							float pct = 1 - (length / radius);
							diff /= length;
							force_ -= diff * pct * strength;
						}
					}

					inline void Update(float delta_time) {
						life_time_ -= delta_time;
						vel_ += force_;
						this->setPosition(this->getPosition() + delta_time*force_);
					}

				private : 
					ofVec3f vel_, force_;
					float life_time_;
			};

			std::vector<Particle> lights_;
			float light_radius_;
			float light_speed_;
			float light_repulsion_radius_;
			float light_repulsion_strength_;
			float light_life_time_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_