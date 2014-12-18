// A fire animation managing 25 animated fire flames. Please refer to the poster
// and paper for the animation details
// The video used as a texture on each particle is a collection PNG images that are
// read and stored as GPU textures when the animation is created

#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_

#include "ofxGif.h"

#include "animation.h"
#include "fold.h"

namespace garment_augmentation {
namespace garment {

	class FlyingLights : public Animation {
		public:

			// Init the particle system and store the video texture's frame on
			// the GPU. Refer to this function to change the speed, etc.
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
			// A class physic particle simulation, with added possibility
			// of drawing each particle with a video texture
			class Particle : public ofPlanePrimitive {
				public:

					Particle() {
						this->set(1, 1);
						vel_ = ofVec3f(0, 0, 0);
						life_time_ = 1;
						frame_index_ = 0;
					}

					Particle(ofVec3f init_pos) {
						this->setPosition(init_pos);
						this->set(1, 1);
						vel_ = ofVec3f(0, 0, 0);
						life_time_ = 1;
						frame_index_ = 0;
					}

					Particle(ofVec3f init_pos, float radius) {
						this->setPosition(init_pos);
						this->set(radius*2, radius*2);
						vel_ = ofVec3f(0, 0, 0);
						life_time_ = 1;
						frame_index_ = 0;
					}

					inline void InitUVTex(float radius, ofTexture &sample) {
						this->resizeToTexture(sample, radius * 2 / max(sample.getWidth(), sample.getHeight()));
					}

					inline void set_life_time(float life_time) {
						life_time_ = life_time;
					}

					inline float life_time() const {
						return life_time_;
					}

					inline int frame_index() const {
						return frame_index_;
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
						this->setPosition(this->getPosition() + delta_time*vel_);

					}

					// Draw the particle with the corresponding PNG frame of the video texture (according
					// to animation_increment)
					inline void Draw(std::vector<ofImage> &textures, float animation_increment) {
						frame_index_ += animation_increment;

						if (frame_index_ >= textures.size())
							frame_index_ = 0; 

						// Fade out the flame on the last second
						if (life_time_ < 1)
							ofSetColor(255, 255, 255, life_time_ * 255);

						textures[frame_index_].bind();
						this->draw();
						textures[frame_index_].unbind();
						if (life_time_ < 1)
							ofSetColor(255, 255, 255, 255);
					}

				private : 
					ofVec3f vel_, force_;
					float life_time_;
					float frame_index_;
			};

			std::vector<Particle> lights_;
			const int k_num_of_lights_;
			float light_radius_;
			float light_speed_;
			float light_repulsion_radius_;
			float light_repulsion_strength_;
			float light_life_time_;
			std::vector<ofImage> video_frames_images_;
			float video_fps_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FLYING_LIGHTS_H_