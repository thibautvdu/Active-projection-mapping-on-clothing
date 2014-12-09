#include "flying_lights.h"

namespace garment_augmentation {
namespace garment {

	FlyingLights::FlyingLights() {
		lights_.reserve(20);
		light_radius_ = 0.2f;
		light_speed_ = 0.15f; // 15 cm/s
		light_repulsion_radius_ = light_radius_;
		light_repulsion_strength_ = light_radius_ * light_speed_ * 0.001;
		light_life_time_ = 10; // seconds

		// Prepare an array of textures
		ofxGifDecoder gif_animation_handler;
		ofxGifFile gif_animation;

		gif_animation_handler.decode("videos/bubbling_fire.gif");
		gif_animation = gif_animation_handler.getFile();

		for (int i = 0; i < gif_animation.getNumFrames(); ++i) {
			gif_frames_textures_.push_back(ofTexture());
			gif_frames_textures_.back().loadData(*gif_animation.getFrameAt(i)->getRawPixels());
		}

		gif_fps_ = 10;
	}

	void FlyingLights::Update(const std::vector<Fold> &folds) {
		for (int i = 0; i < lights_.size(); ++i) {
			if (lights_[i].life_time() <= 0)
				lights_.erase(lights_.begin() + i);
			else
				lights_[i].ResetForce();
		}


		if (!folds.empty()) {
			int living_particles_num = lights_.size();
			for (int i = 0; i < 20 - living_particles_num; ++i) {
				const Fold &f = folds[floor(ofRandom(folds.size() - 1) + 0.5)];
				ofVec3f foldDir = f.a_pt() - f.b_pt();
				lights_.push_back(Particle(f.b_pt() + ofRandomuf() * foldDir));
				lights_.back().AddForce(ofRandomuf() * foldDir.getNormalized() * light_speed_);
				lights_.back().AddForce(ofVec3f(ofRandomf(), ofRandomf(), ofRandomf()).normalize() * light_speed_ * 0.1);
				lights_.back().set_life_time(light_life_time_*ofRandomuf());
				lights_.back().InitUVTex(foldDir.length() * ofRandomuf() * light_radius_, gif_frames_textures_[0]);
			}
		}

		// Mutual repulsion of particles
		for (int i = 0; i < lights_.size(); ++i) {
		for (int j = 0; j < i; j++) {
		lights_[i].AddRepulsionForce(lights_[j], light_repulsion_radius_, light_repulsion_strength_);
		}
		}

		RefreshTime();
		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].Update(last_frame_time());
		}
	}

	void FlyingLights::Draw() {
		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].Draw(gif_frames_textures_, gif_fps_ * last_frame_time());
		}
	}

} // namespace garment
} // namespace garment_augmentation