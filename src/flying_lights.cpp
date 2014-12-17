#include "flying_lights.h"

namespace garment_augmentation {
namespace garment {

	FlyingLights::FlyingLights() {
		lights_.reserve(20);
		light_radius_ = 0.04f;
		light_speed_ = 0.15f; // 15 cm/s
		light_repulsion_radius_ = light_radius_ * 4;
		light_repulsion_strength_ = light_radius_ * light_speed_ * 0.1;
		light_life_time_ = 4; // seconds

		// Prepare an array of textures
		/*
		ofxGIF::fiGifLoader gif_animation_handler_;
		gif_animation_handler_.load("videos/bubbling_fire.gif");

		for (int i = 0; i < gif_animation_handler_.pages.size(); ++i) {
		gif_frames_textures_.push_back(ofTexture());
		gif_frames_textures_.back().loadData(gif_animation_handler_.pages[i].getPixelsRef());
		}*/

		std::stringstream file_name;
		for (int i = 1; i <= 50; ++i) {
			gif_frames_images_.push_back(ofImage());

			file_name << "videos/fire_1/fire1_ ";

			if (i < 10) {
				file_name << "0" << i << ".png";
			}
			else {
				file_name << i << ".png";
			}

			gif_frames_images_.back().loadImage(file_name.str());
			file_name.str(std::string());
			file_name.clear();
		}

		gif_fps_ = 15;
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
			float light_radius_variation = light_radius_ * 0.5;
			for (int i = 0; i < 20 - living_particles_num; ++i) {
				const Fold &f = folds[floor(ofRandom(folds.size() - 1) + 0.5)];
				ofVec3f foldDir = f.a_pt() - f.b_pt();
				lights_.push_back(Particle(f.b_pt() + ofRandomuf() * foldDir));
				lights_.back().AddForce(ofRandom(0.5,1) * foldDir.getNormalized() * light_speed_);
				lights_.back().AddForce(ofVec3f(ofRandomf(), ofRandomf(), ofRandomf()).normalize() * light_speed_ * 0.1);
				lights_.back().set_life_time(light_life_time_*ofRandomuf());
				lights_.back().InitUVTex(ofRandom(light_radius_ - light_radius_variation, light_radius_ + light_radius_variation), gif_frames_images_[0].getTextureReference());
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
		ofEnableAlphaBlending();
		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].Draw(gif_frames_images_, gif_fps_ * last_frame_time());
		}
		ofDisableAlphaBlending();
	}

} // namespace garment
} // namespace garment_augmentation