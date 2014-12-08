#include "flying_lights.h"

namespace garment_augmentation {
namespace garment {

	FlyingLights::FlyingLights() {
		lights_.reserve(100);
		light_radius_ = 0.01f;
		light_speed_ = 0.2f; // 20 cm/s
		light_repulsion_radius_ = light_radius_ * 10;
		light_repulsion_strength_ = light_radius_;
		light_life_time_ = 2.5; // seconds
	}

	void FlyingLights::Update(const std::vector<Fold> &folds) {
		for (int i = 0; i < lights_.size(); ++i) {
			if (lights_[i].life_time() <= 0)
				lights_.erase(lights_.begin() + i);
			else
				lights_[i].ResetForce();
		}

		// Mutual repulsion of particles
		for (int i = 0; i < lights_.size(); ++i) {
			for (int j = 0; j < i; j++) {
				lights_[i].AddRepulsionForce(lights_[j], light_repulsion_radius_, light_repulsion_strength_);
			}
		}

		if (!folds.empty()) {
			int living_particles_num = lights_.size();
			for (int i = 0; i < 100 - living_particles_num; ++i) {
				const Fold &f = folds[floor(ofRandom(folds.size() - 1) + 0.5)];
				ofVec3f foldDir = f.b_pt() - f.a_pt();
				lights_.push_back(Particle(f.a_pt() + ofRandomuf() * foldDir, foldDir.length() * ofRandomuf() * light_radius_));
				lights_[lights_.size() - 1].AddForce(ofVec3f(ofRandomf(), ofRandomf(), ofRandomf()).normalize() * light_speed_);
				lights_[lights_.size() - 1].set_life_time(light_life_time_*ofRandomuf());
			}
		}

		RefreshTime();
		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].Update(last_frame_time());
		}
	}

	void FlyingLights::Draw() {
		ofSetColor(ofColor::red);
		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].draw();
		}
		ofSetColor(255);
	}

} // namespace garment
} // namespace garment_augmentation