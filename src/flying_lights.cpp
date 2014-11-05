#include "flying_lights.h"

namespace garment_augmentation {
namespace garment {

	FlyingLights::FlyingLights() {
		lights_.reserve(100);
		light_radius_ = 0.005f;
		light_speed_ = 0.2f; // 20 cm/s
		light_acceleration_ = 0.001f;
	}

	void FlyingLights::Update(const std::vector<Fold> &folds) {
		RefreshTime();

		if (!folds.empty()) {
			int living_particles_num = lights_.size();
			for (int i = 0; i < 100 - living_particles_num; ++i) {
				const Fold &f = folds[floor(ofRandom(folds.size() - 1)+0.5)];
				ofVec3f foldDir = f.b_pt() - f.a_pt();
				lights_.push_back(Particle(f.a_pt() + ofRandomuf() * foldDir, ofRandomuf() * light_radius_, foldDir.length()/10));
				lights_[lights_.size() - 1].vel_ = ofVec3f(ofRandomf(), ofRandomf(), ofRandomf()).normalize() * light_speed_;
			}
		}

		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].acc_ = ofVec3f(ofRandomf(), ofRandomf(), ofRandomf()).normalize() * light_acceleration_;
			lights_[i].Update();
			if (lights_[i].remaining_life_distance_ <= 0) {
				lights_.erase(lights_.begin() + i);
				i--;
			}
		}
	}

	void FlyingLights::Draw() {
		for (int i = 0; i < lights_.size(); ++i) {
			ofSetColor(0,0,255 * lights_[i].remaining_life_distance_);
			lights_[i].draw();
			ofSetColor(255);
		}
	}

} // namespace garment
} // namespace garment_augmentation