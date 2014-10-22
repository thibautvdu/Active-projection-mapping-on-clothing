#include "flying_lights.h"

namespace garment_augmentation {
namespace garment {

	FlyingLights::FlyingLights() {
		lights_.reserve(100);
	}

	void FlyingLights::Update() {
		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].Update();
		}
	}

	void FlyingLights::Update(const std::vector<Fold> &folds) {
		if (!folds.empty()) {
			for (int i = 0; i < ofRandomf()*10; ++i) {
				const Fold &f = folds[floor(ofRandom(folds.size() - 1)+0.5)];
				ofVec3f foldDir = f.b_pt() - f.a_pt();
				lights_.push_back(Particle(f.a_pt(),0.01f,foldDir.length()));
				lights_[lights_.size()-1].vel_ = foldDir.normalize() * 0.03f;
			}
		}

		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].acc_ = ofVec3f(ofRandomf(), ofRandomf(), ofRandomf()) * 0.0005f;
			lights_[i].Update();
			if (lights_[i].life_ <= 0) {
				lights_.erase(lights_.begin() + i);
				i--;
			}
		}
	}

	void FlyingLights::Draw() {
		for (int i = 0; i < lights_.size(); ++i) {
			ofSetColor(255 * lights_[i].life_,0,0);
			lights_[i].draw();
			ofSetColor(255);
		}
	}

} // namespace garment
} // namespace garment_augmentation