#include "flyingLights.h"

namespace garmentAugmentation {
namespace garment {

	flyingLights::flyingLights() {
		lights_.reserve(20);
	}

	void flyingLights::update() {
		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].update();
		}
	}

	void flyingLights::update(const std::vector<fold> &folds) {
		if (!folds.empty()) {
			for (int i = 0; i < ofRandomf()*2; ++i) {
				const fold &f = folds[floor(ofRandom(folds.size() - 1)+0.5)];
				ofVec3f foldDir = f.getBPt() - f.getAPt();
				lights_.push_back(particle(f.getAPt(),0.01f,foldDir.length()));
				lights_[lights_.size()-1].vel = foldDir.normalize() * 0.01f;
			}
		}

		for (int i = 0; i < lights_.size(); ++i) {
			lights_[i].acc = ofVec3f(ofRandomf(), ofRandomf(), ofRandomf()) * 0.00005f;
			lights_[i].update();
			if (lights_[i].life <= 0) {
				lights_.erase(lights_.begin() + i);
				i--;
			}
		}
	}

	void flyingLights::draw() {
		for (int i = 0; i < lights_.size(); ++i) {
			ofSetColor(255 * lights_[i].life,0,0);
			lights_[i].draw();
			ofSetColor(255);
		}
	}

} // namespace garment
} // namespace garmentAugmentation