#include "flyingLights.h"

namespace garmentAugmentation {

	flyingLights::flyingLights() {
		lights.reserve(20);
	}

	void flyingLights::update() {
		for (int i = 0; i < lights.size(); ++i) {
			lights[i].update();
		}
	}

	void flyingLights::update(std::vector<fold> &folds) {
		if (!folds.empty()) {
			for (int i = 0; i < ofRandomf()*2; ++i) {
				fold &f = folds[floor(ofRandom(folds.size() - 1)+0.5)];
				ofVec3f foldDir = f.getBPt() - f.getAPt();
				lights.push_back(particle(f.getAPt(),0.01f,foldDir.length()));
				lights[lights.size()-1].vel = foldDir.normalize() * 0.01f;
			}
		}

		for (int i = 0; i < lights.size(); ++i) {
			lights[i].acc = ofVec3f(ofRandomf(), ofRandomf(), ofRandomf()) * 0.00005f;
			lights[i].update();
			if (lights[i].life <= 0) {
				lights.erase(lights.begin() + i);
				i--;
			}
		}
	}

	void flyingLights::draw() {
		for (int i = 0; i < lights.size(); ++i) {
			ofSetColor(255 * lights[i].life,0,0);
			lights[i].draw();
			ofSetColor(255);
		}
	}

};