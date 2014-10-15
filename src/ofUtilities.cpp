#include "ofUtilities.h"

namespace ofUtilities {

	void ofVirtualWindow::begin() const {
		ofPushMatrix();
		ofTranslate(xPosition_, yPosition_);
	}

	void ofVirtualWindow::end() const {
		ofPopMatrix();
	}

	void ofVirtualWindow::background(const ofColor color) const {
		ofSetColor(color);
		ofFill();
		ofRect(xPosition_, yPosition_, width_, height_);
		ofNoFill();
		ofSetColor(255);
	}

	// Map function for ofVec2f
	ofVec2f mapVec2f(const ofVec2f value, const ofVec2f inputMin, const ofVec2f inputMax, const ofVec2f outputMin, const ofVec2f outputMax, const bool clamp) {
		float x = ofMap(value.x, inputMin.x, inputMax.x, outputMin.x, outputMax.x, clamp);
		float y = ofMap(value.y, inputMin.y, inputMax.y, outputMin.y, outputMax.y, clamp);

		return ofVec2f(x, y);
	}

	ofPath polylineToPath(const ofPolyline polyline) {
		ofPath path;
		for (int i = 0; i < polyline.getVertices().size(); i++){
			if (i == 0)
				path.moveTo(polyline.getVertices()[i].x, polyline.getVertices()[i].y);
			else
				path.lineTo(polyline.getVertices()[i].x, polyline.getVertices()[i].y);
		}
		path.close();

		return path;
	}
}