#include "ofUtilities.h"

namespace ofUtilities {

	void ofVirtualWindow::begin() const {
		ofPushMatrix();
		ofTranslate(m_xPosition, m_yPosition);
	}

	void ofVirtualWindow::end() const {
		ofPopMatrix();
	}

	void ofVirtualWindow::background(ofColor color) const {
		ofSetColor(color);
		ofFill();
		ofRect(m_xPosition, m_yPosition, m_width, m_height);
		ofNoFill();
		ofSetColor(255);
	}

	// Map function for ofVec2f
	ofVec2f mapVec2f(ofVec2f value, ofVec2f inputMin, ofVec2f inputMax, ofVec2f outputMin, ofVec2f outputMax, bool clamp) {
		float x = ofMap(value.x, inputMin.x, inputMax.x, outputMin.x, outputMax.x, clamp);
		float y = ofMap(value.y, inputMin.y, inputMax.y, outputMin.y, outputMax.y, clamp);

		return ofVec2f(x, y);
	}

	ofPath polylineToPath(ofPolyline polyline) {
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