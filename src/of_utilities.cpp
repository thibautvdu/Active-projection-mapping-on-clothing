#include "of_utilities.h"

namespace of_utilities {

	void VirtualWindow::Begin() const {
		ofPushMatrix();
		ofTranslate(x_position_, y_position_);
	}

	void VirtualWindow::End() const {
		ofPopMatrix();
	}

	void VirtualWindow::background(const ofColor color) const {
		ofSetColor(color);
		ofFill();
		ofRect(x_position_, y_position_, width_, height_);
		ofNoFill();
		ofSetColor(255);
	}

	ofMatrix3x3 VectorRotationMatrix(ofVec3f from, ofVec3f to) {
		ofVec3f crossed = from.getCrossed(to);
		float dot = from.dot(to);
		float crossed_norm = crossed.length();

		ofMatrix3x3 local_rotation(dot, -crossed_norm, 0.f,
									crossed_norm, dot, 0.f,
									0.f, 0.f, 1.f);

		ofVec3f third_base = (to - dot*from).getNormalized();
		ofMatrix3x3 to_local_space(from.x, third_base.x, -crossed.x,
									from.y, third_base.y, -crossed.y,
									from.z, third_base.z, -crossed.z);
		ofMatrix3x3 to_world_space = to_local_space;
		to_world_space.invert();

		return to_local_space * local_rotation * to_world_space;
	}

	// Map function for ofVec2f
	ofVec2f MapVec2f(const ofVec2f value, const ofVec2f inputMin, const ofVec2f inputMax, const ofVec2f outputMin, const ofVec2f outputMax, const bool clamp) {
		float x = ofMap(value.x, inputMin.x, inputMax.x, outputMin.x, outputMax.x, clamp);
		float y = ofMap(value.y, inputMin.y, inputMax.y, outputMin.y, outputMax.y, clamp);

		return ofVec2f(x, y);
	}

	ofPath PolylineToPath(const ofPolyline polyline) {
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