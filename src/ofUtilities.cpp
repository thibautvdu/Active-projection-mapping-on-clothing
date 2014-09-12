#include "ofUtilities.h"

namespace ofUtilities {

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

	void computeNormals(ofMesh& mesh, bool bNormalize){
		for (int i = 0; i < mesh.getVertices().size(); i++) mesh.addNormal(ofPoint(0, 0, 0));

		for (int i = 0; i < mesh.getIndices().size(); i += 3){
			const int ia = mesh.getIndices()[i];
			const int ib = mesh.getIndices()[i + 1];
			const int ic = mesh.getIndices()[i + 2];
			ofVec3f a = mesh.getVertices()[ia];
			ofVec3f b = mesh.getVertices()[ib];
			ofVec3f c = mesh.getVertices()[ic];

			ofVec3f e1 = a - b;
			ofVec3f e2 = c - b;
			ofVec3f no = e2;
			no.cross(e1);

			mesh.getNormals()[ia] += no;
			mesh.getNormals()[ib] += no;
			mesh.getNormals()[ic] += no;
		}

		if (bNormalize)
		for (int i = 0; i < mesh.getNormals().size(); i++) {
			mesh.getNormals()[i].normalize();
		}
	}
}