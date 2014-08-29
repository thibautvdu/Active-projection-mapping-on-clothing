#include "ofSemiImplicitActiveMesh.h"

namespace ofDeformationTracking {

	// INITIALIZATION
	void ofSemiImplicitActiveMesh::generateMesh(const ofPolyline& imageContour) {

	}

	void ofSemiImplicitActiveMesh::generateMesh(const ofPolyline& imageContour, const ofImage& contourMask, const ofRectangle roi) {
		this->clear();
		this->setMode(OF_PRIMITIVE_TRIANGLES);
		this->enableIndices();

		int rows = roi.height / mMeshYResolution;
		int columns = roi.width / mMeshXResolution;
		int stepY = roi.height / rows;
		int stepX = roi.width / columns;

		for (int iy = 0; iy < rows; iy++) {
			for (int ix = 0; ix < columns; ix++) {
				this->addVertex(ofVec3f(ix * stepX + roi.getTopLeft().x, iy * stepY + roi.getTopLeft().y, 0));
			}
		}

		for (int y = 0; y < rows - 1; y++) {
			for (int x = 0; x < columns - 1; x++) {
				
				// The square face
				ofVec3f pointA = this->getVertex((y)*columns + x);
				ofVec3f pointB = this->getVertex((y)*columns + x + 1);
				ofVec3f pointC = this->getVertex((y + 1)*columns + x + 1);
				ofVec3f pointD = this->getVertex((y + 1)*columns + x);
				ofPolyline rectangleFace;
				rectangleFace.addVertex(pointA);
				rectangleFace.addVertex(pointB);
				rectangleFace.addVertex(pointC);
				rectangleFace.addVertex(pointD);
				rectangleFace.close();

				if (intersectionArea(rectangleFace, imageContour) * 100 / rectangleFace.getArea() > mGenerationAreaThresh){
					// Triangle face 1
					this->addTriangle((y)*columns + x, (y)*columns + x + 1, (y + 1)*columns + x);

					// Triangle face 2
					this->addTriangle((y)*columns + x + 1, (y + 1)*columns + x + 1, (y + 1)*columns + x);
				}
			}
		}
	}

	// Optimize the process with the more little bounding rect
	int ofSemiImplicitActiveMesh::intersectionArea(const ofPolyline& contour1, const ofPolyline& contour2) {
		int area = 0;

		ofRectangle boundingBox1 = contour1.getBoundingBox();
		ofRectangle boundingBox2 = contour2.getBoundingBox();

		ofRectangle roi;
		if (boundingBox1.getArea() < boundingBox2.getArea())
			roi = boundingBox1;
		else
			roi = boundingBox2;

		for (int y = roi.getTopLeft().y; y < roi.height + roi.getTopLeft().y; ++y){
			for (int x = roi.getTopLeft().x; x < roi.width + roi.getTopLeft().x; ++x){
				if (contour1.inside(x, y) && contour2.inside(x, y))
					area++;
			}
		}

		return area;
	}


	// COMPUTATION
	void ofSemiImplicitActiveMesh::updateMesh(const ofPolyline& imageContour) {

	}

}