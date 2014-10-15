#include "ofFastMesh.h"

void ofFastMesh::reserveCapacity(const int capacity) {
	this->vertices.reserve(capacity);
	//this->normals.reserve(capacity);
	this->colors.reserve(capacity);
	//this->texCoords.reserve(capacity);
	//this->indices.reserve(capacity * 4);
}

void ofFastMesh::resize(const int size) {
	this->vertices.resize(size);
	this->colors.resize(size);
	bVertsChanged = true;
	bColorsChanged = true;
	bFacesDirty = true;
}

void ofFastMesh::computeNormals(const bool bNormalize){
	for (int i = 0; i < this->getVertices().size(); i++) this->addNormal(ofPoint(0, 0, 0));

	for (int i = 0; i < this->getIndices().size(); i += 3){
		const int ia = this->getIndices()[i];
		const int ib = this->getIndices()[i + 1];
		const int ic = this->getIndices()[i + 2];
		ofVec3f a = this->getVertices()[ia];
		ofVec3f b = this->getVertices()[ib];
		ofVec3f c = this->getVertices()[ic];

		ofVec3f e1 = a - b;
		ofVec3f e2 = c - b;
		ofVec3f no = e2;
		no.cross(e1);

		this->getNormals()[ia] += no;
		this->getNormals()[ib] += no;
		this->getNormals()[ic] += no;
	}

	if (bNormalize)
	for (int i = 0; i < this->getNormals().size(); i++) {
		this->getNormals()[i].normalize();
	}
}