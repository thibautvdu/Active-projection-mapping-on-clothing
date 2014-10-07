#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_FAST_MESH_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_FAST_MESH_H_

#include "ofMesh.h"

class ofFastMesh : public ofMesh {

	public:
		void reserveCapacity(int capacity);
		void resize(int size);
		int getCapacity() const { return this->vertices.capacity(); };
		void computeNormals(bool bNormalize);
};

#endif