// Overwrite the openframeworks mesh for improved performance. Openframeworks' meshes use a
// c++ vector data structure, but doesn't offer the possibity of pre-allocating its size
// resulting in excessive re-allocation when adding vertices. Also add a method to compute the
// normals of the mesh

#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_FAST_MESH_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_FAST_MESH_H_

#include "ofMesh.h"

class ofFastMesh : public ofMesh {

	public:
		// c++ reserve function on the colors and verices' vectors
		void reserveCapacity(const int capacity);

		// c++ resize function on the colors and verices' vectors
		void resize(const int size);

		int getCapacity() const { 
			return this->vertices.capacity(); 
		};

		// compute the normals of the mesh !!! the mesh's indices must be set before calling this funciton !!!
		void computeNormals(const bool bNormalize);
};

#endif