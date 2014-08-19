#ifndef FLEXIBLE_SURFACE_AUGMENTATION_LSCM_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_LSCM_H_

#include <fstream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <set>
#include <cmath>
#include <cassert>
#include <string>
#include <iostream>
#include <algorithm>
#include <sstream>

#include <NL/nl.h>
#include <ofMesh.h>

#include "lscmUtilities.h"

class LSCM {
	public:
		LSCM(ofMesh & m) : mMesh(&m) {}

		// Outline of the algorithm:

		// 1) Find an initial solution by projecting on a plane
		// 2) Lock two vertices of the mesh
		// 3) Copy the initial u,v coordinates to OpenNL
		// 3) Construct the LSCM equation with OpenNL
		// 4) Solve the equation with OpenNL
		// 5) Copy OpenNL solution to the u,v coordinates
		void apply();

		double umax, umin, vmax, vmin;

	protected:
		void setup_lscm();

		// Note: no-need to triangulate the facet,
		// we can do that "virtually", by creating triangles
		// radiating around vertex 0 of the facet.
		// (however, this may be invalid for concave facets)
		void setup_lscm(const Vector3& v0, const ofIndexType i0, const Vector3& v1, const ofIndexType i1, const Vector3& v2, const ofIndexType i2);

		// Computes the coordinates of the vertices of a triangle
		// in a local 2D orthonormal basis of the triangle's plane.
		static void project_triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2, Vector2& z0, Vector2& z1, Vector2& z2);

		// LSCM equation, geometric form :
		// (Z1 - Z0)(U2 - U0) = (Z2 - Z0)(U1 - U0)
		// Where Uk = uk + i.vk is the complex number 
		//                       corresponding to (u,v) coords
		//       Zk = xk + i.yk is the complex number 
		//                       corresponding to local (x,y) coords
		// cool: no divide with this expression,
		//  makes it more numerically stable in
		//  the presence of degenerate triangles.
		void setup_conformal_map_relations(const Vector3& p0, const ofIndexType, const Vector3& p1, const ofIndexType, const Vector3& p2, const ofIndexType);
		void solver_to_mesh(); // copies u,v coordinates from OpenNL solver to the mesh.
		void mesh_to_solver(); // copies u,v coordinates from the mesh to OpenNL solver.
		void project(); // Chooses an initial solution, and locks two vertices

		ofMesh * mMesh;
		int vxminLocked, vxmaxLocked;
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_LSCM_H_