#include "lscm.h"

void LSCM::apply() {
	int nb_vertices = mMesh->getNumVertices(); // MODIFIED
	project();
	nlNewContext();

	nlSolverParameteri(NL_SOLVER, NL_CG);
	nlSolverParameteri(NL_PRECONDITIONER, NL_PRECOND_JACOBI);


	nlSolverParameteri(NL_NB_VARIABLES, 2 * nb_vertices);
	nlSolverParameteri(NL_LEAST_SQUARES, NL_TRUE);
	nlSolverParameteri(NL_MAX_ITERATIONS, 5 * nb_vertices);
	nlSolverParameterd(NL_THRESHOLD, 1e-10);
	nlBegin(NL_SYSTEM);
	mesh_to_solver();
	nlBegin(NL_MATRIX);
	setup_lscm();
	nlEnd(NL_MATRIX);
	nlEnd(NL_SYSTEM);
	nlSolve();
	solver_to_mesh();
	double time;
	NLint iterations;
	nlGetDoublev(NL_ELAPSED_TIME, &time);
	nlGetIntergerv(NL_USED_ITERATIONS, &iterations);
	nlDeleteContext(nlGetCurrent());
}

void LSCM::setup_lscm() {
	std::vector<ofIndexType> indices = mMesh->getIndices();
	for (unsigned int i = 0; i < indices.size(); i += 3) {
		setup_lscm(mMesh->getVertex(indices[i]), indices[i], mMesh->getVertex(indices[i+1]), indices[i+1], mMesh->getVertex(indices[i+2]), indices[i+2]);
	}
}

// Note: no-need to triangulate the facet,
// we can do that "virtually", by creating triangles
// radiating around vertex 0 of the facet.
// (however, this may be invalid for concave facets)
void LSCM::setup_lscm(const ofVec3f v0, ofIndexType i0, const ofVec3f v1, ofIndexType i1, const ofVec3f v2, ofIndexType i2) {  //FIXME HERE
	setup_conformal_map_relations(v0, i0, v1, i1, v2, i2);
}

// Computes the coordinates of the vertices of a triangle
// in a local 2D orthonormal basis of the triangle's plane.
void LSCM::project_triangle(
	const ofVec3f p0,
	const ofVec3f p1,
	const ofVec3f p2,
	ofVec2f& z0,
	ofVec2f& z1,
	ofVec2f& z2
	) {
	ofVec3f X = p1 - p0;
	X.normalize();
	ofVec3f p2mp0 = (p2 - p0);
	ofVec3f Z = ofVec3f(X.y * p2mp0.z - X.z * p2mp0.y, X.z * p2mp0.x - X.x * p2mp0.z, X.x * p2mp0.y - X.y * p2mp0.x);
	Z.normalize();
	ofVec3f Y = ofVec3f(Z.y * X.z - Z.z * X.y, Z.z * X.x - Z.x * X.z, Z.x * X.y - Z.y * X.x);
	const ofVec3f& O = p0;

	float x0 = 0;
	float y0 = 0;
	float x1 = (p1 - O).length();
	float y1 = 0;
	ofVec3f x2vec = (p2 - O) * X;
	float x2 = x2vec.x + x2vec.y + x2vec.z;
	ofVec3f y2vec = (p2 - O) * Y;
	float y2 = y2vec.x + y2vec.y + y2vec.z;

	z0 = ofVec2f(x0, y0);
	z1 = ofVec2f(x1, y1);
	z2 = ofVec2f(x2, y2);
}

// LSCM equation, geometric form :
// (Z1 - Z0)(U2 - U0) = (Z2 - Z0)(U1 - U0)
// Where Uk = uk + i.vk is the complex number 
//                       corresponding to (u,v) coords
//       Zk = xk + i.yk is the complex number 
//                       corresponding to local (x,y) coords
// cool: no divide with this expression,
//  makes it more numerically stable in
//  the presence of degenerate triangles.

void LSCM::setup_conformal_map_relations(
	ofVec3f p0, int id0, ofVec3f p1, int id1, ofVec3f p2, int id2
	) {

	ofVec2f z0, z1, z2;
	project_triangle(p0, p1, p2, z0, z1, z2);
	ofVec2f z01 = z1 - z0;
	ofVec2f z02 = z2 - z0;
	double a = z01.x;
	double b = z01.y;
	double c = z02.x;
	double d = z02.y;
	assert(b == 0.0);

	// Note  : 2*id + 0 --> u
	//         2*id + 1 --> v
	int u0_id = 2 * id0;
	int v0_id = 2 * id0 + 1;
	int u1_id = 2 * id1;
	int v1_id = 2 * id1 + 1;
	int u2_id = 2 * id2;
	int v2_id = 2 * id2 + 1;

	// Note : b = 0

	// Real part
	nlBegin(NL_ROW);
	nlCoefficient(u0_id, -a + c);
	nlCoefficient(v0_id, b - d);
	nlCoefficient(u1_id, -c);
	nlCoefficient(v1_id, d);
	nlCoefficient(u2_id, a);
	nlEnd(NL_ROW);

	// Imaginary part
	nlBegin(NL_ROW);
	nlCoefficient(u0_id, -b + d);
	nlCoefficient(v0_id, -a + c);
	nlCoefficient(u1_id, -d);
	nlCoefficient(v1_id, -c);
	nlCoefficient(v2_id, a);
	nlEnd(NL_ROW);
}

/**
* copies u,v coordinates from OpenNL solver to the mesh.
*/
void LSCM::solver_to_mesh() {
	float u = nlGetVariable(0);
	float v = nlGetVariable(1);
	umax = u; umin = u; vmax = v; vmin = v;

	for (unsigned int i = 0; i<mMesh->getNumVertices(); i++) {
		u = nlGetVariable(2 * i);
		v = nlGetVariable(2 * i + 1);
		mMesh->setTexCoord(i,ofVec2f(u, v));
		umax = u > umax ? u : umax;
		umin = u < umin ? u : umin;
		vmax = v > vmax ? v : vmax;
		vmin = v < vmin ? v : vmin;
	}
}

/**
* copies u,v coordinates from the mesh to OpenNL solver.
*/
void LSCM::mesh_to_solver() {
	for (unsigned int i = 0; i<mMesh->getNumVertices(); i++) {
		float u = mMesh->getTexCoord(i).x;
		float v = mMesh->getTexCoord(i).y;
		nlSetVariable(2 * i, u);
		nlSetVariable(2 * i + 1, v);
		if (i == vxminLocked || i == vxmaxLocked) {
			nlLockVariable(2 * i);
			nlLockVariable(2 * i + 1);
		}
	}
}

// Chooses an initial solution, and locks two vertices
void LSCM::project() {
	// Get bbox
	unsigned int i;

	float xmin = 1e5;
	float ymin = 1e5;
	float zmin = 1e5;
	float xmax = -1e5;
	float ymax = -1e5;
	float zmax = -1e5;

	for (i = 0; i<mMesh->getNumVertices(); i++) {
		const ofVec3f v = mMesh->getVertex(i);
		xmin = std::min(v.x, xmin);
		ymin = std::min(v.y, xmin);
		zmin = std::min(v.z, xmin);

		xmax = std::max(v.x, xmin);
		ymax = std::max(v.y, xmin);
		zmax = std::max(v.z, xmin);
	}

	float dx = xmax - xmin;
	float dy = ymax - ymin;
	float dz = zmax - zmin;

	ofVec3f V1, V2;

	// Find shortest bbox axis
	if (dx < dy && dx < dz) {
		if (dy > dz) {
			V1 = ofVec3f(0, 1, 0);
			V2 = ofVec3f(0, 0, 1);
		}
		else {
			V2 = ofVec3f(0, 1, 0);
			V1 = ofVec3f(0, 0, 1);
		}
	}
	else if (dy < dx && dy < dz) {
		if (dx > dz) {
			V1 = ofVec3f(1, 0, 0);
			V2 = ofVec3f(0, 0, 1);
		}
		else {
			V2 = ofVec3f(1, 0, 0);
			V1 = ofVec3f(0, 0, 1);
		}
	}
	else  {
		if (dx > dy) {
			V1 = ofVec3f(1, 0, 0);
			V2 = ofVec3f(0, 1, 0);
		}
		else {
			V2 = ofVec3f(1, 0, 0);
			V1 = ofVec3f(0, 1, 0);
		}
	}

	// Project onto shortest bbox axis,
	// and lock extrema vertices

	int vxmin = 0;
	float  umin = 1e5;
	int vxmax = 0;
	float  umax = -1e5;

	for (i = 0; i<mMesh->getNumVertices(); i++) {
		ofVec3f V = mMesh->getVertex(i);
		ofVec3f VV1 = V * V1;
		float u = VV1.x + VV1.y + VV1.z;
		ofVec3f VV2 = V * V2;
		float v = VV2.x + VV2.y + VV2.z;
		mMesh->setTexCoord(i,ofVec2f(u, v));
		if (u < umin) {
			vxmin = i;
			umin = u;
		}
		if (u > umax) {
			vxmax = i;
			umax = u;
		}
	}

	vxminLocked = vxmin;
	vxmaxLocked = vxmax;
}