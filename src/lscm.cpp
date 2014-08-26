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

void LSCM::setup_lscm(const Vector3& v0, const ofIndexType i0, const Vector3& v1, const ofIndexType i1, const Vector3& v2, const ofIndexType i2) {
	// We ensure that the triangle vertices are clockwise ordered in the xy plane
		setup_conformal_map_relations(v0, i0, v1, i1, v2, i2);
}

// Computes the coordinates of the vertices of a triangle
// in a local 2D orthonormal basis of the triangle's plane.
void LSCM::project_triangle(
	const Vector3& p0,
	const Vector3& p1,
	const Vector3& p2,
	Vector2& z0,
	Vector2& z1,
	Vector2& z2
	) {
	Vector3 X = p1 - p0;
	X.normalize();
	Vector3 Z = X ^ (p2 - p0);
	Z.normalize();
	Vector3 Y = Z ^ X;
	const Vector3& O = p0;

	double x0 = 0;
	double y0 = 0;
	double x1 = (p1 - O).length();
	double y1 = 0;
	double x2 = (p2 - O) * X;
	double y2 = (p2 - O) * Y;

	z0 = Vector2(x0, y0);
	z1 = Vector2(x1, y1);
	z2 = Vector2(x2, y2);
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
	const Vector3& p0, const ofIndexType id0, const Vector3& p1, const ofIndexType id1, const Vector3& p2, const ofIndexType id2
	) {

	Vector2 z0, z1, z2;
	project_triangle(p0, p1, p2, z0, z1, z2);
	Vector2 z01 = z1 - z0;
	Vector2 z02 = z2 - z0;
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
	umax = -1e30;
	umin = 1e30;
	vmax = -1e30;
	vmin = 1e30;

	for (unsigned int i = 0; i<mMesh->getNumVertices(); i++) {
		double u = nlGetVariable(2 * i);
		double v = nlGetVariable(2 * i + 1);
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
		double u = mMesh->getTexCoord(i).x;
		double v = mMesh->getTexCoord(i).y;
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

	double xmin = 1e30;
	double ymin = 1e30;
	double zmin = 1e30;
	double xmax = -1e30;
	double ymax = -1e30;
	double zmax = -1e30;

	for (i = 0; i<mMesh->getNumVertices(); i++) {
		const Vector3& v = mMesh->getVertex(i);
		xmin = std::min(v.x, xmin);
		ymin = std::min(v.y, ymin);
		zmin = std::min(v.z, zmin);

		xmax = std::max(v.x, xmax);
		ymax = std::max(v.y, ymax);
		zmax = std::max(v.z, zmax);
	}

	double dx = xmax - xmin;
	double dy = ymax - ymin;
	double dz = zmax - zmin;

	Vector3 V1, V2;

	// Find shortest bbox axis
	if (dx < dy && dx < dz) {
		if (dy > dz) {
			V1 = Vector3(0, 1, 0);
			V2 = Vector3(0, 0, 1);
		}
		else {
			V2 = Vector3(0, 1, 0);
			V1 = Vector3(0, 0, 1);
		}
	}
	else if (dy < dx && dy < dz) {
		if (dx > dz) {
			V1 = Vector3(1, 0, 0);
			V2 = Vector3(0, 0, 1);
		}
		else {
			V2 = Vector3(1, 0, 0);
			V1 = Vector3(0, 0, 1);
		}
	}
	else if (dz < dx && dz < dy) {
		if (dx > dy) {
			V1 = Vector3(1, 0, 0);
			V2 = Vector3(0, 1, 0);
		}
		else {
			V2 = Vector3(1, 0, 0);
			V1 = Vector3(0, 1, 0);
		}
	}

	// Project onto shortest bbox axis,
	// and lock extrema vertices

	double  umin = 1e30;
	double  umax = -1e30;

	for (i = 0; i<mMesh->getNumVertices(); i++) {
		const Vector3& V = mMesh->getVertex(i);
		double u = V * V1;
		double v = V * V2;
		mMesh->setTexCoord(i,ofVec2f(u, v));
		if (u < umin) {
			vxminLocked = i;
			umin = u;
		}
		if (u > umax) {
			vxmaxLocked = i;
			umax = u;
		}
	}
}