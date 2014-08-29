#ifndef FLEXIBLE_SURFACE_AUGMENTATION_SEMI_IMPLICIT_ACTIVE_MESH_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_SEMI_IMPLICIT_ACTIVE_MESH_H_

#include "ofMesh.h"
#include "ofImage.h"
#include "ofPolyline.h"
#include <Eigen/Dense>

namespace ofDeformationTracking {

	/*
	An object oriented implementation of "3D Non-Rigid Deformable Surface Estimation Without Feature
	Correspondence" by Bryan Willimon, Ian Walker, and Stan Birchfield
	*/
	class ofSemiImplicitActiveMesh : ofMesh {

		public :
			ofSemiImplicitActiveMesh(float dWeight, float bWeight, int meshXRes, int meshYRes) : mDepthWeight(dWeight), mBoundaryWeight(bWeight), 
				mMeshXResolution(meshXRes), mMeshYResolution(meshYRes) {};

			// GETTERS/SETTERS
			void setDepthWeight(float weight) { this->mDepthWeight = weight; }
			float getDepthWeight() const { return this->mDepthWeight; }

			void setBoundaryWeight(float weight) { this->mBoundaryWeight = weight; }
			float getBoundaryWeight() const { return this->mBoundaryWeight; }

			void setMeshXResolution(int res) { this->mMeshXResolution = res; }
			int getMeshXResolution() const { return this->mMeshXResolution; }
			void setMeshYResolution(int res) { this->mMeshXResolution = res; }
			int getMeshYResolution() const { return this->mMeshXResolution; }
			void setGenerationAreaThresh(float thresh) { this->mGenerationAreaThresh = thresh; }
			float getGenerationAreaThresh(float thresh) const { return mGenerationAreaThresh; }

			const ofMesh& getKinectRelativeMeshRef() const { return this->mKinectRelativeMesh; }

			// INITIALIZATION
			void generateMesh(const ofPolyline& imageContour);
			void generateMesh(const ofPolyline& imageContour, const ofImage& contourMask, const ofRectangle roi = ofRectangle(0,0,0,0));
			int intersectionArea(const ofPolyline& contour1, const ofPolyline& contour2);

			// COMPUTATION
			void updateMesh(const ofPolyline& imageContour);

		private :

			// GENERATION PARAMETERS
			int mMeshXResolution;
			int mMeshYResolution;
			float mGenerationAreaThresh;

			// ENERGY MIN. PARAMETERS
			float mDepthWeight;
			float mBoundaryWeight;

			Eigen::MatrixXi mK;

			// ofMesh this ( = image relative mesh + kinect relative depth only)
			ofMesh mPrevious; // Previous mesh for semi implicit scheme
			ofMesh mKinectRelativeMesh;

	};

}

#endif // FLEXIBLE_SURFACE_AUGMENTATION_SEMI_IMPLICIT_ACTIVE_MESH_H_