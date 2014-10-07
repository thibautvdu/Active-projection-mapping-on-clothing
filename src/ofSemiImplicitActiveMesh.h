#ifndef FLEXIBLE_SURFACE_AUGMENTATION_SEMI_IMPLICIT_ACTIVE_MESH_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_SEMI_IMPLICIT_ACTIVE_MESH_H_

#include "ofMesh.h"
#include "ofImage.h"
#include "ofPolyline.h"
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include "ofxKinectCommonBridge.h"
#include "kinect3dBlobDetector.h"

namespace garmentAugmentation {

	/*
	An object oriented implementation of "3D Non-Rigid Deformable Surface Estimation Without Feature
	Correspondence" by Bryan Willimon, Ian Walker, and Stan Birchfield
	*/
	class ofSemiImplicitActiveMesh : public ofMesh {

		public :
			ofSemiImplicitActiveMesh() : ofMesh() {
				mNeedComputation = false;

				mMeshXResolution = mMeshYResolution = 10;

				mGenerated = false;
				mGenerationAreaThresh = 20;

				mAdaptationRate = 2;
				mBoundaryWeight = 0.6;
				mDepthWeight = 0.8;

				m_minDepth = m_maxDepth = 0;

				mpSolver = new Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>>();
			}

			ofSemiImplicitActiveMesh(int meshXRes, int meshYRes) : mMeshXResolution(meshXRes), mMeshYResolution(meshYRes) {
				mNeedComputation = false;

				mGenerated = false;
				mGenerationAreaThresh = 20;

				mAdaptationRate = 2;
				mBoundaryWeight = 0.6;
				mDepthWeight = 0.8;

				m_minDepth = m_maxDepth = 0;

				mpSolver = new Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>>();
			};

			// GETTERS/SETTERS
			void setDepthWeight(float weight) { mDepthWeight = weight; }
			float getDepthWeight() const { return mDepthWeight; }

			void setBoundaryWeight(float weight) { mBoundaryWeight = weight; }
			float getBoundaryWeight() const { return mBoundaryWeight; }

			void setAdaptationRate(float rate) {
				if (abs(rate - mAdaptationRate) > 0.01) {
					mAdaptationRate = rate;
					mNeedComputation = true;
				}
			}
			float getAdaptationRate() const { return mAdaptationRate; }

			void setMeshXResolution(int res) { mMeshXResolution = res; }
			int getMeshXResolution() const { return mMeshXResolution; }
			void setMeshYResolution(int res) { mMeshXResolution = res; }
			int getMeshYResolution() const { return mMeshXResolution; }
			void setGenerationAreaThresh(float thresh) { mGenerationAreaThresh = thresh; }
			float getGenerationAreaThresh(float thresh) const { return mGenerationAreaThresh; }

			ofMesh& getWorldMeshRef() { return mKinectRelativeMesh; }

			// FLAGS
			bool isGenerated() { return mGenerated; }

			const ofMesh& getKinectRelativeMeshRef() const { return mKinectRelativeMesh; }

			// INITIALIZATION
			void generateMesh(const kinect3dBlobDetector &detector, simple3dBlob &blob);

			// COMPUTATION
			void updateMesh(const kinect3dBlobDetector &detector, simple3dBlob &blob);

		private :

			// GENERATION PARAMETERS
			int mMeshXResolution;
			int mMeshYResolution;
			float mGenerationAreaThresh;
			vector<bool> mIsBoundaryVertices; // index -> is boundary vertice
			vector<int> mMeshBoundaryIndices;

			// ENERGY MIN. PARAMETERS
			float mDepthWeight;
			float mBoundaryWeight;
			float mAdaptationRate;

			Eigen::SparseMatrix<double> mK;
			Eigen::SparseMatrix<double> mA;
			Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>>* mpSolver;

			// ofMesh this ( = image relative mesh + kinect relative depth only)
			ofMesh mKinectRelativeMesh;

			// VARIOUS
			float m_meshAvgDepth; // Used to discriminate incoherent values sent by kinect
			float m_maxDepth, m_minDepth;

			// FLAGS
			bool mGenerated;
			bool mNeedComputation;

			// COMPUTATION METHODS
			void computeSolver();
			int intersectionArea(const ofPolyline& contour1, const ofPolyline& contour2);
	};

}

#endif // FLEXIBLE_SURFACE_AUGMENTATION_SEMI_IMPLICIT_ACTIVE_MESH_H_