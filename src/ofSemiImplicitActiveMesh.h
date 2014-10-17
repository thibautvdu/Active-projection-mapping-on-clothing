#ifndef FLEXIBLE_SURFACE_AUGMENTATION_SEMI_IMPLICIT_ACTIVE_MESH_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_SEMI_IMPLICIT_ACTIVE_MESH_H_

#include "ofMesh.h"
#include "ofImage.h"
#include "ofPolyline.h"
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "ofxKinectCommonBridge.h"
#include "kinect3dBlobDetector.h"

namespace garment_augmentation {
namespace surface_tracking {

	/*
	An object oriented implementation of "3D Non-Rigid Deformable Surface Estimation Without Feature
	Correspondence" by Bryan Willimon, Ian Walker, and Stan Birchfield
	*/
	class ofSemiImplicitActiveMesh : public ofMesh {

		public :
			inline ofSemiImplicitActiveMesh() : ofMesh() {
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

			inline ofSemiImplicitActiveMesh(const int meshXRes, const int meshYRes) : mMeshXResolution(meshXRes), mMeshYResolution(meshYRes) {
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
			inline void setDepthWeight(const float weight) { 
				mDepthWeight = weight;
			}

			inline float getDepthWeight() const {
				return mDepthWeight; 
			}

			inline void setBoundaryWeight(const float weight) {
				mBoundaryWeight = weight; 
			}

			inline float getBoundaryWeight() const {
				return mBoundaryWeight;
			}

			inline void setAdaptationRate(const float rate) {
				if (abs(rate - mAdaptationRate) > 0.01) {
					mAdaptationRate = rate;
					mNeedComputation = true;
				}
			}

			inline float getAdaptationRate() const {
				return mAdaptationRate; 
			}

			inline void setMeshXResolution(const int res) {
				mMeshXResolution = res; 
			}

			inline int getMeshXResolution() const {
				return mMeshXResolution; 
			}

			inline void setMeshYResolution(const int res) {
				mMeshXResolution = res; 
			}
			
			inline int getMeshYResolution() const {
				return mMeshXResolution; 
			}

			inline void setGenerationAreaThresh(const float thresh) {
				mGenerationAreaThresh = thresh; 
			}

			inline float getGenerationAreaThresh(const float thresh) const {
				return mGenerationAreaThresh; 
			}

			// FLAGS
			inline bool isGenerated() const {
				return mGenerated; 
			}

			inline const ofMesh& getKinectRelativeMeshRef() const {
				return mKinectRelativeMesh; 
			}

			// INITIALIZATION
			void generateMesh(const blob_detection::kinect3dBlobDetector &detector, const Simple3dblob &blob);

			// COMPUTATION
			void updateMesh(const blob_detection::kinect3dBlobDetector &detector, const Simple3dblob &blob);

		private :
			// COMPUTATION METHODS
			void computeSolver();
			int intersectionArea(const ofPolyline& contour1, const ofPolyline& contour2);

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
	};

} // namespace surface_tracking
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_SEMI_IMPLICIT_ACTIVE_MESH_H_