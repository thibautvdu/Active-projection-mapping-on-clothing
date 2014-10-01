#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_

#include "ofGraphics.h"
#include "ofFastMesh.h"

class foldTracker {

	public :

		foldTracker(ofMesh *mesh, unsigned int *indicesMap, int width, int height) : mp_mesh(mesh), m_2dIndicesMap(indicesMap),
			m_mapW(width), m_mapH(height) {};

		unsigned int *get2dIndicesMap() const { return m_2dIndicesMap; }
		inline int getIndicesMapWidth() const { return m_mapW; }
		inline int getIndicesMapHeight() const { return m_mapH; }
		ofMesh *getMeshPtr() const { return mp_mesh; }

		class trackerPatch {
			public:
				trackerPatch(foldTracker *t, ofRectangle roi);

				inline void move(int x, int y) {
					m_roi.translate(x, y);
					m_area = m_unfoldedArea = -1;
				}

				inline void moveTo(ofRectangle roi) {
					m_roi = roi;
					m_area = m_unfoldedArea = -1;
				}

				inline float getArea() {
					if (m_area == -1)
						computeAreas();

					return m_area;
				}

				inline float getUnfoldedArea() {
					if (m_unfoldedArea == -1)
						computeAreas();

					return m_unfoldedArea;
				}

				inline float getDeformationPercent() {
					return (getArea() - getUnfoldedArea()) * 100 / getUnfoldedArea();
				}

				int getMeshIndex(int x, int y);

				std::vector<ofVec3f> getPoints();
				void setColor(ofColor color);


				bool insideMesh();

			private:
				const foldTracker *mp_tracker;
				ofMesh *const mp_mesh;
				ofRectangle m_roi;
				float m_area;
				float m_unfoldedArea;

				void computeAreas();

				// ( cover 99.7 % of the gaussian distribution )
				void computeGaussianDist(float sigma);
		};

		inline trackerPatch createPatch(ofRectangle roi) { return trackerPatch(this, roi); }

	private:
		ofMesh *mp_mesh;
		unsigned int *m_2dIndicesMap;
		int m_mapW, m_mapH;
};

#endif