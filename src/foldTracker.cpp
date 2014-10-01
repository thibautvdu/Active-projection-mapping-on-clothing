#include "foldTracker.h"

// ***************************************************************************
//								STATIC
// ***************************************************************************

static std::vector<float> gaussianValues;

// ***************************************************************************
//								CONSTRUCTOR
// ***************************************************************************

foldTracker::trackerPatch::trackerPatch(foldTracker *t, ofRectangle roi) : mp_tracker(t), mp_mesh(mp_tracker->getMeshPtr()), m_roi(roi) {
	m_area = m_unfoldedArea = -1;
	if (gaussianValues.size() == 0)
		computeGaussianDist(1);
}

// ***************************************************************************
//								GET/SET
// ***************************************************************************

std::vector<ofVec3f> foldTracker::trackerPatch::getPoints() {
	std::vector<ofVec3f> res;
	unsigned int *idxMap = mp_tracker->get2dIndicesMap();

	for (int row = m_roi.getTopLeft().y; row < m_roi.getBottomRight().y; row++) {
		for (int col = m_roi.getTopLeft().x; col < m_roi.getBottomRight().x; col++) {
			if (*(idxMap + row*mp_tracker->getIndicesMapWidth() + col) >= 0) {
				if (mp_mesh->getNumVertices() > *(idxMap + row*mp_tracker->getIndicesMapWidth() + col))
					res.push_back(mp_mesh->getVertex(*(idxMap + row*mp_tracker->getIndicesMapWidth() + col)));
			}
		}
	}

	return res;
}

int foldTracker::trackerPatch::getMeshIndex(int x, int y) {
	unsigned int *idxMap = mp_tracker->get2dIndicesMap();
	x += m_roi.getTopLeft().x;
	y += m_roi.getTopLeft().y;

	int res = *(idxMap + y*mp_tracker->getIndicesMapWidth() + x);
	if (res >= 0)
		return res;

	return -1;
}

void foldTracker::trackerPatch::setColor(ofColor color) {
	unsigned int *idxMap = mp_tracker->get2dIndicesMap();

	for (int row = m_roi.getTopLeft().y; row < m_roi.getBottomRight().y; row++) {
		for (int col = m_roi.getTopLeft().x; col < m_roi.getBottomRight().x; col++) {
			if (*(idxMap + row*mp_tracker->getIndicesMapWidth() + col) >= 0) {
				if (mp_mesh->getNumColors() > *(idxMap + row*mp_tracker->getIndicesMapWidth() + col))
					mp_mesh->setColor(*(idxMap + row*mp_tracker->getIndicesMapWidth() + col), color);
			}
		}
	}
}

// ***************************************************************************
//								COMPUTE
// ***************************************************************************

void foldTracker::trackerPatch::computeAreas() {
	m_area = 0;
	m_unfoldedArea = 0;

	unsigned int *idxMap = mp_tracker->get2dIndicesMap();

	// Unfolded ideal plane
	ofVec3f topLeft = mp_mesh->getVertex(*(idxMap + (int)m_roi.getTopLeft().y*mp_tracker->getIndicesMapWidth() + (int)m_roi.getTopLeft().x));
	ofVec3f bottomLeft = mp_mesh->getVertex(*(idxMap + (int)m_roi.getBottomLeft().y*mp_tracker->getIndicesMapWidth() + (int)m_roi.getBottomLeft().x));
	ofVec3f topRight = mp_mesh->getVertex(*(idxMap + (int)m_roi.getTopRight().y*mp_tracker->getIndicesMapWidth() + (int)m_roi.getTopRight().x));
	ofVec3f bottomRight = mp_mesh->getVertex(*(idxMap + (int)m_roi.getBottomRight().y*mp_tracker->getIndicesMapWidth() + (int)m_roi.getBottomRight().x));
	float unfoldedTriangleArea = ((topLeft - bottomLeft).getCrossed(topRight - bottomLeft).length() + (bottomLeft - topRight).getCrossed(bottomRight - topRight).length()) / (2 * 2 * (m_roi.width - 1)*(m_roi.height - 1));

	// Attribute a gaussian weigth along the horizontal of the unfolded patch
	ofVec3f unfoldedHorizontal = ((topRight - topLeft) + (bottomRight - bottomLeft)) / 2;
	float horizontalScale = unfoldedHorizontal.length() / 2;
	unfoldedHorizontal.normalize();
	ofVec3f unfoldedMiddle = (topRight + topLeft + bottomRight + bottomLeft) / 4;
	float posToMiddle, gaussianWeight;


	int pointAIdx = -1, pointBIdx, pointCIdx, pointDIdx;
	ofVec3f posA, posB, posC, posD;
	for (int row = m_roi.getTopLeft().y; row < m_roi.getBottomRight().y; row++) {
		for (int col = m_roi.getTopLeft().x; col < m_roi.getBottomRight().x; col++) {
			if (*(idxMap + row*mp_tracker->getIndicesMapWidth() + col) != pointAIdx) {

				// The square face
				pointAIdx = *(idxMap + row*mp_tracker->getIndicesMapWidth() + col);
				posA = pointAIdx >= 0 ? mp_mesh->getVertex(pointAIdx) : posA;
				pointBIdx = *(idxMap + row*mp_tracker->getIndicesMapWidth() + col + 1);
				posB = pointBIdx >= 0 ? mp_mesh->getVertex(pointBIdx) : posB;
				pointCIdx = *(idxMap + (row + 1)*mp_tracker->getIndicesMapWidth() + col + 1);
				posC = pointCIdx >= 0 ? mp_mesh->getVertex(pointCIdx) : posC;
				pointDIdx = *(idxMap + (row + 1)*mp_tracker->getIndicesMapWidth() + col);
				posD = pointDIdx >= 0 ? mp_mesh->getVertex(pointDIdx) : posD;

				if (pointAIdx >= 0 && pointBIdx >= 0 && pointDIdx >= 0) {
					posToMiddle = (((posA + posB + posC) / 3) - unfoldedMiddle).dot(unfoldedHorizontal) / horizontalScale;
					gaussianWeight = gaussianValues[99 * std::min(fabs(posToMiddle), 1.f)];

					// Triangle face 1
					m_unfoldedArea += unfoldedTriangleArea * gaussianWeight;
					m_area += (posB - posA).getCrossed(posD - posA).length() * gaussianWeight / 2;
				}
				if (pointBIdx >= 0 && pointCIdx >= 0 && pointDIdx >= 0) {
					posToMiddle = (((posB + posC + posD) / 3) - unfoldedMiddle).dot(unfoldedHorizontal) / horizontalScale;
					gaussianWeight = gaussianValues[99 * std::min(fabs(posToMiddle), 1.f)];

					// Triangle face 2
					m_unfoldedArea += unfoldedTriangleArea * gaussianWeight;
					m_area += (posC - posB).getCrossed(posD - posB).length() * gaussianWeight / 2;
				}
			}
		}
	}
}

void foldTracker::trackerPatch::computeGaussianDist(float sigma) {
	if (gaussianValues.size() == 0)
		gaussianValues.resize(100);

	float posI;
	for (int i = 0; i < gaussianValues.size(); ++i) {
		posI = 3 * i / (float)99;
		gaussianValues[i] = (1.0 / (sigma *sqrt(2 * PI))) * exp(-(posI*posI) / (2 * sigma*sigma));
	}
}

// ***************************************************************************
//								OTHERS
// ***************************************************************************

bool foldTracker::trackerPatch::insideMesh() {
	unsigned int *idxMap = mp_tracker->get2dIndicesMap();

	int topLeft = *(idxMap + (int)m_roi.getTopLeft().y*mp_tracker->getIndicesMapWidth() + (int)m_roi.getTopLeft().x);
	int bottomLeft = *(idxMap + (int)m_roi.getBottomLeft().y*mp_tracker->getIndicesMapWidth() + (int)m_roi.getBottomLeft().x);
	int topRight = *(idxMap + (int)m_roi.getTopRight().y*mp_tracker->getIndicesMapWidth() + (int)m_roi.getTopRight().x);
	int bottomRight = *(idxMap + (int)m_roi.getBottomRight().y*mp_tracker->getIndicesMapWidth() + (int)m_roi.getBottomRight().x);

	return (topLeft >= 0 && bottomLeft >= 0 && topRight >= 0 && bottomRight >= 0);
}