#include "foldTracker.h"

namespace garmentAugmentation {
namespace garment {

	std::vector<ofVec3f> foldTracker::getPoints() {
		std::vector<ofVec3f> res;

		const std::vector< std::vector<int> > &mesh2dView = pGarment_->getMesh2dViewRef();

		int col = m_roi.getTopLeft().x + ((m_roi.width - 1) / 2);
		for (int row = m_roi.getTopLeft().y; row < m_roi.getBottomRight().y; row++) {
			//for (int col = m_roi.getTopLeft().x; col < m_roi.getBottomRight().x; col++) {
				if (mesh2dView[col][row] != -1) {
					res.push_back(pGarment_->getMeshRef().getVertex(mesh2dView[col][row]));
				}
			//}
		}

		return res;
	}

	void foldTracker::setColor(ofColor color) {
		const std::vector< std::vector<int> > &mesh2dView = pGarment_->getMesh2dViewRef();

		int col = m_roi.getTopLeft().x + ((m_roi.width - 1) / 2);
		for (int row = m_roi.getTopLeft().y; row < m_roi.getBottomRight().y; row++) {
			//for (int col = m_roi.getTopLeft().x; col < m_roi.getBottomRight().x; col++) {
				if (mesh2dView[col][row] != -1) {
					pGarment_->getMeshRef().setColor(mesh2dView[col][row],color);
				}
			//}
		}
	}

	void foldTracker::computeAreas() {
		m_area = 0;
		m_unfoldedArea = 0;

		const std::vector< std::vector<int> > &mesh2dView = pGarment_->getMesh2dViewRef();
		ofFastMesh &mesh = pGarment_->getMeshRef();

		// Unfolded ideal plane
		ofVec3f topLeft = mesh.getVertex(mesh2dView[m_roi.getTopLeft().x][m_roi.getTopLeft().y]);
		ofVec3f bottomLeft = mesh.getVertex(mesh2dView[m_roi.getBottomLeft().x][m_roi.getBottomLeft().y]);
		ofVec3f topRight = mesh.getVertex(mesh2dView[m_roi.getTopRight().x][m_roi.getTopRight().y]);
		ofVec3f bottomRight = mesh.getVertex(mesh2dView[m_roi.getBottomRight().x][m_roi.getBottomRight().y]);
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
				if (mesh2dView[col][row] != pointAIdx) {

					// The square face
					pointAIdx = mesh2dView[col][row];
					posA = pointAIdx >= 0 ? mesh.getVertex(pointAIdx) : posA;
					pointBIdx = mesh2dView[col+1][row];
					posB = pointBIdx >= 0 ? mesh.getVertex(pointBIdx) : posB;
					pointCIdx = mesh2dView[col + 1][row+1];
					posC = pointCIdx >= 0 ? mesh.getVertex(pointCIdx) : posC;
					pointDIdx = mesh2dView[col ][row + 1];
					posD = pointDIdx >= 0 ? mesh.getVertex(pointDIdx) : posD;

					if (pointAIdx >= 0 && pointBIdx >= 0 && pointDIdx >= 0) {
						posToMiddle = (((posA + posB + posC) / 3) - unfoldedMiddle).dot(unfoldedHorizontal) / horizontalScale;
						gaussianWeight = gaussianValues_[99 * std::min(fabs(posToMiddle), 1.f)];

						// Triangle face 1
						m_unfoldedArea += unfoldedTriangleArea * gaussianWeight;
						m_area += (posB - posA).getCrossed(posD - posA).length() * gaussianWeight / 2;
					}
					if (pointBIdx >= 0 && pointCIdx >= 0 && pointDIdx >= 0) {
						posToMiddle = (((posB + posC + posD) / 3) - unfoldedMiddle).dot(unfoldedHorizontal) / horizontalScale;
						gaussianWeight = gaussianValues_[99 * std::min(fabs(posToMiddle), 1.f)];

						// Triangle face 2
						m_unfoldedArea += unfoldedTriangleArea * gaussianWeight;
						m_area += (posC - posB).getCrossed(posD - posB).length() * gaussianWeight / 2;
					}
				}
			}
		}
	}

	std::vector<float> foldTracker::gaussianValues_;
	void foldTracker::computeGaussianDist(float sigma) {
		if (gaussianValues_.size() == 0)
			gaussianValues_.resize(100);

		float posI;
		for (int i = 0; i < gaussianValues_.size(); ++i) {
			posI = 3 * i / (float)99;
			gaussianValues_[i] = (1.0 / (sigma *sqrt(2 * PI))) * exp(-(posI*posI) / (2 * sigma*sigma));
		}
	}

	bool foldTracker::insideMesh() {
		const std::vector< std::vector<int> > &mesh2dView = pGarment_->getMesh2dViewRef();

		return mesh2dView[m_roi.getTopLeft().x][m_roi.getTopLeft().y] != -1 &&
			mesh2dView[m_roi.getBottomLeft().x][m_roi.getBottomLeft().y] != -1 &&
			mesh2dView[m_roi.getTopRight().x][m_roi.getTopRight().y] != -1 &&
			mesh2dView[m_roi.getBottomRight().x][m_roi.getBottomRight().y] != -1;
	}

} // namespace garment
} // namespace garmentAugmentation