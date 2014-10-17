#include "fold_tracker.h"

namespace garment_augmentation {
namespace garment {

	std::vector<ofVec3f> FoldTracker::GetPoints() {
		std::vector<ofVec3f> res;

		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();

		int col = roi_.getTopLeft().x + ((roi_.width - 1) / 2);
		for (int row = roi_.getTopLeft().y; row < roi_.getBottomRight().y; row++) {
			//for (int col = m_roi.getTopLeft().x; col < m_roi.getBottomRight().x; col++) {
				if (mesh2dView[col][row] != -1) {
					res.push_back(p_garment_->mesh_ref().getVertex(mesh2dView[col][row]));
				}
			//}
		}

		return res;
	}

	void FoldTracker::ColorFill(ofColor color) {
		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();

		int col = roi_.getTopLeft().x + ((roi_.width - 1) / 2);
		for (int row = roi_.getTopLeft().y; row < roi_.getBottomRight().y; row++) {
			//for (int col = m_roi.getTopLeft().x; col < m_roi.getBottomRight().x; col++) {
				if (mesh2dView[col][row] != -1) {
					p_garment_->mesh_ref().setColor(mesh2dView[col][row],color);
				}
			//}
		}
	}

	void FoldTracker::ComputeAreas() {
		area_ = 0;
		unfolded_area_ = 0;

		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();
		ofFastMesh &mesh = p_garment_->mesh_ref();

		// Unfolded ideal plane
		ofVec3f topLeft = mesh.getVertex(mesh2dView[roi_.getTopLeft().x][roi_.getTopLeft().y]);
		ofVec3f bottomLeft = mesh.getVertex(mesh2dView[roi_.getBottomLeft().x][roi_.getBottomLeft().y]);
		ofVec3f topRight = mesh.getVertex(mesh2dView[roi_.getTopRight().x][roi_.getTopRight().y]);
		ofVec3f bottomRight = mesh.getVertex(mesh2dView[roi_.getBottomRight().x][roi_.getBottomRight().y]);
		float unfoldedTriangleArea = ((topLeft - bottomLeft).getCrossed(topRight - bottomLeft).length() + (bottomLeft - topRight).getCrossed(bottomRight - topRight).length()) / (2 * 2 * (roi_.width - 1)*(roi_.height - 1));

		// Attribute a gaussian weigth along the horizontal of the unfolded patch
		ofVec3f unfoldedHorizontal = ((topRight - topLeft) + (bottomRight - bottomLeft)) / 2;
		float horizontalScale = unfoldedHorizontal.length() / 2;
		unfoldedHorizontal.normalize();
		ofVec3f unfoldedMiddle = (topRight + topLeft + bottomRight + bottomLeft) / 4;
		float posToMiddle, gaussianWeight;


		int pointAIdx = -1, pointBIdx, pointCIdx, pointDIdx;
		ofVec3f posA, posB, posC, posD;
		for (int row = roi_.getTopLeft().y; row < roi_.getBottomRight().y; row++) {
			for (int col = roi_.getTopLeft().x; col < roi_.getBottomRight().x; col++) {
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
						gaussianWeight = gaussian_values_[99 * std::min(fabs(posToMiddle), 1.f)];

						// Triangle face 1
						unfolded_area_ += unfoldedTriangleArea * gaussianWeight;
						area_ += (posB - posA).getCrossed(posD - posA).length() * gaussianWeight / 2;
					}
					if (pointBIdx >= 0 && pointCIdx >= 0 && pointDIdx >= 0) {
						posToMiddle = (((posB + posC + posD) / 3) - unfoldedMiddle).dot(unfoldedHorizontal) / horizontalScale;
						gaussianWeight = gaussian_values_[99 * std::min(fabs(posToMiddle), 1.f)];

						// Triangle face 2
						unfolded_area_ += unfoldedTriangleArea * gaussianWeight;
						area_ += (posC - posB).getCrossed(posD - posB).length() * gaussianWeight / 2;
					}
				}
			}
		}
	}

	std::vector<float> FoldTracker::gaussian_values_;
	void FoldTracker::ComputeGaussianDistribution(float sigma) {
		if (gaussian_values_.size() == 0)
			gaussian_values_.resize(100);

		float posI;
		for (int i = 0; i < gaussian_values_.size(); ++i) {
			posI = 3 * i / (float)99;
			gaussian_values_[i] = (1.0 / (sigma *sqrt(2 * PI))) * exp(-(posI*posI) / (2 * sigma*sigma));
		}
	}

	bool FoldTracker::IsInsideMesh() {
		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();

		return mesh2dView[roi_.getTopLeft().x][roi_.getTopLeft().y] != -1 &&
			mesh2dView[roi_.getBottomLeft().x][roi_.getBottomLeft().y] != -1 &&
			mesh2dView[roi_.getTopRight().x][roi_.getTopRight().y] != -1 &&
			mesh2dView[roi_.getBottomRight().x][roi_.getBottomRight().y] != -1;
	}

} // namespace garment
} // namespace garment_augmentation