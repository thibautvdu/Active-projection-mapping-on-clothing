#include "deformation_tracker.h"

#include "of_utilities.h"

namespace garment_augmentation {
namespace garment {

	std::vector<ofVec3f> DeformationTracker::GetPoints() {
		std::vector<ofVec3f> res;

		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();

		int col = roi_.getTopLeft().x + ((roi_.width - 1) / 2);
		int row = roi_.getTopLeft().y + ((roi_.height - 1) / 2);
		/*for (int row = roi_.getTopLeft().y; row < roi_.getBottomRight().y; row++) {
			//for (int col = m_roi.getTopLeft().x; col < m_roi.getBottomRight().x; col++) {
				if (mesh2dView[col][row] != -1) {
					res.push_back(p_garment_->mesh_ref().getVertex(mesh2dView[col][row]));
				}
			//}
		}*/
		if (mesh2dView[col][row] != -1)
			res.push_back(p_garment_->mesh_ref().getVertex(mesh2dView[col][row]));

		return res;
	}

	ofVec3f DeformationTracker::GetCenter() {
		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();
		int col = roi_.getTopLeft().x + ((roi_.width - 1) / 2);
		int row = roi_.getTopLeft().y + ((roi_.height - 1) / 2);

		if (mesh2dView[col][row] != -1)
			return p_garment_->mesh_ref().getVertex(mesh2dView[col][row]);

		return ofVec3f(std::numeric_limits<float>::infinity());
	}

	void DeformationTracker::ShapeOn(const int x, const int y) {
		roi_.setPosition(x, y);
		const std::vector< std::vector<int> > &mesh_2dview = p_garment_->mesh2d_view();
		ofFastMesh &mesh = p_garment_->mesh_ref();

		// Unfolded ideal plane
		ofVec3f top_left = mesh.getVertex(mesh_2dview[roi_.getTopLeft().x][roi_.getTopLeft().y]);
		ofVec3f bottom_left = mesh.getVertex(mesh_2dview[roi_.getBottomLeft().x][roi_.getBottomLeft().y-1]);
		ofVec3f top_right = mesh.getVertex(mesh_2dview[roi_.getTopRight().x-1][roi_.getTopRight().y]);
		ofVec3f bottom_right = mesh.getVertex(mesh_2dview[roi_.getBottomRight().x-1][roi_.getBottomRight().y-1]);
		ofVec3f middle = (top_left + bottom_left + top_right + bottom_right) / 4;

		// Compute the local z-axis
		ofVec3f local_z = (top_right - top_left).getCrossed(top_left - bottom_left).normalize() * -1; // -1 : compensate for the right and left inversion of the kinect

		ofMatrix3x3 to_local_coords = of_utilities::VectorRotationMatrix(local_z, ofVec3f(0, 0, 1));

		ofVec3f point;
		int image_row = roi_.getTopLeft().y;
		int image_col = roi_.getTopLeft().x;
		for (int row = 0; row < roi_.height; row++) {
			for (int col = 0; col < roi_.width; col++) {
				if (mesh_2dview[image_col][image_row] != -1) {
					point = mesh.getVertex(mesh_2dview[image_col][image_row]) - middle; // Relative to the middle of the patch
					patch_depth_map_[col][row] = point.x * to_local_coords.g + point.y * to_local_coords.h + point.z * to_local_coords.i; // Rotation to the normal of the patch
				}
				else {
					patch_depth_map_[col][row] = std::numeric_limits<float>::infinity();
				}
				++image_col;
			}
			++image_row;
		}
	}

	void DeformationTracker::HalfTubeShaping(float depth) {
		const int width = roi_.width;
		const int height = roi_.height;
		const int half_width = (width-1) / 2;
		int middle_x = roi_.getTopLeft().x + ((width - 1) / 2);

		for (int x = 0; x < width; ++x) {
			for (int y = 0; y < height; ++y) {
				patch_depth_map_[x][y] = (1 - abs((x - middle_x) / half_width)) * depth;
			}
		}
	}

	void DeformationTracker::ColorFill(ofColor color) {
		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();

		for (int row = roi_.getTopLeft().y; row < roi_.getBottomRight().y; row++) {
			for (int col = roi_.getTopLeft().x; col < roi_.getBottomRight().x; col++) {
				if (mesh2dView[col][row] != -1) {
					p_garment_->mesh_ref().setColor(mesh2dView[col][row],color);
				}
			}
		}
	}

	void DeformationTracker::ComputeDeltaDepth() {
		const std::vector< std::vector<int> > &mesh_2dview = p_garment_->mesh2d_view();
		ofFastMesh &mesh = p_garment_->mesh_ref();

		// Unfolded ideal plane
		ofVec3f top_left = mesh.getVertex(mesh_2dview[roi_.getTopLeft().x][roi_.getTopLeft().y]);
		ofVec3f bottom_left = mesh.getVertex(mesh_2dview[roi_.getBottomLeft().x][roi_.getBottomLeft().y-1]);
		ofVec3f top_right = mesh.getVertex(mesh_2dview[roi_.getTopRight().x-1][roi_.getTopRight().y]);
		ofVec3f bottom_right = mesh.getVertex(mesh_2dview[roi_.getBottomRight().x-1][roi_.getBottomRight().y-1]);
		ofVec3f middle = (top_left + bottom_left + top_right + bottom_right) / 4;
		/*ofVec3f top_left = ofVec3f(0,5,2);
		ofVec3f bottom_left = ofVec3f(0, 0, 2);
		ofVec3f top_right = ofVec3f(2, 5, 0);
		ofVec3f bottom_right = ofVec3f(2, 0, 0);*/

		// Compute the local z-axis
		ofVec3f local_z = (top_right - top_left).getCrossed(top_left - bottom_left).normalize() * -1; // -1 : compensate for the right and left inversion of the kinect
		int col = roi_.getTopLeft().x + ((roi_.width - 1) / 2);
		int row = roi_.getTopLeft().y + ((roi_.height - 1) / 2);
		
		// Compute the rotation from the world z-axis to the local z-axis
		ofMatrix3x3 to_local_coords = of_utilities::VectorRotationMatrix(local_z,ofVec3f(0, 0, 1));

		/*ofVec3f point(2, 5, 2);
		ofLog() << "transformed x: " << point.x * to_local_coords.a + point.y * to_local_coords.b + point.z * to_local_coords.c;
		ofLog() << "transformed y: " << point.x * to_local_coords.d + point.y * to_local_coords.e + point.z * to_local_coords.f;
		ofLog() << "transformed z: " << point.x * to_local_coords.g + point.y * to_local_coords.h + point.z * to_local_coords.i;*/
		
		// For each known point in the patch
		ofVec3f point;
		delta_depth_ = 0;
		int nb_points = 0;
		int roi_y_max = roi_.getBottomRight().y;
		int roi_x_max = roi_.getBottomRight().x;
		for (int row = roi_.getTopLeft().y; row < roi_y_max; row++) {
			for (int col = roi_.getTopLeft().x; col < roi_x_max; col++) {
				if (mesh_2dview[col][row] != -1) {

					// Transform to local coordinate and get the relative depth
					point = mesh.getVertex(mesh_2dview[col][row]) - middle; // Relative to the middle of the patch
					delta_depth_ += point.x * to_local_coords.g + point.y * to_local_coords.h + point.z * to_local_coords.i; // Rotation to the normal of the patch
					nb_points++;
				}
			}
		}
		delta_depth_ /= nb_points;
	}

	void DeformationTracker::ComputeDissimilarity() {
		const std::vector< std::vector<int> > &mesh_2dview = p_garment_->mesh2d_view();
		ofFastMesh &mesh = p_garment_->mesh_ref();

		// Unfolded ideal plane
		ofVec3f top_left = mesh.getVertex(mesh_2dview[roi_.getTopLeft().x][roi_.getTopLeft().y]);
		ofVec3f bottom_left = mesh.getVertex(mesh_2dview[roi_.getBottomLeft().x ][roi_.getBottomLeft().y - 1]);
		ofVec3f top_right = mesh.getVertex(mesh_2dview[roi_.getTopRight().x-1][roi_.getTopRight().y]);
		ofVec3f bottom_right = mesh.getVertex(mesh_2dview[roi_.getBottomRight().x - 1][roi_.getBottomRight().y - 1]);
		ofVec3f middle = (top_left + bottom_left + top_right + bottom_right) / 4;

		// Compute the local z-axis
		ofVec3f local_z = (top_right - top_left).getCrossed(top_left - bottom_left).normalize() * -1; // -1 : compensate for the right and left inversion of the kinect

		// Compute the rotation from the world z-axis to the local z-axis
		ofMatrix3x3 to_local_coords = of_utilities::VectorRotationMatrix(local_z, ofVec3f(0, 0, 1));

		// For each known point in the patch
		ofVec3f point;
		dissimilarity_ = 0;
		int nb_points = 0;
		int image_row = roi_.getTopLeft().y;
		int image_col = roi_.getTopLeft().x;
		for (int row = 0; row < roi_.height; row++) {
			for (int col = 0; col < roi_.width; col++) {
				if (mesh_2dview[image_col][image_row] != -1 && patch_depth_map_[col][row] != std::numeric_limits<float>::infinity()) {
					// Transform to local coordinate and get the relative depth
					point = mesh.getVertex(mesh_2dview[image_col][image_row]) - middle; // Relative to the middle of the patch
					dissimilarity_ += abs(patch_depth_map_[col][row] - point.x * to_local_coords.g + point.y * to_local_coords.h + point.z * to_local_coords.i); // Rotation to the normal of the patch
					nb_points++;
				}
				image_col++;
			}
			image_row++;
		}
		dissimilarity_ /= nb_points;
	}

	void DeformationTracker::ComputeDeltaDepthGaussian() {
		float m_area = 0;
		float m_unfoldedArea = 0;

		const std::vector< std::vector<int> > &mesh_2dview = p_garment_->mesh2d_view();
		ofFastMesh &mesh = p_garment_->mesh_ref();

		// Unfolded ideal plane
		ofVec3f top_left = mesh.getVertex(mesh_2dview[roi_.getTopLeft().x][roi_.getTopLeft().y]);
		ofVec3f bottom_left = mesh.getVertex(mesh_2dview[roi_.getBottomLeft().x][roi_.getBottomLeft().y - 1]);
		ofVec3f top_right = mesh.getVertex(mesh_2dview[roi_.getTopRight().x-1][roi_.getTopRight().y]);
		ofVec3f bottom_right = mesh.getVertex(mesh_2dview[roi_.getBottomRight().x - 1][roi_.getBottomRight().y - 1]);
		float unfoldedTriangleArea = ((top_left - bottom_left).getCrossed(top_right - bottom_left).length() + (bottom_left - top_right).getCrossed(bottom_right - top_right).length()) / (2 * 2 * (roi_.width - 1)*(roi_.height - 1));

		// Attribute a gaussian weigth along the horizontal of the unfolded patch
		ofVec3f unfoldedHorizontal = ((top_right - top_left) + (bottom_right - bottom_left)) / 2;
		float horizontalScale = unfoldedHorizontal.length() / 2;
		unfoldedHorizontal.normalize();
		ofVec3f unfoldedMiddle = (top_right + top_left + bottom_right + bottom_left) / 4;
		float posToMiddle, gaussianWeight;


		int pointAIdx = -1, pointBIdx, pointCIdx, pointDIdx;
		ofVec3f posA, posB, posC, posD;
		for (int row = roi_.getTopLeft().y; row < roi_.getBottomRight().y; row++) {
			for (int col = roi_.getTopLeft().x; col < roi_.getBottomRight().x; col++) {
				if (mesh_2dview[col][row] != pointAIdx) {

					// The square face
					pointAIdx = mesh_2dview[col][row];
					posA = pointAIdx >= 0 ? mesh.getVertex(pointAIdx) : posA;
					pointBIdx = mesh_2dview[col + 1][row];
					posB = pointBIdx >= 0 ? mesh.getVertex(pointBIdx) : posB;
					pointCIdx = mesh_2dview[col + 1][row + 1];
					posC = pointCIdx >= 0 ? mesh.getVertex(pointCIdx) : posC;
					pointDIdx = mesh_2dview[col][row + 1];
					posD = pointDIdx >= 0 ? mesh.getVertex(pointDIdx) : posD;

					if (pointAIdx >= 0 && pointBIdx >= 0 && pointDIdx >= 0) {
						posToMiddle = (((posA + posB + posC) / 3) - unfoldedMiddle).dot(unfoldedHorizontal) / horizontalScale;
						gaussianWeight = gaussian_values_[99 * std::min(fabs(posToMiddle), 1.f)];

						// Triangle face 1
						m_unfoldedArea += unfoldedTriangleArea * gaussianWeight;
						m_area += (posB - posA).getCrossed(posD - posA).length() * gaussianWeight / 2;
					}
					if (pointBIdx >= 0 && pointCIdx >= 0 && pointDIdx >= 0) {
						posToMiddle = (((posB + posC + posD) / 3) - unfoldedMiddle).dot(unfoldedHorizontal) / horizontalScale;
						gaussianWeight = gaussian_values_[99 * std::min(fabs(posToMiddle), 1.f)];

						// Triangle face 2
						m_unfoldedArea += unfoldedTriangleArea * gaussianWeight;
						m_area += (posC - posB).getCrossed(posD - posB).length() * gaussianWeight / 2;
					}
				}
			}
		}
		delta_depth_gaussian_ = (m_area - m_unfoldedArea) * 100 / m_unfoldedArea;
	}

	bool DeformationTracker::IsInsideMesh() {
		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();

		return mesh2dView[roi_.getTopLeft().x][roi_.getTopLeft().y] != -1 &&
			mesh2dView[roi_.getBottomLeft().x][roi_.getBottomLeft().y-1] != -1 &&
			mesh2dView[roi_.getTopRight().x-1][roi_.getTopRight().y] != -1 &&
			mesh2dView[roi_.getBottomRight().x-1][roi_.getBottomRight().y-1] != -1;
	}

	std::vector<float> DeformationTracker::gaussian_values_;
	void DeformationTracker::ComputeGaussianDist(float sigma) {
		if (gaussian_values_.size() == 0) {
			gaussian_values_.resize(100);

			float posI;
			for (int i = 0; i < gaussian_values_.size(); ++i) {
				posI = 3 * i / (float)99;
				gaussian_values_[i] = (1.0 / (sigma *sqrt(2 * PI))) * exp(-(posI*posI) / (2 * sigma*sigma));
			}
		}
	}

} // namespace garment
} // namespace garment_augmentation