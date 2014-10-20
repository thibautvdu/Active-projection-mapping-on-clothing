#include "fold_tracker.h"

#include "of_utilities.h"

namespace garment_augmentation {
namespace garment {

	std::vector<ofVec3f> FoldTracker::GetPoints() {
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

	void FoldTracker::ColorFill(ofColor color) {
		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();

		int col = roi_.getTopLeft().x + ((roi_.width - 1) / 2);
		for (int row = roi_.getTopLeft().y; row < roi_.getBottomRight().y; row++) {
			for (int col = roi_.getTopLeft().x; col < roi_.getBottomRight().x; col++) {
				if (mesh2dView[col][row] != -1) {
					p_garment_->mesh_ref().setColor(mesh2dView[col][row],color);
				}
			}
		}
	}

	void FoldTracker::ComputeDeltaDepth() {
		const std::vector< std::vector<int> > &mesh_2dview = p_garment_->mesh2d_view();
		ofFastMesh &mesh = p_garment_->mesh_ref();

		// Unfolded ideal plane
		ofVec3f top_left = mesh.getVertex(mesh_2dview[roi_.getTopLeft().x][roi_.getTopLeft().y]);
		ofVec3f bottom_left = mesh.getVertex(mesh_2dview[roi_.getBottomLeft().x][roi_.getBottomLeft().y]);
		ofVec3f top_right = mesh.getVertex(mesh_2dview[roi_.getTopRight().x][roi_.getTopRight().y]);
		ofVec3f bottom_right = mesh.getVertex(mesh_2dview[roi_.getBottomRight().x][roi_.getBottomRight().y]);
		ofVec3f middle = (top_left + bottom_left + top_right + bottom_right) / 4;
		/*ofVec3f top_left = ofVec3f(0,5,2);
		ofVec3f bottom_left = ofVec3f(0, 0, 2);
		ofVec3f top_right = ofVec3f(2, 5, 0);
		ofVec3f bottom_right = ofVec3f(2, 0, 0);*/

		// Compute the local z-axis
		ofVec3f local_z = (top_right - top_left).getCrossed(top_left - bottom_left).normalize() * -1; // -1 : compensate for the right and left inversion of the kinect
		//ofLog() << "local z :" << local_z;
		
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
		for (int row = roi_.getTopLeft().y; row < roi_.getBottomRight().y; row++) {
			for (int col = roi_.getTopLeft().x; col < roi_.getBottomRight().x; col++) {
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

	bool FoldTracker::IsInsideMesh() {
		const std::vector< std::vector<int> > &mesh2dView = p_garment_->mesh2d_view();

		return mesh2dView[roi_.getTopLeft().x][roi_.getTopLeft().y] != -1 &&
			mesh2dView[roi_.getBottomLeft().x][roi_.getBottomLeft().y] != -1 &&
			mesh2dView[roi_.getTopRight().x][roi_.getTopRight().y] != -1 &&
			mesh2dView[roi_.getBottomRight().x][roi_.getBottomRight().y] != -1;
	}

} // namespace garment
} // namespace garment_augmentation