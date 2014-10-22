#include "ransac_lines.h"

namespace garment_augmentation {
namespace math {

using mrpt::math::TPoint3D;
using mrpt::math::CMatrixTemplateNumeric;

	void Ransac3DsegmentFit(const CMatrixDouble  &all_data, const vector_size_t  &use_indices, std::vector< CMatrixDouble > &fit_models) {
		ASSERT_(use_indices.size() == 2);

		try
		{
			fit_models.resize(1);
			CMatrixDouble &M = fit_models[0];

			M.setSize(2, 3);
			for (size_t i = 0; i < 3; i++) {
				M(0, i) = all_data(i, use_indices[0]);
				M(1, i) = all_data(i, use_indices[1]);
			}
		}
		catch (std::exception &)
		{
			fit_models.clear();
			return;
		}
	}

	void Ransac3DsegmentDistance(const CMatrixDouble &all_data, const std::vector< CMatrixDouble > & test_models, const double distance_threshold,
		unsigned int & out_bestModelIndex, vector_size_t & out_inlierIndices) {
		ASSERT_(test_models.size() == 1)
			out_bestModelIndex = 0;
		const CMatrixDouble &M = test_models[0];

		ASSERT_(size(M, 1) == 2 && size(M, 2) == 3);

		TSegment3D segment;
		segment.point1 = TPoint3D(M(0, 0), M(0, 1), M(0, 2));
		segment.point2 = TPoint3D(M(1, 0), M(1, 1), M(1, 2));
		TLine3D line(segment);

		const size_t N = size(all_data, 2);
		out_inlierIndices.clear();
		out_inlierIndices.reserve(100);
		for (size_t i = 0; i<N; i++)
		{
			const double d = line.distance(TPoint3D(all_data.get_unsafe(0, i), all_data.get_unsafe(1, i), all_data.get_unsafe(2, i)));
			if (d<distance_threshold)
				out_inlierIndices.push_back(i);
		}
	}

	void RansacDetect3Dsegments(const std::vector<ofVec3f> &point_cloud, const double threshold, const size_t min_inliers_for_valid_line, 
		std::vector<std::pair<size_t, Of3dsegment> > &out_detected_segments) {

		out_detected_segments.clear();

		if (point_cloud.empty())
			return;

		// The running lists of remaining points after each plane, as a matrix:
		CMatrixDouble remainingPoints(3, point_cloud.size());
		for (int i = 0; i < point_cloud.size(); ++i) {
			remainingPoints(0, i) = point_cloud[i].x;
			remainingPoints(1, i) = point_cloud[i].y;
			remainingPoints(2, i) = point_cloud[i].z;
		}

		// For each line
		while (size(remainingPoints, 2) >= 2)
		{
			vector_size_t this_best_inliers;
			CMatrixDouble this_best_model;

			mrpt::math::RANSAC::execute(
				remainingPoints,
				Ransac3DsegmentFit,
				Ransac3DsegmentDistance,
				Ransac3DsegmentDegenerate,
				threshold,
				2,  // Minimum set of points
				this_best_inliers,
				this_best_model,
				false, // Verbose
				0.99999  // Prob. of good result
			);

			// Is this line good enough?
			if (this_best_inliers.size() >= min_inliers_for_valid_line)
			{
				// Fit a line to the inliers
				Eigen::MatrixX3d matrix_a(this_best_inliers.size(), 3);
				for (int i = 0; i < this_best_inliers.size(); ++i) {
					int inlier_idx = this_best_inliers[i];
					matrix_a(i, 0) = remainingPoints(0, inlier_idx);
					matrix_a(i, 1) = remainingPoints(1, inlier_idx);
					matrix_a(i, 2) = remainingPoints(2, inlier_idx);
				}

				Eigen::MatrixX3d centered = matrix_a.rowwise() - matrix_a.colwise().mean();
				Eigen::Matrix3d cov = (centered.transpose() * centered) / double(matrix_a.rows());

				Eigen::EigenSolver<Eigen::MatrixXd> solver;
				solver.compute(cov, true);

				size_t max_index;
				double sqrt_eigen_value = std::sqrt(solver.eigenvalues().real().maximum(&max_index));
				Eigen::Vector3d director = solver.eigenvectors().col(max_index).real();
				director.normalize();

				// Get the projection of each inlier on the line
				Eigen::VectorXd a_projection = centered * director;
				Eigen::Vector3d mean = matrix_a.colwise().mean().transpose();
				Eigen::Vector3d a = mean + a_projection.maximum() * director;
				Eigen::Vector3d b = mean + a_projection.minimum() * director;
				
				/*
				Eigen::Vector3d mean = matrix_a.colwise().mean().transpose();
				Eigen::Vector3d a = mean - sqrt_eigen_value * director;
				Eigen::Vector3d b = mean + sqrt_eigen_value * director;
				*/

				// Discriminate the points on their y coordinate to get consistant results over time
				if (a.y() < b.y())
					out_detected_segments.push_back(std::make_pair<size_t, Of3dsegment>(this_best_inliers.size(), Of3dsegment(b.x(), b.y(), b.z(), a.x(), a.y(), a.z())));
				else
					out_detected_segments.push_back(std::make_pair<size_t, Of3dsegment>(this_best_inliers.size(), Of3dsegment(a.x(), a.y(), a.z(), b.x(), b.y(), b.z())));

				/*
				out_detected_segments.push_back(std::make_pair<size_t, OfEigen3dsegment>(this_best_inliers.size(), 
					OfEigen3dsegment(this_best_model(0, 0), this_best_model(0, 1), this_best_model(0, 2), 
									this_best_model(1, 0), this_best_model(1, 1), this_best_model(1, 2)))); */

				// Discard the selected points so they are not used again for finding subsequent planes:
				remainingPoints.removeColumns(this_best_inliers);
			}
			else
			{
				break; // Do not search for more lines
			}
		}
	}

} // namespace math
} // namespace garment_augmentation