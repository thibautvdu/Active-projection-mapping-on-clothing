#include "ransac_kalman_segments.h"

namespace garment_augmentation {
namespace math {

	using mrpt::math::TPoint3D;
	using mrpt::math::TLine3D;
	using mrpt::math::CMatrixTemplateNumeric;

	void RansacKalman3dSegmentTracker::SegmentModel(const CMatrixDouble  &all_data, const vector_size_t  &use_indices, std::vector< CMatrixDouble > &fit_models) {
		ASSERT_(use_indices.size() == 2);

		try
		{
			fit_models.resize(1);
			CMatrixDouble &M = fit_models[0];

			M.setSize(2, 1);
			M(0, 0) = use_indices[0];
			M(1, 0) = use_indices[1];
		}
		catch (std::exception &)
		{
			fit_models.clear();
			return;
		}
	}

	namespace {
		int k_min_inliers_for_valid_segment_;
	}

	void RansacKalman3dSegmentTracker::SegmentDistance(const CMatrixDouble &all_data, const std::vector< CMatrixDouble > & test_models, const double distance_threshold,
		unsigned int & out_bestModelIndex, vector_size_t & out_inlierIndices) {
		ASSERT_(test_models.size() == 1)
			out_bestModelIndex = 0;
		const CMatrixDouble &M = test_models[0];

		ASSERT_(size(M, 1) == 2 && size(M, 2) == 1);
		TLine3D line;
		try
		{
			line = TLine3D(
				TPoint3D(all_data.get_unsafe(0, M(0, 0)), all_data.get_unsafe(1, M(0, 0)), all_data.get_unsafe(2, M(0, 0))),
				TPoint3D(all_data.get_unsafe(0, M(1, 0)), all_data.get_unsafe(1, M(1, 0)), all_data.get_unsafe(2, M(1, 0)))
				);
		}
		catch (std::logic_error &)
		{
			out_inlierIndices.clear();
			return;
		}

		// Test the verticality of the line first
		double director[3];
		line.getUnitaryDirectorVector(director);
		double angle_cos = /*director[0] * 0 +*/ director[1] * 1 /* + director[2] * 0*/; // angle with vector (0,1,0)
		if (angle_cos < 0.9 && angle_cos > -0.9)
			return;

		const size_t N = size(all_data, 2);
		std::vector<std::pair<size_t, float>> inliers;
		inliers.reserve(30);
		TPoint3D candidate;
		double last_x = std::numeric_limits<double>::infinity(); // Used to ensure we don't include horizontally aligned inliers
		double current_x;
		for (size_t i = 0; i<N; i++)
		{
			if (fabs((current_x = all_data.get_unsafe(0, i)) - last_x) > 0.001) { // > 1mm
				candidate = TPoint3D(current_x, all_data.get_unsafe(1, i), all_data.get_unsafe(2, i));
				const double d = line.distance(candidate);
				if (d < distance_threshold) {
					inliers.push_back(std::pair<size_t, float>(i, candidate.y));
					last_x = current_x;
				}
			}
		}

		// Sort the inliers according to the Y axis
		if (inliers.size() > k_min_inliers_for_valid_segment_) {
			std::sort(inliers.begin(), inliers.end(), ComparePoints);

			// Look for the biggest group if there is an undesired gap along Y
			int max_group_begin = 0, max_group_size = 1;
			int temp_group_begin = 0;
			for (int i = 1; i < inliers.size(); ++i) {
				if (inliers[i].second - inliers[i - 1].second > 0.2) { // 20cm
					if ((i - 1) - temp_group_begin > max_group_size) {
						max_group_begin = temp_group_begin;
						max_group_size = (i - 1) - temp_group_begin;
					}
					temp_group_begin = i;
				}
			}
			if ((inliers.size() - 1) - temp_group_begin > max_group_size) {
				max_group_begin = temp_group_begin;
				max_group_size = (inliers.size() - 1) - temp_group_begin;
			}

			out_inlierIndices.clear();
			out_inlierIndices.reserve(max_group_size);
			for (int i = max_group_begin; i < max_group_begin + max_group_size; ++i) {
				out_inlierIndices.push_back(inliers[i].first);
			}
		}
	}

	void RansacKalman3dSegmentTracker::Track3Dsegments(const std::vector<ofVec3f> &point_cloud, const double threshold, const size_t min_inliers_for_valid_segment, std::vector<std::pair<Of3dsegment,float>> &out_segments) {

		if (point_cloud.empty())
			return;

		// Update the current segments with kalman filter
		std::vector<int> point_cloud_index_remaining;
		point_cloud_index_remaining.reserve(point_cloud.size());
		if (!kalman_filters_.empty()) {

			// Compute the estimate of the segments
			cv::Mat prediction_mat;
			std::vector<std::pair<TSegment3D, std::vector<int>>> predictions(kalman_filters_.size());
			float last_frame_time = ofGetLastFrameTime();
			for (int i = 0; i < kalman_filters_.size(); ++i) {

				UpdateKalmanDeltaTime(kalman_filters_[i], last_frame_time);

				prediction_mat = kalman_filters_[i].predict();
				float * p_prediction_mat = prediction_mat.ptr<float>(0);
				predictions[i].first = TSegment3D(
					TPoint3D(p_prediction_mat[0], p_prediction_mat[1], p_prediction_mat[2]),
					TPoint3D(p_prediction_mat[3], p_prediction_mat[4], p_prediction_mat[5])
					);
				predictions[i].second.reserve(20);
			}

			// Affect the point to the best segment if one or more are in reach
			TPoint3D candidate;
			for (size_t i = 0; i < point_cloud.size(); ++i)
			{
				double d_min = std::numeric_limits<double>::infinity();
				int d_min_segment_idx = -1;
				for (int j = 0; j < predictions.size(); ++j) {
					candidate = TPoint3D(point_cloud[i].x, point_cloud[i].y, point_cloud[i].z);
					const double d = predictions[j].first.distance(candidate);
					if (d < threshold && d < d_min) {
						d_min = d;
						d_min_segment_idx = j;
					}
				}
				if (d_min_segment_idx != -1) {
					predictions[d_min_segment_idx].second.push_back(i);
				}
				else {
					point_cloud_index_remaining.push_back(i);
				}
			}

			// Compute the mesured segments and get their corrected estimate if there are enough points
			Of3dsegment measured_segment;
			cv::Mat measurements;
			for (int i = 0; i < kalman_filters_.size(); ++i) {
				if (predictions[i].second.size() >= min_inliers_for_valid_segment) {
					measured_segment = LeastSquareFit(predictions[i].second, point_cloud);
					measurements = (cv::Mat_<float>(6, 1) << 
						measured_segment.a().x, measured_segment.a().y, measured_segment.a().z, 
						measured_segment.b().x, measured_segment.b().y, measured_segment.b().z);
					prediction_mat = kalman_filters_[i].correct(measurements);

					float * p_prediction_mat = prediction_mat.ptr<float>(0);
					segments_[i].first = Of3dsegment(p_prediction_mat[0], p_prediction_mat[1], p_prediction_mat[2],
						p_prediction_mat[3], p_prediction_mat[4], p_prediction_mat[5]);
					segments_[i].second = segments_life_time_; // Renew the life time
				}
				else {
					segments_[i].first = Of3dsegment(predictions[i].first.point1.x, predictions[i].first.point1.y, predictions[i].first.point1.z,
						predictions[i].first.point2.x, predictions[i].first.point2.y, predictions[i].first.point2.z);
					segments_[i].second -= last_frame_time;
				}
			}
		}

		// Suppress the eventual segments whose life time is expired
		for (int i = segments_.size() - 1; i >= 0; --i) {
			if (0 >= segments_[i].second) {
				SuppressSegment(i);
			}
		}

		// Remaining points for RANSAC algorithm
		CMatrixDouble remaining_points;
		if (kalman_filters_.empty()) {
			remaining_points = CMatrixDouble(3, point_cloud.size());

			for (int i = 0; i < point_cloud.size(); ++i) {
				remaining_points(0, i) = point_cloud[i].x;
				remaining_points(1, i) = point_cloud[i].y;
				remaining_points(2, i) = point_cloud[i].z;
			}
		}
		else {
			remaining_points = CMatrixDouble(3, point_cloud_index_remaining.size());

			for (int i = 0; i < point_cloud_index_remaining.size(); ++i) {
				remaining_points(0, i) = point_cloud[point_cloud_index_remaining[i]].x;
				remaining_points(1, i) = point_cloud[point_cloud_index_remaining[i]].y;
				remaining_points(2, i) = point_cloud[point_cloud_index_remaining[i]].z;
			}
		}

		k_min_inliers_for_valid_segment_ = min_inliers_for_valid_segment;
		// For each line
		while (size(remaining_points, 2) >= min_inliers_for_valid_segment)
		{
			vector_size_t this_best_inliers;
			CMatrixDouble this_best_model;

			mrpt::math::RANSAC::execute(
				remaining_points,
				SegmentModel,
				SegmentDistance,
				SegmentDegenerate,
				threshold,
				2,  // Minimum set of points
				this_best_inliers,
				this_best_model,
				false, // Verbose
				0.99999  // Prob. of good result
				);

			// Is this line good enough?
			if (this_best_inliers.size() >= min_inliers_for_valid_segment)
			{
				Of3dsegment fitted = LeastSquareFit(this_best_inliers, remaining_points);

				segments_.push_back(std::pair<Of3dsegment, float>(fitted, segments_life_time_));
				kalman_filters_.push_back(cv::KalmanFilter(12, 6));
				InitKalmanFilter(*(kalman_filters_.end()-1), fitted.a(), fitted.b());

				// Discard the selected points so they are not used again for finding subsequent planes:
				remaining_points.removeColumns(this_best_inliers);
			}
			else
			{
				break; // Do not search for more lines
			}
		}

		out_segments = segments_;
	}

	Of3dsegment RansacKalman3dSegmentTracker::LeastSquareFit(const vector_size_t &best_inliers, const CMatrixDouble &point_cloud) {
		Of3dsegment res;

		// Fit a line to the inliers
		Eigen::MatrixX3d matrix_a(best_inliers.size(), 3);
		for (int i = 0; i < best_inliers.size(); ++i) {
			int inlier_idx = best_inliers[i];
			matrix_a(i, 0) = point_cloud(0,inlier_idx);
			matrix_a(i, 1) = point_cloud(1, inlier_idx);
			matrix_a(i, 2) = point_cloud(2, inlier_idx);
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
		std::sort(a_projection.data(), a_projection.data() + a_projection.size());

		double a_max = a_projection[a_projection.size() - 1];
		double a_min = a_projection[0];

		Eigen::Vector3d mean = matrix_a.colwise().mean().transpose();
		Eigen::Vector3d a = mean + a_max * director;
		Eigen::Vector3d b = mean + a_min * director;

		// Discriminate the points on their y coordinate to get consistant results over time
		if (a.y() < b.y())
			res = Of3dsegment(a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
		else
			res = Of3dsegment(b.x(), b.y(), b.z(), a.x(), a.y(), a.z());

		return res;
	}

	Of3dsegment RansacKalman3dSegmentTracker::LeastSquareFit(const std::vector<int> &best_inliers, const std::vector<ofVec3f> &point_cloud) {
		Of3dsegment res;

		// Fit a line to the inliers
		Eigen::MatrixX3d matrix_a(best_inliers.size(), 3);
		for (int i = 0; i < best_inliers.size(); ++i) {
			int inlier_idx = best_inliers[i];
			matrix_a(i, 0) = point_cloud[inlier_idx].x;
			matrix_a(i, 1) = point_cloud[inlier_idx].y;
			matrix_a(i, 2) = point_cloud[inlier_idx].z;
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
		std::sort(a_projection.data(), a_projection.data() + a_projection.size());

		double a_max = a_projection[a_projection.size() - 1];
		double a_min = a_projection[0];

		Eigen::Vector3d mean = matrix_a.colwise().mean().transpose();
		Eigen::Vector3d a = mean + a_max * director;
		Eigen::Vector3d b = mean + a_min * director;

		// Discriminate the points on their y coordinate to get consistant results over time
		if (a.y() < b.y())
			res = Of3dsegment(a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
		else
			res = Of3dsegment(b.x(), b.y(), b.z(), a.x(), a.y(), a.z());

		return res;
	}

	const cv::Mat RansacKalman3dSegmentTracker::k_kalman_transition_matrix_init_ = (cv::Mat_<float>(12, 12) <<
		1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, // (0,6)  -> dt
		0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, // (1,7)  -> dt
		0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, // (2,8)  -> dt
		0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, // (3,9)  -> dt
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, // (4,10) -> dt
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, // (5,11) -> dt
		0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);

} // namespace math
} // namespace garment_augmentation