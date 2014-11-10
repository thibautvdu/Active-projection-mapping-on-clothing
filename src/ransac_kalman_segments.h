#ifndef FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_KALMAN_SEGMENTS_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_KALMAN_SEGMENTS_H_

#include <ofMain.h>
#include <mrpt/math.h>
#include <mrpt/math/ransac.h>
#include <mrpt/random.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "ofxCv.h"

#include "of_3dsegment.h"

using mrpt::vector_size_t;
using mrpt::math::CMatrixDouble;

namespace garment_augmentation {
namespace math {

	class RansacKalman3dSegmentTracker {

		public :

			RansacKalman3dSegmentTracker() {}

			void Track3Dsegments(const std::vector<ofVec3f> &point_cloud, const double threshold, const size_t min_inliers_for_valid_line);

			inline void SuppressSegment(int idx) {
				if (segments_.size() <= idx) {
					ofLogError("RansacKalman3dSegmentTracker::SuppressSegment") << "Index out of bound";
					return;
				}

				segments_.erase(segments_.begin() + idx);
				kalman_filters_.erase(kalman_filters_.begin() + idx);
			}

		private :
			// Segment model function
			static void SegmentModel(const CMatrixDouble  &all_data, const vector_size_t  &use_indices, std::vector< CMatrixDouble > &fit_models);

			// Return "true" if the selected points are a degenerate case (the same point or too close)
			static inline bool SegmentDegenerate(const CMatrixDouble &all_data, const mrpt::vector_size_t &use_indices) {
				MRPT_UNUSED_PARAM(all_data);
				MRPT_UNUSED_PARAM(use_indices);
				return false;
			}

			// Function to fit the model to an array of point
			static void SegmentDistance(const CMatrixDouble &allData, const std::vector< CMatrixDouble > & testModels, const double distanceThreshold,
				unsigned int & out_bestModelIndex, vector_size_t & out_inlierIndices);

			static inline bool ComparePoints(const std::pair<size_t, float> &a, const std::pair<size_t, float> &b) {
				return a.second < b.second;
			}

			static Of3dsegment LeastSquareFit(const vector_size_t &best_inliers, const CMatrixDouble &point_cloud);
			static Of3dsegment LeastSquareFit(const std::vector<int> &best_inliers, const std::vector<ofVec3f> &point_cloud);

			static void InitKalmanFilter(cv::KalmanFilter &kalman_filter) {
				kalman_filter.transitionMatrix = kalman_transition_matrix_init_;
				setIdentity(kalman_filter.measurementMatrix);
				setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(0));
				setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(0));
				setIdentity(kalman_filter.errorCovPost, cv::Scalar::all(0));
			}

			std::vector<std::pair<Of3dsegment,bool>> segments_; // true: has been updated with measurements, false: pure estimation
			std::vector<cv::KalmanFilter> kalman_filters_;

			const static cv::Mat kalman_transition_matrix_init_;
	};

} // namespace math
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_KALMAN_SEGMENTS_H_