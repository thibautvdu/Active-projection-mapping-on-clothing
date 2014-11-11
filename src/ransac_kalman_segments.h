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
			RansacKalman3dSegmentTracker() : segments_life_time_(1) {}
			RansacKalman3dSegmentTracker(const float segments_life_time) : segments_life_time_(segments_life_time) {}

			void Track3Dsegments(const std::vector<ofVec3f> &point_cloud, const double threshold, const size_t min_inliers_for_valid_line, std::vector<std::pair<Of3dsegment, float>> &out_segments);

			inline void SuppressSegment(int idx) {
				if (segments_.size() <= idx) {
					ofLogError("RansacKalman3dSegmentTracker::SuppressSegment") << "Index out of bound";
					return;
				}

				segments_.erase(segments_.begin() + idx);
				kalman_filters_.erase(kalman_filters_.begin() + idx);
			}

			inline void set_segments_life_time(const float lifetime) {
				segments_life_time_ = lifetime;
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

			inline static void InitKalmanFilter(cv::KalmanFilter &kalman_filter, ofVec3f point_a, ofVec3f point_b) {
				kalman_filter.transitionMatrix = k_kalman_transition_matrix_init_;

				float *p_post_state = kalman_filter.statePost.ptr<float>(0);
				// Two segments points
				p_post_state[0] = point_a.x; p_post_state[1] = point_a.y; p_post_state[2] = point_a.z;
				p_post_state[3] = point_b.x; p_post_state[4] = point_b.y; p_post_state[5] = point_b.z;
				// Velocity to 0
				p_post_state[6] = 0; p_post_state[7] = 0; p_post_state[8] = 0;
				p_post_state[9] = 0; p_post_state[10] = 0; p_post_state[11] = 0;

				cv::setIdentity(kalman_filter.measurementMatrix);
				cv::setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(0.001));
				cv::setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(0.01));
				cv::setIdentity(kalman_filter.errorCovPost, cv::Scalar::all(0.1));
			}

			inline static void UpdateKalmanDeltaTime(cv::KalmanFilter &kalman_filter, float delta_time) {
				kalman_filter.transitionMatrix.ptr<float>(0)[6] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(1)[7] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(2)[8] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(3)[9] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(4)[10] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(5)[11] = delta_time;
			}

			std::vector<std::pair<Of3dsegment,float>> segments_; // float => lifetime
			std::vector<cv::KalmanFilter> kalman_filters_;
			float segments_life_time_;

			const static cv::Mat k_kalman_transition_matrix_init_;
	};

} // namespace math
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_KALMAN_SEGMENTS_H_