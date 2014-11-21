#ifndef FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_KALMAN_SEGMENTS_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_KALMAN_SEGMENTS_H_

#include <ofMain.h>
#include <mrpt/math.h>
#include <mrpt/math/ransac.h>
#include <mrpt/random.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "ofxCv.h"

#include "of_3dsegment_orientation.h"

using mrpt::vector_size_t;
using mrpt::math::CMatrixDouble;

namespace garment_augmentation {
namespace math {

	class RansacKalman3dSegmentTracker {

		public :
			RansacKalman3dSegmentTracker() : segments_life_time_(1) {}
			RansacKalman3dSegmentTracker(const float segments_life_time) : segments_life_time_(segments_life_time) {}

			void Track3Dsegments(const std::vector<ofVec3f> &point_cloud, const double threshold, const size_t min_inliers_for_valid_line, std::vector<std::pair<Of3dsegmentOrientation, float>> &out_segments);

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

			void TuneKalmanCovariances(float process_noise, float measurement_noise, float post_error) {
				for (int i = 0; i < kalman_filters_.size(); ++i) {
					cv::setIdentity(kalman_filters_[i].processNoiseCov, cv::Scalar::all(process_noise));
					cv::setIdentity(kalman_filters_[i].measurementNoiseCov, cv::Scalar::all(measurement_noise));
					cv::setIdentity(kalman_filters_[i].errorCovPost, cv::Scalar::all(post_error));
				}
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

			static Of3dsegmentOrientation LeastSquareFit(const vector_size_t &best_inliers, const CMatrixDouble &point_cloud);
			static Of3dsegmentOrientation LeastSquareFit(const std::vector<int> &best_inliers, const std::vector<ofVec3f> &point_cloud);

			inline static void InitKalmanFilter(cv::KalmanFilter &kalman_filter, Of3dsegmentOrientation segment) {
				kalman_filter.transitionMatrix = k_kalman_transition_matrix_init_;

				float *p_post_state = kalman_filter.statePost.ptr<float>(0);
				// Orientation
				p_post_state[0] = segment.orientation().x; p_post_state[1] = segment.orientation().y; p_post_state[2] = segment.orientation().z;
				// Center
				p_post_state[3] = segment.center().x; p_post_state[4] = segment.center().y; p_post_state[5] = segment.center().z;
				// Orientation
				p_post_state[6] = segment.length();
				// Orientation velocity
				p_post_state[7] = 0; p_post_state[8] = 0; p_post_state[9] = 0;
				// Center velocity
				p_post_state[10] = 0; p_post_state[11] = 0; p_post_state[12] = 0;

				cv::setIdentity(kalman_filter.measurementMatrix);
				cv::setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(0.0004));
				cv::setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(0.0009));
				cv::setIdentity(kalman_filter.errorCovPost, cv::Scalar::all(0.025));
			}

			inline static void UpdateKalmanDeltaTime(cv::KalmanFilter &kalman_filter, float delta_time) {
				kalman_filter.transitionMatrix.ptr<float>(0)[7] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(1)[8] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(2)[9] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(3)[10] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(4)[11] = delta_time;
				kalman_filter.transitionMatrix.ptr<float>(5)[12] = delta_time;
			}

			std::vector<std::pair<Of3dsegmentOrientation,float>> segments_; // float => lifetime
			std::vector<cv::KalmanFilter> kalman_filters_;
			float segments_life_time_;

			const static cv::Mat k_kalman_transition_matrix_init_;
	};

} // namespace math
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_KALMAN_SEGMENTS_H_