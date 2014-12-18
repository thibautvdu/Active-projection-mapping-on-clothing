// Combination of a kalman filtering, iterative RANSAC algorithm and least square fitting
// please refer to the poster or paper for details on the logic and algorithm
// This is based on the MRPT library's RANSAC template, for more details on the various
// functions rewritten here, please refer to the MRPT website

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


			/* MRPT RANSAC ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	*/

			// Track the existing 3D segments in the point cloud and look for new segments
			// the threshold parameter is both used as the distance threshold for assigning 
			// inliers to exising fold's kalman predictions as well as to find new segments
			// with the RANSAC algorithm
			// In the same logic, the minimin of inliers is both used to valid an update of
			// an existing fold and to validate the creation of a new segment i the ouptput of
			// the RANSAC algorithm
			// The point cloud parameter is the collection of craters' centers we retrieved with
			// the CUDA craters/lumps detection algorithm
			void Track3Dsegments(const std::vector<ofVec3f> &point_cloud, const double threshold, const size_t min_inliers_for_valid_line, std::vector<std::pair<Of3dsegmentOrientation, float>> &out_segments);

			/* MRPT RANSAC ROUTINES	-	-	-	-	-	-	-	-	-	-	-	-	*/



			/* VARIOUS FUNCTIONS	/	/	/	/	/	/	/	/	/	/	/	/	*/

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

			/* VARIOUS FUNCTIONS	-	-	-	-	-	-	-	-	-	-	-	-	*/



			/* KALMAN FUNCTIONS	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

			// Tune the kalman covariances. Please notice that outside of these value, each of
			// the parameters have a different weight in the covariances' error matrices as explained
			// in the paper and briefly in the poster
			void TuneKalmanCovariances(float process_noise, float measurement_noise, float post_error) {
				for (int i = 0; i < kalman_filters_.size(); ++i) {
					kalman_filters_[0].processNoiseCov = k_process_noise_cov_ * process_noise;
					kalman_filters_[0].measurementNoiseCov = k_measurement_noise_cov_ * measurement_noise;
					kalman_filters_[0].errorCovPost = k_error_cov_post_ * post_error;
				}
			}

			// Use or not velocity in the transition matrix
			void SetVelocityUse(bool use) {
				use_velocity_ = use;
			}

			/* KALMAN FUNCTIONS	-	-	-	-	-	-	-	-	-	-	-	-	-	*/

		private :

			/* MRPT RANSAC ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	*/

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

			/* MRPT RANSAC ROUTINES	-	-	-	-	-	-	-	-	-	-	-	-	*/



			/* VARIOUS FUNCTIONS	/	/	/	/	/	/	/	/	/	/	/	/	*/

			// Least square line fitting functions from a point cloud (openframeworks or MRPT) 
			static Of3dsegmentOrientation LeastSquareFit(const vector_size_t &best_inliers, const CMatrixDouble &point_cloud);
			static Of3dsegmentOrientation LeastSquareFit(const std::vector<int> &best_inliers, const std::vector<ofVec3f> &point_cloud);

			/* VARIOUS FUNCTIONS	-	-	-	-	-	-	-	-	-	-	-	-	*/



			/* KALMAN FUNCTIONS	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

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

				// Dummy values as it is updated at every frame in the main loop...
				kalman_filter.processNoiseCov =  k_process_noise_cov_ * 0.0004;
				kalman_filter.measurementNoiseCov = k_measurement_noise_cov_ * 0.0009;
				kalman_filter.errorCovPost = k_error_cov_post_ * 0.025;
			}

			// Update the delta time before computing the prediction to make it consistent
			// with the real time
			inline static void UpdateKalmanDeltaTime(cv::KalmanFilter &kalman_filter, float delta_time) {
					kalman_filter.transitionMatrix.ptr<float>(0)[7] = delta_time;
					kalman_filter.transitionMatrix.ptr<float>(1)[8] = delta_time;
					kalman_filter.transitionMatrix.ptr<float>(2)[9] = delta_time;
					kalman_filter.transitionMatrix.ptr<float>(3)[10] = delta_time;
					kalman_filter.transitionMatrix.ptr<float>(4)[11] = delta_time;
					kalman_filter.transitionMatrix.ptr<float>(5)[12] = delta_time;
			}

			/* KALMAN FUNCTIONS	-	-	-	-	-	-	-	-	-	-	-	-	-	*/



			/* INTERNAL VARIABLES	/	/	/	/	/	/	/	/	/	/	/	/	*/

			std::vector<std::pair<Of3dsegmentOrientation,float>> segments_; // float => lifetime
			std::vector<cv::KalmanFilter> kalman_filters_;
			float segments_life_time_;
			bool use_velocity_;

			const static cv::Mat k_kalman_transition_matrix_init_;
			const static cv::Mat k_process_noise_cov_;
			const static cv::Mat k_measurement_noise_cov_;
			const static cv::Mat k_error_cov_post_;

			/* INTERNAL VARIABLES	-	-	-	-	-	-	-	-	-	-	-	-	*/
	};

} // namespace math
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_KALMAN_SEGMENTS_H_