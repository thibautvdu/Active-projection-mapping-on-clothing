#ifndef FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_LINES_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_LINES_H_

#include <ofMain.h>
#include <mrpt/math.h>
#include <mrpt/math/ransac.h>
#include <mrpt/random.h>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "of_3dsegment.h"

namespace garment_augmentation {
namespace math {

using mrpt::vector_size_t;
using mrpt::math::CMatrixDouble;

	void RansacDetect3Dsegments(const std::vector<ofVec3f> &point_cloud, const double threshold, const size_t min_inliers_for_valid_line,
		std::vector<std::pair<size_t, Of3dsegment> > &out_detected_segments);

	// Function to calculate distances between a line and an array of points.
	void Ransac3DsegmentDistance(const CMatrixDouble &allData, const std::vector< CMatrixDouble > & testModels, const double distanceThreshold,
		unsigned int & out_bestModelIndex, vector_size_t & out_inlierIndices);

	inline bool ComparePoints(const std::pair<size_t, float> &a, const std::pair<size_t, float> &b) {
		return a.second < b.second;
	}

	// Return "true" if the selected points are a degenerate case (the same point or too close)
	inline bool Ransac3DsegmentDegenerate(const CMatrixDouble &all_data, const mrpt::vector_size_t &use_indices) {
		MRPT_UNUSED_PARAM(all_data);
		MRPT_UNUSED_PARAM(use_indices);
		return false;
	}

	// Define the line fitting fun
	void Ransac3DsegmentFit(const CMatrixDouble  &all_data, const vector_size_t  &use_indices, std::vector< CMatrixDouble > &fit_models);

} // namespace math
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_RANSAC_LINES_H_