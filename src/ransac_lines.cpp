#include "ransac_lines.h"

namespace garment_augmentation {
namespace math {

using mrpt::math::TPoint3D;
using mrpt::math::CMatrixTemplateNumeric;

	void Ransac3DlineFit(const CMatrixDouble  &all_data, const vector_size_t  &use_indices, std::vector< CMatrixDouble > &fit_models) {
		ASSERT_(use_indices.size() == 2);

		TPoint3D  p1(all_data(0, use_indices[0]), all_data(1, use_indices[0]), all_data(2, use_indices[0]));
		TPoint3D  p2(all_data(0, use_indices[1]), all_data(1, use_indices[1]), all_data(2, use_indices[1]));

		try
		{
			TLine3D  line(p1, p2);
			fit_models.resize(1);
			CMatrixDouble &M = fit_models[0];

			M.setSize(2, 3);
			for (size_t i = 0; i < 3; i++) {
				M(0, i) = line.pBase[i];
				M(1, i) = line.director[i];
			}
		}
		catch (std::exception &)
		{
			fit_models.clear();
			return;
		}
	}

	void Ransac3DlineDistance(const CMatrixDouble &all_data, const std::vector< CMatrixDouble > & test_models, const double distance_threshold,
		unsigned int & out_bestModelIndex, vector_size_t & out_inlierIndices) {
		ASSERT_(test_models.size() == 1)
			out_bestModelIndex = 0;
		const CMatrixDouble &M = test_models[0];

		ASSERT_(size(M, 1) == 2 && size(M, 2) == 3)

		TLine3D line;
		line.pBase[0] = M(0, 0);
		line.pBase[1] = M(0, 1);
		line.pBase[2] = M(0, 2);
		line.director[0] = M(1, 0);
		line.director[1] = M(1, 1);
		line.director[2] = M(1, 2);

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

	void RansacDetect3Dlines( const std::vector<ofVec3f> &point_cloud,
		std::vector<std::pair<size_t, TLine3D> >   &out_detected_lines,
		const double threshold,
		const size_t min_inliers_for_valid_line
		) {

		out_detected_lines.clear();

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
		TLine3D validLine;
		while (size(remainingPoints, 2) >= 2)
		{
			vector_size_t	this_best_inliers;
			CMatrixDouble this_best_model;

			mrpt::math::RANSAC::execute(
				remainingPoints,
				Ransac3DlineFit,
				Ransac3DlineDistance,
				Ransac3DlineDegenerate,
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
				validLine.pBase[0] = this_best_model(0, 0);
				validLine.pBase[1] = this_best_model(0, 1);
				validLine.pBase[2] = this_best_model(0, 2);
				validLine.director[0] = this_best_model(1, 0);
				validLine.director[1] = this_best_model(1, 1);
				validLine.director[2] = this_best_model(1, 2);
				// Add this line to the output list:
				out_detected_lines.push_back(std::make_pair<size_t, TLine3D>(this_best_inliers.size(), TLine3D(validLine)));
				out_detected_lines.rbegin()->second.unitarize();

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