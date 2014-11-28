#ifndef FLEXIBLE_SURFACE_AUGMENTATION_DEFORMATION_DETECTION_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_DEFORMATION_DETECTION_H_

#include <opencv2/cudev/common.hpp>

using namespace cv::cuda;

void DetectDeformations(const PtrStepSz<float3> world_coordinates, const float threshold,
	const int half_window_width, cudaStream_t stream, PtrStepSz<uchar> detected_deformations);

#endif // FLEXIBLE_SURFACE_AUGMENTATION_CUDA_TEST_H_