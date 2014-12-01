#ifndef FLEXIBLE_SURFACE_AUGMENTATION_CUDA_ACCELERATED_DEFORMATION_DETECTION_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_CUDA_ACCELERATED_DEFORMATION_DETECTION_H_

#include <opencv2/cuda.hpp>
#include <opencv2/cudev/common.hpp>

namespace garment_augmentation {
namespace cuda_optimization {

	using namespace cv::cuda;

	void AcceleratedDeformationsDetection(const GpuMat world_coordinates, const float threshold, const int window_width, GpuMat detected_deformations, cv::cuda::Stream& s = cv::cuda::Stream::Null());

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_CUDA_ACCELERATED_DEFORMATION_DETECTION_H_