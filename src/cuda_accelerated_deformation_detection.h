#ifndef FLEXIBLE_SURFACE_AUGMENTATION_CUDA_ACCELERATED_DEFORMATION_DETECTION_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_CUDA_ACCELERATED_DEFORMATION_DETECTION_H_

#include <opencv2\cuda.hpp>

namespace garment_augmentation {
namespace cuda_optimization {

	void AcceleratedDeformationsDetection(const cv::cuda::GpuMat &world_coordinates, cv::cuda::GpuMat &output, cv::cuda::Stream& s = cv::cuda::Stream::Null());

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_CUDA_ACCELERATED_DEFORMATION_DETECTION_H_