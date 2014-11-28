#include "cuda_accelerated_deformation_detection.h"

#include <opencv2/core/cuda_stream_accessor.hpp>

#include "deformation_detection.h"

namespace garment_augmentation {
namespace cuda_optimization {

	void AcceleratedDeformationsDetection(const cv::cuda::GpuMat &world_coordinates, cv::cuda::GpuMat &output, cv::cuda::Stream& s) {
		cv::cuda::PtrStepSz<float3> ptr_world = world_coordinates;
		cv::cuda::PtrStepSz<float3> ptr_output = output;
		cudaStream_t stream = cv::cuda::StreamAccessor::getStream(s);
		DetectDeformations(world_coordinates, 0.1, 5, stream, output);
	}

}
}