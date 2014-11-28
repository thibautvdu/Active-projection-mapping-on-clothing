#include <opencv2/cudev/common.hpp>
#include <cuda/cuda.h>

using namespace cv::cuda;

__global__ void swap_rb_kernel(const PtrStepSz<uchar3> src, PtrStep<uchar3> dst);

void swap_rb_caller(const PtrStepSz<uchar3>& src, PtrStep<uchar3> dst, cudaStream_t stream);