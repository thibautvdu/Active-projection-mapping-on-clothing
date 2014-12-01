#ifndef FLEXIBLE_SURFACE_AUGMENTATION_CUDA_MATH_VECTORS_HELPER_CUH_
#define FLEXIBLE_SURFACE_AUGMENTATION_CUDA_MATH_VECTORS_HELPER_CUH_

#include <opencv2/cudev/common.hpp>

namespace garment_augmentation {
namespace cuda_optimization {

	__device__ inline float3 operator/(const float3 &a, float scal) {
		return make_float3(a.x / scal, a.y / scal, a.z / scal);
	}

	__device__ inline float3 operator*(const float3 &a, float scal) {
		return make_float3(a.x * scal, a.y * scal, a.z * scal);
	}

	__device__ inline float3 operator-(const float3 &a, const float3 &b) {
		return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	__device__ inline float3 operator+(const float3 &a, const float3 &b) {
		return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
	}

	__device__ inline float3 Crossed(const float3 &a, const float3 &b) {
		return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
	}

	__device__ inline float Dot(const float3 &a, const float3 &b) {
		return a.x*b.x + a.y*b.y + a.z*b.z;
	}

	__device__ inline float Norm(const float3 &a) {
		return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
	}

	__device__ inline void Normalize(float3 &a) {
		a = a / Norm(a);
	}

}
}

#endif