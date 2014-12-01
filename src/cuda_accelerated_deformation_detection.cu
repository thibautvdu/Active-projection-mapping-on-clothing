#include "cuda_accelerated_deformation_detection.h"

#include <opencv2/cudev/common.hpp>
#include <stdio.h>

#include "cuda_math_vectors_helper.cuh"

namespace garment_augmentation {
namespace cuda_optimization {

	__device__ float3 VectorRotationZ(float3 from, float3 to) {
		float3 crossed = Crossed(from, to);
		float dot = Dot(from,to);
		float crossed_norm = Norm(crossed);

		float3 third_base = (to - from*dot);
		Normalize(third_base);

		// local rotation matrix
		float lrm_a = dot;
		float lrm_b = -crossed_norm;
		float lrm_c = 0.f;
		float lrm_d = crossed_norm;
		float lrm_e = dot;
		float lrm_f = 0.f;
		float lrm_g = 0.f;
		float lrm_h = 0.f;
		float lrm_i = 1.f;

		// to local space transformation matrix
		float lsm_a = from.x;
		float lsm_b = third_base.x;
		float lsm_c = -crossed.x;
		float lsm_d = from.y;
		float lsm_e = third_base.y,
		float lsm_f = -crossed.y;
		float lsm_g = from.z;
		float lsm_h = third_base.z,
		float lsm_i = -crossed.z;

		float determinant_local_space_matrix = 
			lsm_a * lsm_e * lsm_i
			+ lsm_b * lsm_f * lsm_g
			+ lsm_d * lsm_h * lsm_c
			- lsm_g * lsm_e * lsm_c
			- lsm_d * lsm_b * lsm_i
			- lsm_h * lsm_f * lsm_a;

		// to world space transformation matrix
		float wsm_a = (lsm_e * lsm_i - lsm_h * lsm_f) / determinant_local_space_matrix;
		float wsm_b = (-lsm_b * lsm_i + lsm_h * lsm_c) / determinant_local_space_matrix;
		float wsm_c = (lsm_b * lsm_f - lsm_e * lsm_c) / determinant_local_space_matrix;
		float wsm_d = (-lsm_d * lsm_i + lsm_g * lsm_f) / determinant_local_space_matrix;
		float wsm_e = (lsm_a * lsm_i - lsm_g * lsm_c) / determinant_local_space_matrix;
		float wsm_f = (-lsm_a * lsm_f + lsm_d * lsm_c) / determinant_local_space_matrix;
		float wsm_g = (lsm_d * lsm_h - lsm_g * lsm_e) / determinant_local_space_matrix;
		float wsm_h = (-lsm_a * lsm_h + lsm_g * lsm_b) / determinant_local_space_matrix;
		float wsm_i = (lsm_a * lsm_e - lsm_d * lsm_b) / determinant_local_space_matrix;

		float3 transform_on_z;
		transform_on_z.x =
			lsm_g*(lrm_a*wsm_a + lrm_b*wsm_d + lrm_c*wsm_g) + 
			lsm_h*(lrm_d*wsm_a + lrm_e*wsm_d + lrm_f*wsm_g) + 
			lsm_i*(lrm_g*wsm_a + lrm_h*wsm_d + lrm_i*wsm_g);
		transform_on_z.y =
			lsm_g*(lrm_a*wsm_b + lrm_b*wsm_e + lrm_c*wsm_h) +
			lsm_h*(lrm_d*wsm_b + lrm_e*wsm_e + lrm_f*wsm_h) +
			lsm_i*(lrm_g*wsm_b + lrm_h*wsm_e + lrm_i*wsm_h);
		transform_on_z.z =
			lsm_g*(lrm_a*wsm_c + lrm_b*wsm_f + lrm_c*wsm_i) +
			lsm_h*(lrm_d*wsm_c + lrm_e*wsm_f + lrm_f*wsm_i) +
			lsm_i*(lrm_g*wsm_c + lrm_h*wsm_f + lrm_i*wsm_i);

		return transform_on_z;
	}

	__device__ float DeltaDepth(const PtrStepSz<float3> world_coordinates, const float3 average_center, const float3 transformation, int x, int y, int half_width) {
		float delta_depth = 0;
		int nb_points = 0;

		for (int row = y - half_width; row <= y + half_width; row++) {
			for (int col = x - half_width; col <= x + half_width; col++) {
				if (!isinf(world_coordinates(row, col).z)) {
					float3 point = world_coordinates(row, col) - average_center; // Relative to the middle of the patch
					delta_depth += point.x * transformation.x + point.y * transformation.y + point.z * transformation.z; // Rotation to the normal of the patch
					nb_points++;
				}
			}
		}

		return delta_depth / nb_points;
	}

	// detected_deformations contain 0 if the deformation value of the area < threshold or the size of the area if >= threshold
	// half_window_width should be equal to floor(window_width/2)
	__global__ void ComputeDeformations(const PtrStepSz<float3> world_coordinates, const float threshold,
		const int half_window_width, PtrStepSz<uchar> detected_deformations) {
		int x = threadIdx.x + blockIdx.x * blockDim.x;
		int y = threadIdx.y + blockIdx.y * blockDim.y;

		if (x < world_coordinates.cols - half_window_width && y < world_coordinates.rows - half_window_width && x >= half_window_width && y >= half_window_width)
		{
			if (isinf(world_coordinates(x,y).z)) {
				detected_deformations(y, x) = 0;
				return;
			}
			float3 top_left_pt = world_coordinates(y - half_window_width, x - half_window_width);
			if (isinf(top_left_pt.z)) {
				detected_deformations(y, x) = 0;
				return;
			}
			float3 top_right_pt = world_coordinates(y - half_window_width, x + half_window_width);
			if (isinf(top_right_pt.z)) {
				detected_deformations(y, x) = 0;
				return;
			}
			float3 bottom_left_pt = world_coordinates(y + half_window_width, x - half_window_width);
			if (isinf(bottom_left_pt.z)) {
				detected_deformations(y, x) = 0;
				return;
			}
			float3 bottom_right_pt = world_coordinates(y + half_window_width, x + half_window_width);
			if (isinf(bottom_right_pt.z)) {
				detected_deformations(y, x) = 0;
				return;
			}

			float3 average_center = (top_left_pt + top_right_pt + bottom_left_pt + bottom_right_pt) / 4;

			float3 local_z_axis = Crossed(top_right_pt - top_left_pt, top_left_pt - bottom_left_pt) * -1;
			Normalize(local_z_axis);
			float3 world_axis = make_float3(0, 0, 1);
			float3 to_local_z = VectorRotationZ(local_z_axis,world_axis);

			float deformation = DeltaDepth(world_coordinates, average_center, to_local_z, x, y, half_window_width);
			if (deformation > threshold) {
				detected_deformations(y, x) = half_window_width;
			}
			else {
				detected_deformations(y, x) = 0;
			}
		}
		else {
			detected_deformations(y, x) = 0;
		}
	}

	__host__ void AcceleratedDeformationsDetection(const GpuMat world_coordinates, const float threshold,
	const int window_width, GpuMat detected_deformations, cv::cuda::Stream& s) {
		cudaStream_t stream = cv::cuda::StreamAccessor::getStream(s);

		int half_window_width;
		if (window_width % 2 == 0)
			half_window_width = window_width / 2;
		else
			half_window_width = (window_width - 1) / 2;

		dim3 block(32, 8);
		dim3 grid((world_coordinates.cols + block.x - 1) / block.x, (world_coordinates.rows + block.y - 1) / block.y);
		ComputeDeformations <<<grid, block, 0, stream>>>(world_coordinates,threshold,half_window_width,detected_deformations);

		if (stream == 0)
			cudaDeviceSynchronize();
	}

}
}