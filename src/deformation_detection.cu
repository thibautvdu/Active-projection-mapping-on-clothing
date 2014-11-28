#include "deformation_detection.h"

#include <opencv2/cudev/common.hpp>
#include <stdio.h>

using namespace cv::cuda;

namespace garment_augmentation {
namespace cuda_optimization {

	const float k_float_zero = 0.0001; // in meters


	// Vector helpers
	__device__ float3 operator/(const float3 &a, float scal) {
		return make_float3(a.x / scal, a.y / scal, a.z / scal);
	}

	__device__ float3 operator*(const float3 &a, float scal) {
		return make_float3(a.x * scal, a.y * scal, a.z * scal);
	}

	__device__ float3 operator-(const float3 &a, const float3 &b) {
		return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	__device__ float3 Crossed(const float3 &a, const float3 &b) {
		return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
	}

	__device__ float Dot(const float3 &a, const float3 &b) {
		return a.x*b.x + a.y*b.y + a.z*b.z;
	}

	__device__ float Norm(const float3 &a) {
		return a.x + a.y + a.z;
	}

	__device__ void Normalize(float3 &a) {
		float norm = Norm(a);
		a = a / norm;
	}

	__device__ float3 VectorRotationZ(float3 from, float3 to) {
		float3 crossed = Crossed(from, to);
		float dot = Dot(from,to);
		float crossed_norm = Norm(crossed);

		float3 third_base = (to - from*dot);
		Normalize(third_base);

		// to local space transformation matrix
		float lstm_a = from.x;
		float lstm_b = third_base.x;
		float lstm_c = -crossed.x;
		float lstm_d = from.y;
		float lstm_e = third_base.y,
		float lstm_f = -crossed.y;
		float lstm_g = from.z;
		float lstm_h = third_base.z,
		float lstm_i = -crossed.z;

		float determinant_local_space_matrix = 
			lstm_a * lstm_e * lstm_i
			+ lstm_b * lstm_f * lstm_g
			+ lstm_d * lstm_h * lstm_c
			- lstm_g * lstm_e * lstm_c
			- lstm_d * lstm_b * lstm_i
			- lstm_h * lstm_f * lstm_a;

		float local_space_invert_a = (lstm_e * lstm_i - lstm_h * lstm_f) / determinant_local_space_matrix;
		float local_space_invert_b = (-lstm_b * lstm_i + lstm_h * lstm_c) / determinant_local_space_matrix;
		float local_space_invert_c = (lstm_b * lstm_f - lstm_e * lstm_c) / determinant_local_space_matrix;
		float local_space_invert_d = (-lstm_d * lstm_i + lstm_g * lstm_f) / determinant_local_space_matrix;
		float local_space_invert_e = (lstm_a * lstm_i - lstm_g * lstm_c) / determinant_local_space_matrix;
		float local_space_invert_f = (-lstm_a * lstm_f + lstm_d * lstm_c) / determinant_local_space_matrix;
		float local_space_invert_g = (lstm_d * lstm_h - lstm_g * lstm_e) / determinant_local_space_matrix;
		float local_space_invert_h = (-lstm_a * lstm_h + lstm_g * lstm_b) / determinant_local_space_matrix;
		float local_space_invert_i = (lstm_a * lstm_e - lstm_d * lstm_b) / determinant_local_space_matrix;

		float3 transform_on_z;
		transform_on_z.x = 
			from.z*dot*local_space_invert_a + from.z*-crossed_norm*local_space_invert_c + from.z*0.f*local_space_invert_g +
			third_base.z*crossed_norm*local_space_invert_a + third_base.z*dot*local_space_invert_c + third_base.z*0.f*local_space_invert_g +
			-crossed.z*0.f*local_space_invert_a + -crossed.z*0.f*local_space_invert_c + -crossed.z*1.f*local_space_invert_g;
		transform_on_z.y =
			from.z*dot*local_space_invert_b + from.z*-crossed_norm*local_space_invert_e + from.z*0.f*local_space_invert_h +
			third_base.z*crossed_norm*local_space_invert_b + third_base.z*dot*local_space_invert_e + third_base.z*0.f*local_space_invert_h +
			-crossed.z*0.f*local_space_invert_b + -crossed.z*0.f*local_space_invert_e + -crossed.z*1.f*local_space_invert_h;
		transform_on_z.z =
			from.z*dot*local_space_invert_d + from.z*-crossed_norm*local_space_invert_f + from.z*0.f*local_space_invert_i +
			third_base.z*crossed_norm*local_space_invert_d + third_base.z*dot*local_space_invert_f + third_base.z*0.f*local_space_invert_i +
			-crossed.z*0.f*local_space_invert_d + -crossed.z*0.f*local_space_invert_f + -crossed.z*1.f*local_space_invert_i;

		return transform_on_z;
	}

	// detected_deformations contain 0 if the deformation value of the area < threshold or the size of the area if >= threshold
	// half_window_width should be equal to floor(window_width/2)
	__global__ void ComputeDeformations(const PtrStepSz<float3> world_coordinates, const float threshold,
		const int half_window_width, PtrStepSz<uchar> detected_deformations) {
		int x = threadIdx.x + blockIdx.x * blockDim.x;
		int y = threadIdx.y + blockIdx.y * blockDim.y;

		if (x < world_coordinates.cols - half_window_width && y < world_coordinates.rows - half_window_width && x >= half_window_width && y >= half_window_width)
		{
			float3 top_left_pt = world_coordinates(y - half_window_width, x - half_window_width);
			if (top_left_pt.z < k_float_zero) return;
			float3 top_right_pt = world_coordinates(y - half_window_width, x + half_window_width);
			if (top_right_pt.z < k_float_zero) return;
			float3 bottom_left_pt = world_coordinates(y + half_window_width, x - half_window_width);
			if (bottom_left_pt.z < k_float_zero) return;
			float3 bottom_right_pt = world_coordinates(y + half_window_width, x + half_window_width);
			if (bottom_right_pt.z < k_float_zero) return;
			
			float3 local_z_axis = Crossed(top_right_pt - top_left_pt, top_left_pt - bottom_left_pt);
			float3 world_axis = make_float3(0, 0, 0);
			
			float3 to_local_z = VectorRotationZ(world_axis, local_z_axis);

			float deformation = DeltaDepth(world_coordinates, to_local_z, x, y, half_window_width);
			if (deformation > threshold) {
				detected_deformations(y, x) = half_window_width;
			}
		}
	}

	__device__ float DeltaDepth(const PtrStepSz<float3> world_coordinates, const float3 transformation, int x, int y, int half_width) {
		float delta_depth;
		int nb_points = 0;

		for (int row = y - half_width; row < y + half_width; row++) {
			for (int col = y - half_width; col < x + half_width; col++) {
				if (world_coordinates(row,col).z > k_float_zero) {
					float3 point = world_coordinates(row, col) - world_coordinates(y, x); // Relative to the middle of the patch
					delta_depth += point.x * transformation.x + point.y * transformation.y + point.z * transformation.z; // Rotation to the normal of the patch
					nb_points++;
				}
			}
		}

		return delta_depth / nb_points;
	}

	__host__ void DetectDeformations(const PtrStepSz<float3> world_coordinates, const float threshold,
		const int half_window_width, cudaStream_t stream, PtrStepSz<uchar> detected_deformations) {
		dim3 block(32, 8);
		dim3 grid((world_coordinates.cols + block.x - 1) / block.x, (world_coordinates.rows + block.y - 1) / block.y);
		ComputeDeformations <<<grid, block, 0, stream>>>(world_coordinates,threshold,half_window_width,detected_deformations);

		if (stream == 0)
			cudaDeviceSynchronize();
	}

}
}