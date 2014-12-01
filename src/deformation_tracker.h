#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_

#include "interactive_garment.h"

namespace garment_augmentation {
namespace garment {

	class DeformationTracker {
		public:
			DeformationTracker(InteractiveGarment *garment, const ofRectangle roi) : p_garment_(garment), roi_(roi) {
				need_computation_ = true;
				//patch_depth_map_.resize(static_cast<int>(roi_.width), std::vector<float>(static_cast<int>(roi_.height),std::numeric_limits<float>::infinity()));
			}


			inline void Move(const int x, const int y) {
				roi_.translate(x, y);
				need_computation_ = true;
			}

			inline void MoveLeftTopTo(const int x, const int y) {
				roi_.setPosition(x, y);
				need_computation_ = true;
			}

			inline void MoveCenterTo(const int x, const int y) {
				roi_.set(x - (roi_.width - 1) / 2, y - (roi_.height - 1) / 2, roi_.width, roi_.height);
			}

			inline void Resize(const int w, const int h) {
				roi_.setWidth(w);
				roi_.setHeight(h);
				//patch_depth_map_ = std::vector<std::vector<float>>(static_cast<int>(roi_.width), std::vector<float>(static_cast<int>(roi_.height), std::numeric_limits<float>::infinity()));
				need_computation_ = true;
			}

			inline int GetLeftTopX() const {
				return roi_.x;
			}

			inline int GetLeftTopY() const {
				return roi_.y;
			}

			void ShapeOn(const int x, const int y);
			void HalfTubeShaping(float depth);

			inline float GetFoldness() {
				if (need_computation_)
					ComputeDeltaDepth();

				return delta_depth_;
			}

			inline float GetDissimilarityWithPatch() {
				if (need_computation_)
					ComputeDissimilarity();

				return dissimilarity_;
			}

			inline float GetFoldnessGaussian() {
				if (need_computation_) {
					ComputeGaussianDist(1);
					ComputeDeltaDepthGaussian();
				}

				return delta_depth_gaussian_;
			}

			std::vector<ofVec3f> GetPoints();
			ofVec3f GetCenter();
			void ColorFill(const ofColor color);
			void WritePatchLocation(const int index);

			bool IsInsideMesh();

		private:
			void ComputeDeltaDepth();
			void ComputeDissimilarity();
			void ComputeDeltaDepthGaussian();
			static void ComputeGaussianDist(float sigma);

			std::vector<std::vector<float>> patch_depth_map_;
			InteractiveGarment *const p_garment_;
			ofRectangle roi_;
			bool need_computation_;
			float delta_depth_;
			float dissimilarity_;
			float delta_depth_gaussian_;
			static std::vector<float> gaussian_values_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_TRACKER_H_