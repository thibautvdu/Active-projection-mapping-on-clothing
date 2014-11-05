#ifndef FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_

#include "ofFastMesh.h"
#include "kinect3dBlobDetector.h"
#include "fold.h"
#include "animation.h"

namespace garment_augmentation {
namespace garment {

	class InteractiveGarment {
		public :
			inline ofFastMesh &mesh_ref() { 
				return mesh_; 
			}

			inline const std::vector< std::vector<int> > &mesh2d_view() const { 
				return mesh2d_view_;
			}

			inline const Simple3dblob &blob() const { 
				return blob_; 
			}

			void Update(const blob_detection::kinect3dBlobDetector &detector, const Simple3dblob blob);

			void UpdateFolds(std::vector<Fold> &folds);

			inline void DrawMesh() { 
				ofPushMatrix();
					ofTranslate(blob_.massCenter);
					mesh_.drawVertices(); 
				ofPopMatrix();
			}

			inline void DrawFolds() {
				ofPushMatrix();
				ofTranslate(blob_.massCenter);
				for (int i = 0; i < folds_.size(); ++i) {
					folds_[i].Draw();
				}
				ofPopMatrix();
			}

			inline void AddAnimation(std::unique_ptr<Animation> anim) {
				animations_.push_back(std::move(anim));
			}

			inline void UpdateAnimations() {
				for (int i = 0; i < animations_.size(); ++i) {
					animations_[i]->Update();
					animations_[i]->Update(folds_);
					animations_[i]->Update(contour_);
				}
			}

			inline void DrawAnimations() {
					for (int i = 0; i < animations_.size(); ++i) {
						ofPushMatrix();
						
						if (animations_[i]->RelativeToModel())
							ofTranslate(blob_.massCenter);
						
						animations_[i]->Draw();

						ofPopMatrix();
					}
			}

		private :
			template <typename P_TYPE_A, typename P_TYPE_B>
			static bool ComparePairSecond(const std::pair<P_TYPE_A, P_TYPE_B> a, const std::pair<P_TYPE_A, P_TYPE_B> b);

			static bool CompareFoldsConfigurations(const std::pair<std::vector<int>, float> a, const std::pair<std::vector<int>, float> b);
			void FoldsPermutation(const std::vector<Fold> &new_folds, std::pair<std::vector<int>,float> &current_configuration, const float threshold, int new_folds_idx);

			Simple3dblob blob_;
			ofFastMesh mesh_;
			std::vector<ofVec3f> contour_;
			std::vector< std::vector<int> > mesh2d_view_;

			std::vector<Fold> folds_;
			std::vector <std::pair<std::vector<int>,float>> folds_tracking_permutations_;
			std::vector<std::unique_ptr<Animation>> animations_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_