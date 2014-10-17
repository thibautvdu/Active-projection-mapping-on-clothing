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

			inline ofPolyline &contour2d_ref() { 
				return contour2d_; 
			}

			inline const std::vector< std::vector<int> > &mesh2d_view() const { 
				return mesh2d_view_;
			}

			inline const Simple3dblob &blob() const { 
				return blob_; 
			}

			inline std::vector<Fold> &folds_ref() { 
				return folds_; 
			}

			void Update(const blob_detection::kinect3dBlobDetector &detector, const Simple3dblob blob);

			inline void DrawMesh() { 
				mesh_.drawVertices(); 
			}

			inline void AddFold(const Fold f) {
				folds_.push_back(f);
			}

			inline void AddAnimation(std::unique_ptr<Animation> anim) {
				animations_.push_back(std::move(anim));
			}

			inline void UpdateAnimations() {
				for (int i = 0; i < animations_.size(); ++i) {
					animations_[i]->Update(folds_);
				}
			}

			inline void DrawAnimations() {
				for (int i = 0; i < animations_.size(); ++i) {
					animations_[i]->Draw();
				}
			}

		private :
			Simple3dblob blob_;
			ofFastMesh mesh_;
			std::vector< std::vector<int> > mesh2d_view_;
			ofPolyline contour2d_;

			std::vector<Fold> folds_;
			std::vector<std::unique_ptr<Animation>> animations_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_