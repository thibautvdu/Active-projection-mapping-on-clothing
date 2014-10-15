#ifndef FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_

#include "ofFastMesh.h"
#include "kinect3dBlobDetector.h"
#include "fold.h"
#include "animation.h"

namespace garmentAugmentation {
namespace garment {

	class interactiveGarment {
		public :
			inline ofFastMesh &getMeshRef() { 
				return mesh_; 
			}

			inline ofPolyline &getContour2dRef() { 
				return contour2d_; 
			}

			inline const std::vector< std::vector<int> > &getMesh2dViewRef() const { 
				return mesh2dView_;
			}

			inline const simple3dBlob &getBlobRef() const { 
				return blob_; 
			}

			inline std::vector<fold> &getFoldsRef() { 
				return folds_; 
			}

			void update(const blobDetection::kinect3dBlobDetector &detector, const simple3dBlob blob);

			inline void drawMesh() { 
				mesh_.drawVertices(); 
			}

			inline void addFold(const fold f) {
				folds_.push_back(f);
			}

			inline void addAnimation(std::unique_ptr<animation> anim) {
				animations_.push_back(std::move(anim));
			}

			inline void updateAnimations() {
				for (int i = 0; i < animations_.size(); ++i) {
					animations_[i]->update(folds_);
				}
			}

			inline void drawAnimations() {
				for (int i = 0; i < animations_.size(); ++i) {
					animations_[i]->draw();
				}
			}

		private :
			simple3dBlob blob_;
			ofFastMesh mesh_;
			std::vector< std::vector<int> > mesh2dView_;
			ofPolyline contour2d_;

			std::vector<fold> folds_;
			std::vector<std::unique_ptr<animation>> animations_;
	};

} // namespace garment
} // namespace garmentAugmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_