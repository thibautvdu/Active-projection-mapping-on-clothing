#ifndef FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_

#include "ofFastMesh.h"
#include "kinect3dBlobDetector.h"

namespace garmentAugmentation {

	class interactiveGarment {
		public :
			interactiveGarment() {
			}

			inline ofFastMesh& getMeshRef() { return mesh_; }
			inline ofPolyline& getContour2dRef() { return contour2d_; }
			inline const std::vector< std::vector<int> > &getMesh2dViewRef() const { return mesh2dView_; };

			void update(const kinect3dBlobDetector &detector, const simple3dBlob blob);
			inline void drawMesh() { mesh_.drawVertices(); }

		private :
			simple3dBlob blob_;
			ofFastMesh mesh_;
			std::vector< std::vector<int> > mesh2dView_;
			ofPolyline contour2d_;
	};
};

#endif // !FLEXIBLE_SURFACE_AUGMENTATION_INTERACTIVE_GARMENT_H_