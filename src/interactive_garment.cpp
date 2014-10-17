#include "interactive_garment.h"

namespace garment_augmentation {
namespace garment {

	void InteractiveGarment::Update(const blob_detection::kinect3dBlobDetector &detector, const Simple3dblob blob) {
		blob_ = blob;

		int pcW = detector.getWidth();
		int pcH = detector.getHeight();

		if (mesh_.getCapacity() != pcW*pcH) {
			mesh_.reserveCapacity(pcW*pcH);
			mesh2d_view_.resize(pcW, vector<int>(pcH));
		}

		mesh_.resize(blob_.nbPoints);

		int vIdx = 0;
		for (int y = 0; y < pcH; y++){
			for (int x = 0; x < pcW; ++x) {
				if (detector.getCloudPoint(y*pcW + x).flag_ == blob_.idx) {
					mesh_.setVertex(vIdx, detector.getCloudPoint(y*pcW + x).pos_);
					mesh_.setColor(vIdx, ofColor::white);
					mesh2d_view_[x][y] = vIdx;
					vIdx++;
				}
				else
					mesh2d_view_[x][y] = -1;
			}
		}

		// Construct the 2d contour
		contour2d_.resize(blob_.contour2d_indices.size());
		for (int i = 0; i < blob_.contour2d_indices.size(); ++i) {
			contour2d_[i] = detector.getCloudPoint(i).pos_;
		}
		contour2d_.close();
	}

} // namespace garment
} // namespace garment_augmentation