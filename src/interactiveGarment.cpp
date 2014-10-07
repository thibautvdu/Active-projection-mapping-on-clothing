#include "interactiveGarment.h"

namespace garmentAugmentation {

	void interactiveGarment::update(const kinect3dBlobDetector &detector, const simple3dBlob blob) {
		blob_ = blob;

		int pcW = detector.getWidth();
		int pcH = detector.getHeight();

		if (mesh_.getCapacity() != pcW*pcH) {
			mesh_.reserveCapacity(pcW*pcH);
			mesh2dView_.resize(pcW, vector<int>(pcH, -1));
		}

		mesh_.resize(blob_.nbPoints);

		int vIdx = 0;
		for (int y = 0; y < pcH; y++){
			for (int x = 0; x < pcW; ++x) {
				if (detector[y*pcW + x].flag == blob_.idx) {
					mesh_.setVertex(vIdx, detector[y*pcW + x].pos);
					mesh_.setColor(vIdx, ofColor::white);
					mesh2dView_[x][y] = vIdx;
					vIdx++;
				}
			}
		}

		// Construct the 2d contour
		contour2d_.resize(blob_.contourIndices2d.size());
		for (int i = 0; i < blob_.contourIndices2d.size(); ++i) {
			contour2d_[i] = detector[i].pos;
		}
		contour2d_.close();
	}
};