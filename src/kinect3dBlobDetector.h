#ifndef FLEXIBLE_SURFACE_AUGMENTATION_KINECT_3D_BLOB_DETECTOR_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_KINECT_3D_BLOB_DETECTOR_H_

#include "ofMain.h"

#include "ofxKinectCommonBridge.h"
#include "simple3dBlob.h"
#include "pointCloud.h"

namespace garmentAugmentation {
namespace blobDetection {

	enum kinectBlobDetectorResolution { BF_HIGH_RES = 1, BF_MEDIUM_RES = 2, BF_LOW_RES = 4 };

	class kinect3dBlobDetector {
		public:
			kinect3dBlobDetector();
			~kinect3dBlobDetector();
			void init(ofxKinectCommonBridge *newKinect);
			bool isInited();
			void setRotation(const ofVec3f newRotation);
			void setTranslation(const ofVec3f newTranslation);
			void setScale(const ofVec3f newScale);
			bool setResolution(enum kinectBlobDetectorResolution newResolution);
			int getWidth() const { return width; }
			int getHeight() const { return height; }
			const cloudPoint& operator[](int idx) const {
				return m_pointCloud[idx];
			}

			ofVec3f getScale() const;
			enum kinectBlobDetectorResolution getResolution();

			bool findBlobs(ofImage * maskImage,
				const ofVec3f thresh3D, const int thresh2D,
				const float minVol, const float maxVol,
				const int minPoints, const unsigned int maxBlobs);
			int nBlobs;
			std::vector<simple3dBlob * > blobs;

		protected:

			ofxKinectCommonBridge * kinectPtr;
			ofVec3f scale;
			int width;
			int height;
			enum kinectBlobDetectorResolution resolution;
			int kWidth;
			int kHeight;
			int kNPix;
			int nPix;

			cloudPoint*  m_pointCloud;

			bool bFinderInited;
			bool bAllocatedMap;

			ofVec3f nullPoint; // xyz value considered invalid

			bool createCloud(unsigned char * maskPix);
	};

} // namespace blobDetection
} // namespace garmentAugmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_KINECT_3D_BLOB_DETECTOR_H_