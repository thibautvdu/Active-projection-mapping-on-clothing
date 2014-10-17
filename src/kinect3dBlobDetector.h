#ifndef FLEXIBLE_SURFACE_AUGMENTATION_KINECT_3D_BLOB_DETECTOR_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_KINECT_3D_BLOB_DETECTOR_H_

#include "ofMain.h"

#include "ofxKinectCommonBridge.h"
#include "simple_3dblob.h"
#include "point_cloud.h"

namespace garment_augmentation {
namespace blob_detection {

	enum kinectBlobDetectorResolution { BF_HIGH_RES = 1, BF_MEDIUM_RES = 2, BF_LOW_RES = 4 };

	class kinect3dBlobDetector {
		public:
			kinect3dBlobDetector();

			~kinect3dBlobDetector();

			void init(const ofxKinectCommonBridge *newKinect, const int width, const int height);

			bool isInited() const;

			enum kinectBlobDetectorResolution getResolution();

			bool findBlobs(ofImage *maskImage,
				const ofVec3f thresh3D, const int thresh2D,
				const float minVol, const float maxVol,
				const int minPoints, const unsigned int maxBlobs);

			ofVec3f getScale() const;

			void setRotation(const ofVec3f newRotation);

			void setTranslation(const ofVec3f newTranslation);

			void setScale(const ofVec3f newScale);

			bool setResolution(const enum kinectBlobDetectorResolution newResolution);

			int getWidth() const { 
				return width; 
			}

			int getHeight() const { 
				return height; 
			}

			inline const CloudPoint& getCloudPoint(const int idx) const {
				return m_pointCloud[idx];
			}

			int nBlobs;
			std::vector<Simple3dblob * > blobs;

		private:
			const static int clockwiseBacktracking[];
			const static int clockwiseX[];
			const static int clockwiseY[];


			bool createCloud(unsigned char * maskPix);

			const ofxKinectCommonBridge *kinectPtr;
			ofVec3f scale;
			int width;
			int height;
			enum kinectBlobDetectorResolution resolution;
			int kWidth;
			int kHeight;
			int kNPix;
			int nPix;

			CloudPoint*  m_pointCloud;

			bool bFinderInited;
			bool bAllocatedMap;

			ofVec3f nullPoint; // xyz value considered invalid

	};

} // namespace blob_detection
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_KINECT_3D_BLOB_DETECTOR_H_