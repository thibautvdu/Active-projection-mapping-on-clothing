#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_FAST_3D_BLOB_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_FAST_3D_BLOB_H_

#include "ofMain.h"
#include "ofxKinectCommonBridge.h"

enum ofFastBlobResolution { BF_HIGH_RES = 1, BF_MEDIUM_RES = 2, BF_LOW_RES = 4 };
enum cloudPointFlag { FLAG_OFF_THRESHOLD = -5, FLAG_BACKGROUND = -4, FLAG_IDLE = -3, FLAG_QUEUED = -2, FLAG_PROCESSED = -1 };

class cloudPoint {
	public :
		cloudPoint() {
			boundary = false;
		}

		int flag;
		bool boundary;
		ofVec3f pos;
};

class ofFast3dBlob {
	public :
		int idx;
		vector<int> contourIndices;
		ofVec3f centroid;
		ofVec3f minX, minY, minZ; // points with minimum x / y / z
		ofVec3f maxX, maxY, maxZ; // points with maximum x / y / z
		ofVec3f boundingBoxMax, boundingBoxMin; // min bounding xyz
		ofVec3f dimensions; //dimensions
		ofVec3f massCenter;
		float volume; // volume
};

class ofFast3dBlobDetector {

	public:
		ofFast3dBlobDetector();
		~ofFast3dBlobDetector();
		void init(ofxKinectCommonBridge *newKinect);
		bool isInited();
		void setRotation(const ofVec3f newRotation);
		void setTranslation(const ofVec3f newTranslation);
		void setScale(const ofVec3f newScale);
		bool setResolution(enum ofFastBlobResolution newResolution);
		int getWidth() const { return width; }
		int getHeight() const { return height; }
		const cloudPoint& operator[](int idx) const {
			return m_pointCloud[idx];
		}

		ofVec3f getScale() const;
		enum ofFastBlobResolution getResolution();

		bool findBlobs(ofImage * maskImage,
			const ofVec3f thresh3D, const int thresh2D,
			const float minVol, const float maxVol,
			const int minPoints, const unsigned int maxBlobs);
		int nBlobs;
		std::vector<ofFast3dBlob*> blobs;

	protected:

		ofxKinectCommonBridge * kinectPtr;
		ofVec3f scale;
		int width;
		int height;
		enum ofFastBlobResolution resolution;
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



#endif // !FLEXIBLE_SURFACE_AUGMENTATION_OF_FAST_3D_BLOB_H_