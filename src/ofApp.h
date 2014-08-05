#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinectCommonBridge.h"

class ofApp : public ofBaseApp {

	public:
		void setup();
		void update();
		void draw();
		void exit();

		ofxKinectCommonBridge mOfxKinect;
		KCBHANDLE mKcbKinect;
		ofImage mOfSegmentedImage;
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_APP_H_
