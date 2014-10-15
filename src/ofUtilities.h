#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_

#include "ofGraphics.h"
#include "ofPolyline.h"
#include "ofPath.h"

namespace ofUtilities {

	/*
	A window simulation when using a fullscreen window covering both screens
	*/
	class ofVirtualWindow {
		public:
			inline ofVirtualWindow() : xPosition_(0), yPosition_(0), width_(0), height_(0) {}
			inline ofVirtualWindow(int x, int y, int w, int h) : xPosition_(x), yPosition_(y), width_(w), height_(h) {}

			int getX() const { return xPosition_; }
			int getY() const { return yPosition_; }
			int getWidth() const { return width_; }
			int getHeight() const { return height_; }

			void begin() const;
			void end() const;

			void background(ofColor color) const;

		private:
			int xPosition_, yPosition_;
			int width_, height_;
	};

	ofVec2f mapVec2f(ofVec2f value, ofVec2f inputMin, ofVec2f inputMax, ofVec2f outputMin, ofVec2f outputMax, bool clamp = false);	// Map function for ofVec2f
	ofPath polylineToPath(ofPolyline polyline);

} // namespace ofUtilities

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_