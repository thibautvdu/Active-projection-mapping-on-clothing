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
			inline ofVirtualWindow(const int x, const int y, const int w, const int h) : xPosition_(x), yPosition_(y), width_(w), height_(h) {}

			inline int getX() const { return xPosition_; }
			inline int getY() const { return yPosition_; }
			inline int getWidth() const { return width_; }
			inline int getHeight() const { return height_; }

			void begin() const;
			void end() const;

			void background(const ofColor color) const;

		private:
			int xPosition_, yPosition_;
			int width_, height_;
	};

	ofVec2f mapVec2f(const ofVec2f value, const ofVec2f inputMin, const ofVec2f inputMax, const ofVec2f outputMin, const ofVec2f outputMax, const bool clamp = false);	// Map function for ofVec2f
	ofPath polylineToPath(const ofPolyline polyline);

} // namespace ofUtilities

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_