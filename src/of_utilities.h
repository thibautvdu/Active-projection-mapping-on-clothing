#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_

#include "ofGraphics.h"
#include "ofPolyline.h"
#include "ofPath.h"

namespace of_utilities {

	/*
	A window simulation when using a fullscreen window covering both screens
	*/
	class VirtualWindow {
		public:
			inline VirtualWindow() : x_position_(0), y_position_(0), width_(0), height_(0) {}
			inline VirtualWindow(const int x, const int y, const int w, const int h) : x_position_(x), y_position_(y), width_(w), height_(h) {}

			inline int x() const { return x_position_; }
			inline int y() const { return y_position_; }
			inline int width() const { return width_; }
			inline int height() const { return height_; }

			void begin() const;
			void end() const;

			void background(const ofColor color) const;

		private:
			int x_position_, y_position_;
			int width_, height_;
	};

	ofVec2f MapVec2f(const ofVec2f value, const ofVec2f input_min, const ofVec2f input_max, const ofVec2f output_min, const ofVec2f output_max, const bool clamp = false);	// Map function for ofVec2f
	ofPath PolylineToPath(const ofPolyline polyline);

} // namespace ofUtilities

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_