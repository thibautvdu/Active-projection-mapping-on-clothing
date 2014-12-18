// A few utilities to fill the gaps (missing features) of openframeworks

#ifndef FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_

#include "ofMain.h"

namespace of_utilities {

	// Simulate a two-windows behavior when using a large window which cover both monitor and projector
	// this is necessary as openframeworks doesn't handle multiple windows with the programmable renderer
	// yet
	class VirtualWindow {
		public:
			inline VirtualWindow() : x_position_(0), y_position_(0), width_(0), height_(0) {}
			inline VirtualWindow(const int x, const int y, const int w, const int h) : x_position_(x), y_position_(y), width_(w), height_(h) {}

			inline int x() const { return x_position_; }
			inline int y() const { return y_position_; }
			inline int width() const { return width_; }
			inline int height() const { return height_; }

			void Begin() const;
			void End() const;

			void background(const ofColor color) const;

		private:
			int x_position_, y_position_;
			int width_, height_;
	};

	// Return the rotation matrix from the vector 'from' to the vector 'to'
	ofMatrix3x3 VectorRotationMatrix(ofVec3f from, ofVec3f to);

	// Equivalent of the float mapping function of openframeworks but with 2D vectors
	ofVec2f MapVec2f(const ofVec2f value, const ofVec2f input_min, const ofVec2f input_max, const ofVec2f output_min, const ofVec2f output_max, const bool clamp = false);

	// Convert a polyline object to a path object
	ofPath PolylineToPath(const ofPolyline polyline);

} // namespace ofUtilities

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_