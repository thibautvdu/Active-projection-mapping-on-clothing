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
		ofVirtualWindow() : m_xPosition(0), m_yPosition(0), m_width(0), m_height(0) {}
		ofVirtualWindow(int x, int y, int w, int h) : m_xPosition(x), m_yPosition(y), m_width(w), m_height(h) {}

		int getX() const { return m_xPosition; }
		int getY() const { return m_yPosition; }
		int getWidth() const { return m_width; }
		int getHeight() const { return m_height; }

		void begin();
		void end();

		void background(ofColor color);

	private:
		int m_xPosition, m_yPosition;
		int m_width, m_height;

	};

	ofVec2f mapVec2f(ofVec2f value, ofVec2f inputMin, ofVec2f inputMax, ofVec2f outputMin, ofVec2f outputMax, bool clamp = false);	// Map function for ofVec2f
	ofPath polylineToPath(ofPolyline polyline);
}

#endif // FLEXIBLE_SURFACE_AUGMENTATION_OF_UTILITIES_H_