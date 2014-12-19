// This is the main function where the application is launched after the creation of
// an openGL context

#include "ofMain.h"
#include "ofApp.h"
#include "ofAppGLFWWindow.h"

int main() {

	ofSetCurrentRenderer(ofGLProgrammableRenderer::TYPE);
	ofSetupOpenGL(1200, 800, OF_WINDOW);

	// Fullscreen
	// Cover both the monitor and projector screen with a single large window
	// (multi window is not possible in openframeworks yet while using the
	// programmable renderer)
	//ofAppBaseWindow* window = ofGetWindowPtr();
	//ofAppGLFWWindow* GLFWWindow = (ofAppGLFWWindow*)window;

	//GLFWWindow->setMultiDisplayFullscreen(true);
	//window->setFullscreen(true);
	// !Fullscreen

	ofRunApp(new ofApp());

}
