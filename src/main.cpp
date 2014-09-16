#include "ofMain.h"
#include "ofApp.h"
#include "ofAppGLFWWindow.h"

//========================================================================
int main() {

	ofSetCurrentRenderer(ofGLProgrammableRenderer::TYPE);
	ofSetupOpenGL(1200, 800, OF_WINDOW);	// setup the GL context

	// Cover all the screens with one fullscreen window
	ofAppBaseWindow* window = ofGetWindowPtr();
	ofAppGLFWWindow* GLFWWindow = (ofAppGLFWWindow*)window;

	GLFWWindow->setMultiDisplayFullscreen(true);
	window->setFullscreen(true);

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

}
