#include "ofApp.h"

#include <stdlib.h>
#include "cvHelper.h"
#include "lscm.h"

/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::setup() {
	// HARDWARE INIT	/	/	/	/	/	/	/	/	/	/	/	/

	// Kinect
	m_kinectWidth = 640;
	m_kinectHeight = 480;

	bool initSensor = m_ofxKinect.initSensor();
	if (!initSensor) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's sensor";
		exit();
	}

	bool initStreams = m_ofxKinect.initColorStream(m_kinectWidth, m_kinectHeight) && m_ofxKinect.initDepthStream(m_kinectWidth, m_kinectHeight, false, true);
	if (!initStreams) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's color and/or depth streams";
		exit();
	}

	bool kinectStarted = m_ofxKinect.start();
	if (!kinectStarted) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't start the kinect";
		exit();
	}

	// Projector
	m_projectorWindow = ofUtilities::ofVirtualWindow(1920, 0, 1024, 768);
	m_kinectProjectorToolkit.loadCalibration("cal.xml", m_projectorWindow.getWidth(), m_projectorWindow.getHeight());

	// HARDWARE INIT	-	-	-	-	-	-	-	-	-	-	-	-


	// OPEN GL	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	ofDisableAlphaBlending();
	ofSetFrameRate(30);
	m_shader.load("shaders/toProjector.vert", "shaders/toProjector.frag");

	// OPEN GL	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// KINECT SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/	/

	// Background learning
	m_bgLearningCycleCount = 0;
	m_askBgLearning = false;
	m_learntBg = false;

	// Background segmentation
	m_bgMask.allocate(m_kinectWidth,m_kinectHeight,OF_IMAGE_GRAYSCALE);
	m_cvBgMask = ofxCv::toCv(m_bgMask);

	// Blob detection
	m_blobFound = false;

	// Garment segmentation
	m_garmentSeg.allocate(m_kinectWidth, m_kinectHeight, OF_IMAGE_GRAYSCALE);
	m_cvGarmentSeg = ofxCv::toCv(m_garmentSeg);

	// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

	// Blob finder and tracker
	m_blobFinder.init(&m_ofxKinect, false); // standarized coordinate system: z in the direction of gravity
	m_blobFinder.setResolution(BF_MEDIUM_RES);
	m_blobFinder.setRotation(ofVec3f(0, 0, 0));
	m_blobFinder.setTranslation(ofVec3f(0, 0, 0));
	m_blobFinder.setScale(ofVec3f(0.001,0.001,0.001)); // mm to meters

	// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

	m_chessboardImage.loadImage("chessboard.png");

	// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


	// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	m_gui.setup();
	m_gui.add(m_bgLearningCycleGui.setup("bg learning iterations", 10, 1, 50));
	m_gui.add(m_nearClipGui.setup("near clip",1.500,0,8.000));
	m_gui.add(m_farClipGui.setup("far clip", 3.000, 0, 8.000));
	m_gui.add(m_cannyThresh1.setup("canny thres 1", 0, 0, 255));
	m_gui.add(m_cannyThresh2.setup("canny thres 2", 0, 0, 255));

	// Keys
	m_askPause = false;
	m_askSaveAssets = false;
	m_askBgExport = false;

	// FPS
	m_gui.add(m_fpsGui.setup(std::string("fps :")));

	// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
}

void ofApp::update() {
	// EVENTS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	if (m_askSaveAssets) {
		m_modelBlob->mesh.save("mesh_export.ply");
		ofSaveImage(m_ofxKinect.getColorPixelsRef(), "color_ir_export.tif");
		ofSaveImage(m_ofxKinect.getDepthPixelsRef(), "depth_export.tif");
		ofLogNotice("ofApp::update") << "Exported the 3D mesh, IR/color and depth image";
		m_askSaveAssets = false;
	}
	if (m_askBgExport) {
		if (m_learntBg) {
			ofSaveImage(m_depthBg, "background_export.tif");
			ofLogNotice("ofApp::update") << "Exported the learnt background";
		}
		else {
			ofLogNotice("ofApp::update") << "Can't export the background, press b to learn background";
		}
		m_askBgExport = false;
	}
	if (m_askPause)
		return;

	// EVENTS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// HARDWARE	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	m_ofxKinect.update();

	// HARDWARE	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// PROCESS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/
	if (m_ofxKinect.isFrameNew()) {
		// Background learning
		/*
		if (m_askBgLearning) {
			m_bgLearningCycleCount = m_bgLearningCycleGui;
			m_learntBg = false;
			m_askBgLearning = false;

			ofLogNotice("ofApp::update()") << "Learning background . . .";
			m_depthBg = mOfxKinect.getDepthPixels();
			m_cvDepthBg = ofxCv::toCv(m_depthBg);
			--m_bgLearningCycleCount;

			if (m_bgLearningCycleCount == 0) {
				m_learntBg = true;
				ofLogNotice("ofApp::update()") << "Background learning complete";
			}
		}
		else if (m_bgLearningCycleCount != 0) {
			for (int i = 0; i < m_depthBg.getWidth()*m_depthBg.getHeight(); ++i) {
				if (0 == m_depthBg[i]) {
					m_depthBg[i] = mOfxKinect.getDepthPixelsRef()[i];
				}
			}

			--m_bgLearningCycleCount;
			if (m_bgLearningCycleCount == 0) {
				m_learntBg = true;
				ofLogNotice("ofApp::update()") << "Background learning complete";
			}
		}
		*/
		if (!m_learntBg) {
			bool importedPixels = ofLoadImage(m_depthBg, "background_export.tif");
			if (importedPixels) {
				m_depthBg.setNumChannels(1);
				m_cvDepthBg = ofxCv::toCv(m_depthBg);
				m_learntBg = true;
				ofLogNotice("ofApp::update()") << "Imported background";
			}
		}

		// Point cloud processing
		if (m_learntBg) {
			// Background removal
			cv::Mat diff;
			cv::Mat currentDepth = ofxCv::toCv(m_ofxKinect.getDepthPixelsRef());
			cv::absdiff(m_cvDepthBg, currentDepth, diff);
			UCHAR *p_maskRow;
			USHORT *p_diffRow;
			USHORT *p_depth;
			for (int row = 0; row < m_kinectHeight; ++row) {
				p_maskRow = m_cvBgMask.ptr<UCHAR>(row);
				p_diffRow = diff.ptr<USHORT>(row);
				p_depth = currentDepth.ptr<USHORT>(row);
				for (int col = 0; col < m_kinectWidth; ++col) {
					if (*(p_depth + col) != 0)
						*(p_maskRow + col) = *(p_diffRow + col) / 10; // to cm
					else
						*(p_maskRow + col) = 0;
				}
			}
			cv::Mat kernel = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(11, 11));
			cv::erode(m_cvBgMask, m_cvBgMask, kernel);
			cv::dilate(m_cvBgMask, m_cvBgMask, kernel);
			cv::threshold(m_cvBgMask, m_cvBgMask, 20, 255, CV_THRESH_BINARY);
			m_bgMask.update();

			// Blob detection
			float finderRes = m_blobFinder.getResolution();
			finderRes *= finderRes;

			m_blobFinder.findBlobs(&m_bgMask, ofVec3f(-10, -10, -10), ofVec3f(10, 10, 10),
				ofVec3f(0.05, 0.05, 0.1), 2, 0.06, 1.2, (int)(0.001*m_kinectHeight*m_kinectWidth / finderRes), 1);

			m_blobFound = false;
			if (m_blobFinder.nBlobs != 0) {
				m_modelBlob = m_blobFinder.blobs[0];
				for (int i = 1; i < m_blobFinder.nBlobs; ++i) {
					m_modelBlob = m_modelBlob->volume < m_blobFinder.blobs[i]->volume ? m_blobFinder.blobs[i] : m_modelBlob;
				}
				m_blobFound = true;
			}

			if (m_blobFound) {
				//ofUtilities::computeNormals(m_modelBlob->mesh, false);

				// Segmenting the garment on the body
				//ofImage grayStream;
				//grayStream.setFromPixels(m_ofxKinect.getColorPixelsRef());
				//cv::Mat cvGrayStream = ofxCv::toCv(grayStream);
				//cv::cvtColor(cvGrayStream, cvGrayStream, CV_RGB2GRAY);
				//cv::Canny(cvGrayStream, m_cvGarmentSeg, m_cannyThresh1, m_cannyThresh2);
				//m_garmentSeg.update();
			}
		}
	}
	// PROCESS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
}

void ofApp::draw() {
	if (m_ofxKinect.isFrameNew()) {
		// COMPUTER SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/

		ofBackground(ofColor::grey);
		m_projectorWindow.background(ofColor::black);

		// Top Left
		m_ofxKinect.draw(0, 0);

		// Top Right
		m_ofxKinect.drawDepth(m_kinectWidth, 0);

		// Bottom Left
		m_bgMask.draw(0, m_kinectHeight);

		// Bottom Right
		if (m_blobFound) {
			//m_garmentSeg.draw(m_kinectWidth, m_kinectHeight);

			ofPushMatrix();
				ofEnableDepthTest();
				ofTranslate(m_kinectWidth, m_kinectHeight);
				ofScale(1000, 1000, 1000);
				ofTranslate(0, 0, -m_modelBlob->maxZ.z -1);
				m_modelBlob->mesh.drawVertices();
				ofTranslate(m_modelBlob->massCenter);
				ofDrawAxis(0.5f);
				//ofScale(maxX.x - minX.x, maxY.y - minY.y, maxZ.z - minZ.z);
				ofDisableDepthTest();
			ofPopMatrix();
		}

		// GUI
		m_fpsGui.setup(std::string("fps : ") + std::to_string(ofGetFrameRate()));
		m_gui.draw();

		// COMPUTER SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-

		 
		// PROJECTOR SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/

		if (m_blobFound) {
			m_projectorWindow.begin();
				m_shader.begin();
				m_shader.setUniformMatrix4f("toProjector", ofMatrix4x4::getTransposedOf(m_kinectProjectorToolkit.getTransformMatrix()));
					m_modelBlob->mesh.draw();
				m_shader.end();
			m_projectorWindow.end();
		}

		// PROJECTOR SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-
	}
}

/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::exit() {
	m_ofxKinect.stop();
}

void ofApp::mousePressed(int x, int y, int button) {

}

void ofApp::keyPressed(int key) {
	if (key == ' ') {
		m_askPause = !m_askPause;
	}
	else if (key == 's') {
		m_askSaveAssets = true;
	}
	else if (key == 'b') {
		m_askBgLearning = true;
	}
	else if (key == 'i') {
		m_askBgExport = true;
	}
}

/* OF ROUTINES	-	-	-	-	-	-	-	-	-	-	-	-	-	*/


/* METHODS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::toProjectorSpace(ofMesh& mesh) {
	ofVec2f projectorCoordinates;

	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		projectorCoordinates = m_kinectProjectorToolkit.getProjectedPoint(mesh.getVertex(i)*1000);

		projectorCoordinates.x = ofMap(projectorCoordinates.x, 0, 1, 0, m_projectorWindow.getWidth());
		projectorCoordinates.y = ofMap(projectorCoordinates.y, 0, 1, 0, m_projectorWindow.getHeight());

		mesh.setVertex(i, ofVec3f(projectorCoordinates.x, projectorCoordinates.y, 0));
	}
}

void ofApp::meshParameterizationLSCM(ofMesh& mesh, int textureSize) {
	mesh.enableTextures();
	ofVec2f* texCoords = new ofVec2f[mesh.getNumVertices()];
	mesh.addTexCoords(texCoords, mesh.getNumVertices());
	delete[] texCoords;

	LSCM lscm(mesh);
	lscm.apply();
	float uRange = std::abs(lscm.umax - lscm.umin);
	float vRange = std::abs(lscm.vmax - lscm.vmin);
	float maxRange = std::max(uRange, vRange);
	ofVec2f outputRange(textureSize*uRange / maxRange, textureSize*vRange / maxRange);

	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		mesh.setTexCoord(i, ofUtilities::mapVec2f(mesh.getTexCoord(i), ofVec2f(lscm.umin, lscm.vmin), ofVec2f(lscm.umax, lscm.vmax), ofVec2f(0, 0), outputRange));
	}
}

/* METHODS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	*/