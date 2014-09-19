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

	bool initSensor = mOfxKinect.initSensor();
	if (!initSensor) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's sensor";
		exit();
	}

	bool initStreams = mOfxKinect.initColorStream(m_kinectWidth, m_kinectHeight) && mOfxKinect.initDepthStream(m_kinectWidth, m_kinectHeight, false, true);
	if (!initStreams) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's color and/or depth streams";
		exit();
	}

	bool kinectStarted = mOfxKinect.start();
	if (!kinectStarted) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't start the kinect";
		exit();
	}

	// Projector
	mKinectProjectorToolkit.loadCalibration("calibration.xml");
	mProjectorWindow = ofUtilities::ofVirtualWindow(1920, 0, 1024, 768);

	// HARDWARE INIT	-	-	-	-	-	-	-	-	-	-	-	-


	// OPEN GL	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	ofDisableAlphaBlending();
	ofSetFrameRate(30);

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

	// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

	// Blob finder and tracker
	m_blobFinder.init(&mOfxKinect, false); // standarized coordinate system: z in the direction of gravity
	m_blobFinder.setResolution(BF_HIGH_RES);
	m_blobFinder.setRotation(ofVec3f(0, 0, 0));
	m_blobFinder.setTranslation(ofVec3f(0, 0, 0));
	m_blobFinder.setScale(ofVec3f(0.001, 0.001, 0.001)); // mm to meters

	// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

	// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


	// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	m_gui.setup();
	m_gui.add(m_bgLearningCycleGui.setup("bg learning iterations", 10, 1, 50));
	m_gui.add(m_nearClipGui.setup("near clip",1.500,0,8.000));
	m_gui.add(m_farClipGui.setup("far clip", 3.000, 0, 8.000));

	// Keys
	m_askPause = false;
	m_askSaveMesh = false;

	// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
}

void ofApp::update() {
	// EVENTS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	if (m_askSaveMesh) {
		m_modelBlob.mesh.save("export.ply");
		ofLogNotice("ofApp::update") << "Exported the 3D mesh";
		m_askSaveMesh = false;
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

	mOfxKinect.update();

	// HARDWARE	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// PROCESS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/
	if (mOfxKinect.isFrameNew()) {
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
			cv::absdiff(m_cvDepthBg, ofxCv::toCv(mOfxKinect.getDepthPixelsRef()), diff);
			for (int row = 0; row < m_kinectHeight; ++row) {
				for (int col = 0; col < m_kinectWidth; ++col) {
					if (mOfxKinect.getDepthPixelsRef()[row*m_kinectWidth + col] != 0)
						*(m_cvBgMask.ptr<UCHAR>(row) +col) = min(*(diff.ptr<USHORT>(row) +col) / 10, 255); // to cm
					else
						*(m_cvBgMask.ptr<UCHAR>(row) +col) = 0;
				}
			}
			cv::Mat kernel = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(3, 3));
			cv::erode(m_cvBgMask, m_cvBgMask, kernel);
			cv::dilate(m_cvBgMask, m_cvBgMask, kernel);
			cv::threshold(m_cvBgMask, m_cvBgMask, 10, 255, CV_THRESH_BINARY);
			m_bgMask.update();

			// Blob detection
			float finderRes = m_blobFinder.getResolution();
			finderRes *= finderRes;

			m_blobFinder.findBlobs(&m_bgMask, ofVec3f(-10, -10, -10), ofVec3f(10, 10, 10),
				ofVec3f(0.05, 0.05, 0.1), 2, 0.06, 1.2, (int)(0.001*m_kinectHeight*m_kinectWidth / finderRes), 2);

			m_blobFound = false;
			if (m_blobFinder.nBlobs != 0) {
				m_modelBlob = m_blobFinder.blobs[0];
				for (int i = 1; i < m_blobFinder.nBlobs; ++i) {
					m_modelBlob = m_modelBlob.volume < m_blobFinder.blobs[i].volume ? m_blobFinder.blobs[i] : m_modelBlob;
				}
				m_blobFound = true;
			}
		}
	}
	// PROCESS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
}

void ofApp::draw() {
	if (mOfxKinect.isFrameNew()) {

		// COMPUTER SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/

		ofBackground(ofColor::grey);
		mProjectorWindow.background(ofColor::black);

		// Top Left
		mOfxKinect.draw(0, 0);

		// Top Right
		mOfxKinect.drawDepth(m_kinectWidth, 0);

		// Bottom Left
		m_bgMask.draw(0, m_kinectHeight);

		// Bottom Right
		if (m_blobFound) {
			ofPushMatrix();
				ofEnableDepthTest();
				ofTranslate(m_kinectWidth, m_kinectHeight);
				ofScale(1000, 1000, 1000);
				ofTranslate(0, 0, -m_modelBlob.maxZ.z -1);
				m_modelBlob.mesh.drawVertices();
				ofTranslate(m_modelBlob.massCenter);
				ofDrawAxis(0.5f);
				//ofScale(maxX.x - minX.x, maxY.y - minY.y, maxZ.z - minZ.z);
				ofDisableDepthTest();
			ofPopMatrix();
		}

		// GUI
		m_gui.draw();

		// COMPUTER SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-

		 
		// PROJECTOR SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/
		/*
		if (mOfGarmentContour.size() != 0) {
			mGarmentGeneratedMesh.computeWorldMesh(mOfxKinect);
			ofMesh worldMesh = mGarmentGeneratedMesh.getWorldMeshRef();
			toProjectorSpace(worldMesh);
			mProjectorWindow.begin();
			mChessboardImage.bind();
			worldMesh.draw();
			mChessboardImage.unbind();

				ofPath projectorGarmentPath = ofUtilities::polylineToPath(projectorGarmentContour);
				projectorGarmentPath.setFilled(true);
				projectorGarmentPath.setFillColor(ofColor::red);
				projectorGarmentPath.draw();

			mProjectorWindow.end();
		}*/

		// PROJECTOR SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-
	}
}

/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::exit() {
	mOfxKinect.stop();
}

void ofApp::mousePressed(int x, int y, int button) {

}

void ofApp::keyPressed(int key) {
	if (key == ' ') {
		m_askPause = !m_askPause;
	}
	else if (key == 's') {
		m_askSaveMesh = true;
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
		projectorCoordinates = mKinectProjectorToolkit.getProjectedPoint(mesh.getVertex(i));

		projectorCoordinates.x = ofMap(projectorCoordinates.x, 0, 1, 0, mProjectorWindow.getWidth());
		projectorCoordinates.y = ofMap(projectorCoordinates.y, 0, 1, 0, mProjectorWindow.getHeight());

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