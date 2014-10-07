#include "ofApp.h"

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "cvHelper.h"
#include "lscm.h"
#include "foldTracker.h"

/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::setup() {
	// HARDWARE INIT	/	/	/	/	/	/	/	/	/	/	/	/

	// Kinect
	kinectWidth_ = 640;
	kinectHeight_ = 480;

	bool initSensor = ofxKinect_.initSensor();
	if (!initSensor) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's sensor";
		exit();
	}

	bool initStreams = ofxKinect_.initColorStream(kinectWidth_, kinectHeight_) && ofxKinect_.initDepthStream(kinectWidth_, kinectHeight_, false, true);
	if (!initStreams) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's color and/or depth streams";
		exit();
	}

	bool kinectStarted = ofxKinect_.start();
	if (!kinectStarted) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't start the kinect";
		exit();
	}

	// Projector
	projectorWindow_ = ofUtilities::ofVirtualWindow(1920, 0, 1024, 768);
	kinectProjectorToolkit_.loadCalibration("cal.xml", projectorWindow_.getWidth(), projectorWindow_.getHeight());

	// HARDWARE INIT	-	-	-	-	-	-	-	-	-	-	-	-


	// OPEN GL	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	ofDisableAlphaBlending();
	ofSetFrameRate(30);
	shader_.load("shaders/toProjector.vert", "shaders/toProjector.frag");

	// OPEN GL	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// KINECT SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/	/

	// Background learning
	bgLearningCycleCount_ = 0;
	askBgLearning_ = false;
	learntBg_ = false;

	// Background segmentation
	bgMask_.allocate(kinectWidth_,kinectHeight_,OF_IMAGE_GRAYSCALE);
	cvBgMask_ = ofxCv::toCv(bgMask_);

	// Blob detection
	blobFound_ = false;

	// KINECT SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

	// Blob finder and tracker
	blobFinder_.init(&ofxKinect_); // standarized coordinate system: z in the direction of gravity
	//m_blobFinder.setResolution(BF_HIGH_RES);
	blobFinder_.setScale(ofVec3f(0.001,0.001,0.001)); // mm to meters

	// Fold processing
	askFoldComputation_ = false;

	// Mesh
	//m_blobMesh = garmentAugmentation::ofSemiImplicitActiveMesh(20, 20);

	// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

	chessboardImage_.loadImage("chessboard.png");

	// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


	// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	gui_.setup();
	gui_.add(bgLearningCycleGui_.setup("bg learning iterations", 10, 1, 50));
	gui_.add(nearClipGui_.setup("near clip",1.500,0,8.000));
	gui_.add(farClipGui_.setup("far clip", 3.000, 0, 8.000));
	gui_.add(cannyThresh1_.setup("canny thres 1", 160, 0, 255));
	gui_.add(cannyThresh2_.setup("canny thres 2", 0, 0, 255));

	// Mesh
	//m_gui.add(m_adaptationRate.setup("adaptation rate", 2, 0, 10));
	//m_gui.add(m_boundaryWeight.setup("boundary weight", 0, 0, 4));
	//m_gui.add(m_depthWeight.setup("depth weight", 2, 0, 4));

	// Keys
	askPause_ = false;
	askSaveAssets_ = false;
	askBgExport_ = false;

	// FPS
	gui_.add(fpsGui_.setup(std::string("fps :")));

	// GUI	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
}

void ofApp::update() {
	// EVENTS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	if (askSaveAssets_) {
		//m_blobMesh.save("mesh_export.ply");
		ofSaveImage(ofxKinect_.getColorPixelsRef(), "color_ir_export.tif");
		ofSaveImage(ofxKinect_.getDepthPixelsRef(), "depth_export.tif");
		ofSaveImage(normalsImg_, "normals_export.tif");
		ofLogNotice("ofApp::update") << "Exported the 3D mesh, IR/color and depth image";
		askSaveAssets_ = false;
	}
	if (askBgExport_) {
		if (learntBg_) {
			ofSaveImage(depthBg_, "background_export.tif");
			ofLogNotice("ofApp::update") << "Exported the learnt background";
		}
		else {
			ofLogNotice("ofApp::update") << "Can't export the background, press b to learn background";
		}
		askBgExport_ = false;
	}
	if (askPause_)
		return;

	//m_blobMesh.setAdaptationRate(m_adaptationRate);
	//m_blobMesh.setBoundaryWeight(m_boundaryWeight);
	//m_blobMesh.setDepthWeight(m_depthWeight);

	// EVENTS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// HARDWARE	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	ofxKinect_.update();

	// HARDWARE	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-


	// PROCESS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/
	if (ofxKinect_.isFrameNew()) {
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
		if (!learntBg_) {
			bool importedPixels = ofLoadImage(depthBg_, "background_export.tif");
			if (importedPixels) {
				depthBg_.setNumChannels(1);
				cvDepthBg_ = ofxCv::toCv(depthBg_);
				learntBg_ = true;
				ofLogNotice("ofApp::update()") << "Imported background";
			}
		}

		// Point cloud processing
		if (learntBg_) {
			// Background removal
			cv::Mat diff;
			cv::Mat currentDepth = ofxCv::toCv(ofxKinect_.getDepthPixelsRef());
			cv::absdiff(cvDepthBg_, currentDepth, diff);
			UCHAR *p_maskRow;
			USHORT *p_diffRow;
			USHORT *p_depth;
			for (int row = 0; row < kinectHeight_; ++row) {
				p_maskRow = cvBgMask_.ptr<UCHAR>(row);
				p_diffRow = diff.ptr<USHORT>(row);
				p_depth = currentDepth.ptr<USHORT>(row);
				for (int col = 0; col < kinectWidth_; ++col) {
					if (*(p_depth + col) != 0)
						*(p_maskRow + col) = *(p_diffRow + col) / 10; // to cm
					else
						*(p_maskRow + col) = 0;
				}
			}
			cv::Mat kernel = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(11, 11));
			cv::erode(cvBgMask_, cvBgMask_, kernel);
			cv::dilate(cvBgMask_, cvBgMask_, kernel);
			cv::threshold(cvBgMask_, cvBgMask_, 20, 255, CV_THRESH_BINARY);
			bgMask_.update();

			// Blob detection
			float finderRes = blobFinder_.getResolution();
			finderRes *= finderRes;

			blobFinder_.findBlobs(&bgMask_,
				ofVec3f(0.05, 0.05, 0.1), 1, 0.06, 1.2, (int)(0.001*kinectHeight_*kinectWidth_ / finderRes), 1);

			blobFound_ = false;
			if (blobFinder_.nBlobs != 0)
				blobFound_ = true;

			if (blobFound_) {
				garment_.update(blobFinder_, *blobFinder_.blobs[0]);
				markFolds();
				/*
				if (!m_blobMesh.isGenerated()) {
					m_blobMesh.generateMesh(m_blobFinder, *m_modelBlob);
				}
				else {
					m_blobMesh.updateMesh(m_blobFinder, *m_modelBlob);
				}*/
			}
		}
	}
	// PROCESS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	-
}

void ofApp::draw() {
	if (ofxKinect_.isFrameNew()) {
		// COMPUTER SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/

		ofBackground(ofColor::grey);
		projectorWindow_.background(ofColor::black);

		// Top Left
		ofxKinect_.draw(0, 0);

		// Top Right
		ofxKinect_.drawDepth(kinectWidth_, 0);

		// Bottom Left
		bgMask_.draw(0, kinectHeight_);

		// Bottom Right
		if (blobFound_) {
			//m_normalsImg.draw(m_kinectWidth, m_kinectHeight);
			if (askPause_)
				easyCam_.begin();
			ofPushMatrix();
			//ofEnableDepthTest();
			ofTranslate(kinectWidth_, kinectHeight_);
			ofScale(1000, 1000, 1000);
			ofTranslate(0, 0, -garment_.getBlobRef().maxZ.z - 1);
			//ofScale(0.001*m_blobFinder.getResolution(), 0.001*m_blobFinder.getResolution(), 0.001);
			//m_blobMesh.drawWireframe();
			garment_.drawMesh();
			if (foldAxis_.size() != 0) {
				ofSetColor(ofColor::blue);
				foldAxis_.draw();
				ofSetColor(ofColor::white);
			}
			//ofDisableDepthTest();
			ofPopMatrix();
			if (askPause_)
				easyCam_.end();
		}

		// GUI
		fpsGui_.setup(std::string("fps : ") + std::to_string(ofGetFrameRate()));
		gui_.draw();

		// COMPUTER SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-

		 
		// PROJECTOR SCREEN	/	/	/	/	/	/	/	/	/	/	/	/	/

		if (blobFound_) {
			projectorWindow_.begin();
				shader_.begin();
				shader_.setUniformMatrix4f("toProjector", ofMatrix4x4::getTransposedOf(kinectProjectorToolkit_.getTransformMatrix()));
					//m_modelBlob->mesh.draw();
					ofSetColor(ofColor::red);
					ofSetLineWidth(5);
					foldAxis_.draw();
					ofSetLineWidth(1);
					ofSetColor(ofColor::white);
				shader_.end();
			projectorWindow_.end();
		}

		// PROJECTOR SCREEN	-	-	-	-	-	-	-	-	-	-	-	-	-
	}
}

/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::exit() {
	ofxKinect_.stop();
}

void ofApp::mousePressed(int x, int y, int button) {

}

void ofApp::keyPressed(int key) {
	if (key == ' ') {
		askPause_ = !askPause_;
	}
	else if (key == 's') {
		askSaveAssets_ = true;
	}
	else if (key == 'b') {
		askBgLearning_ = true;
	}
	else if (key == 'i') {
		askBgExport_ = true;
	}
	else if (key == 'c') {
		askFoldComputation_ = true;
	}
}

/* OF ROUTINES	-	-	-	-	-	-	-	-	-	-	-	-	-	*/


/* METHODS	/	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::toProjectorSpace(ofMesh& mesh) {
	ofVec2f projectorCoordinates;

	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		projectorCoordinates = kinectProjectorToolkit_.getProjectedPoint(mesh.getVertex(i)*1000);

		projectorCoordinates.x = ofMap(projectorCoordinates.x, 0, 1, 0, projectorWindow_.getWidth());
		projectorCoordinates.y = ofMap(projectorCoordinates.y, 0, 1, 0, projectorWindow_.getHeight());

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

void ofApp::markFolds() {
	int width = kinectWidth_ / blobFinder_.getResolution();
	int height = kinectHeight_ / blobFinder_.getResolution();

	int patchSize = 9;
	garmentAugmentation::foldTracker tracker(&garment_);
	garmentAugmentation::foldTracker::trackerPatch patch = tracker.createPatch(ofRectangle(0, 0, 0, 0));
	float deformation;
	std::vector<ofVec3f> points;
	for (int y = 0; y < height - 5; y+= 20) {
		for (int x = 0; x < width - patchSize; x++) {
			patch.moveTo(ofRectangle(x, y, patchSize, 5));
			if (patch.insideMesh()) {
				deformation = patch.getDeformationPercent();
				if (deformation  > cannyThresh1_ / 2.55) {
					patch.setColor(ofColor::red);
					if (askFoldComputation_) {
						std::vector<ofVec3f> patchPts = patch.getPoints();
						points.insert(points.end(),patchPts.begin(),patchPts.end());
					}
				}
			}
		}
	}

	if (askFoldComputation_) {
		double xBar = 0, yBar = 0, zBar = 0;
		for (int i = 0; i < points.size(); ++i) {
			xBar += points[i].x / points.size(); yBar += points[i].y / points.size(); zBar += points[i].z / points.size();
		}

		Eigen::MatrixX3f A(points.size(), 3);
		for (int i = 0; i < points.size(); ++i) {
			A(i, 0) = points[i].x - xBar; A(i, 1) = points[i].y - yBar; A(i, 2) = points[i].z - zBar;
		}

		ofVec3f rightVector;
		Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinV);
		rightVector.x = svd.matrixV()(0, 0);
		rightVector.y = svd.matrixV()(1, 0);
		rightVector.z = svd.matrixV()(2, 0);

		ofVec3f meanVector(xBar, yBar, zBar);

		foldAxis_.clear();
		foldAxis_.addVertex(meanVector + -10000 * rightVector);
		foldAxis_.addVertex(meanVector + 10000 * rightVector);

		//m_askFoldComputation = false;
	}
}

/*
void ofApp::markFolds(ofFastMesh& mesh) {
	unsigned int* map =  mesh.get2dIndicesMap();
	if (!m_normalsImg.bAllocated())
		m_normalsImg.allocate(mesh.getIndicesMapWidth(), mesh.getIndicesMapHeight(),OF_IMAGE_GRAYSCALE);
	m_normalsImg.setColor(ofColor::black);

	m_cvNormalsImg = ofxCv::toCv(m_normalsImg);

	ofImage color;
	color.allocate(mesh.getIndicesMapWidth(), mesh.getIndicesMapHeight(), OF_IMAGE_COLOR);
	cv::Mat cvColor = ofxCv::toCv(color);

	cv::Vec3b* p_row;
	ofVec3f normal;
	int idx;
	for (int row = 0; row < mesh.getIndicesMapHeight(); ++row) {
		p_row = cvColor.ptr<cv::Vec3b>(row);
		for (int col = 0; col < mesh.getIndicesMapWidth(); ++col) {
			idx = *(map + row*mesh.getIndicesMapWidth() + col);
			if ( idx >= 0) {
				normal = mesh.getNormal(idx);
				*(p_row + col) = cv::Vec3b(normal.x * 255, normal.y * 255, normal.z * 255);
			}
		}
	}
	
	cv::Mat horizontalFilter = (cv::Mat_<float>(5, 5) << 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, -1, -1, 1, -1, -1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0);
	//horizontalFilter /= 10;
	cv::filter2D(cvColor, cvColor, m_cvNormalsImg.depth(), horizontalFilter);
	uchar* p_uRow;
	cv::Vec3b rgb;
	for (int row = 0; row < mesh.getIndicesMapHeight(); ++row) {
		p_row = cvColor.ptr<cv::Vec3b>(row);
		p_uRow = m_cvNormalsImg.ptr<uchar>(row);
		for (int col = 0; col < mesh.getIndicesMapWidth(); ++col) {
			if (*(map + row*mesh.getIndicesMapWidth() + col) >= 0) {
				rgb = *(p_row + col);
				*(p_uRow + col) = (rgb.val[0] + rgb.val[1] + rgb.val[2]) / 3;
			}
		}
	}
	m_normalsImg.update();
} */

/*void ofApp::markFolds(ofFastMesh& mesh) {
	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		mesh.addColor(ofColor::lightYellow);
	}

	int width = m_kinectWidth / m_blobFinder.getResolution();
	int height = m_kinectHeight / m_blobFinder.getResolution();

	int patchSize = 25;
	ofFastMesh::ofFastMeshPatch patch = mesh.getPatch(ofRectangle(0, 0, patchSize, patchSize));

	float deformation;
	std::vector<ofFastMesh::ofFastMeshPatch> candidates;
	for (int x = 0; x < width - patchSize; x += patchSize/2) {
		for (int y = 0; y < height - patchSize; y += patchSize / 2) {
			patch.move(ofRectangle(x, y, patchSize, patchSize));
			if (patch.insideMesh()) {
				deformation = patch.deformationAreaPercent();

				// try to reduce the patch
				if (deformation > m_cannyThresh1 / 2.55) {
					float newDeformation = deformation;
					ofFastMesh::ofFastMeshPatch newPatch = patch;
					int reduce = 1;
					do {
						deformation = newDeformation;
						patch = newPatch;
						newPatch.move(ofRectangle(x + reduce, y + reduce, patchSize - reduce * 2, patchSize - reduce * 2));
						if (!newPatch.insideMesh())
							break;
						newDeformation = newPatch.deformationAreaPercent();
						reduce++;
					} while (reduce < patchSize / 2 && newDeformation > deformation);

					candidates.push_back(patch);
				}
			}
		}
	}

	for (int i = 0; i < candidates.size(); ++i) {
		if (candidates[i].deformationAreaPercent() > m_cannyThresh2 / 2.55)
			candidates[i].setColor(ofColor::red);
	}
}*/

/* METHODS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	*/