#include "ofApp.h"

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "cvHelper.h"
#include "lscm.h"
#include "foldTracker.h"
#include "flyingLights.h"

/* CONSTANTS	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

// HARDWARE HANDLERS	/	/	/	/	/	/	/	/	/	/	/	/

const int ofApp::screenWidth_ = 1920, ofApp::screenHeight_ = 1080;
const int ofApp::projectorWidth_ = 1024, ofApp::projectorHeight_ = 768;
const int ofApp::kinectWidth_ = 640, ofApp::kinectHeight_ = 480;

// HARDWARE HANDLERS	-	-	-	-	-	-	-	-	-	-	-	-

// KINECT WORLD SPACE	/	/	/	/	/	/	/	/	/	/	/	/

const float ofApp::toWorldUnits_ = 0.001; // mm to meters

// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-

/* CONSTANTS	/	/	/	/	/	/	/	/	/	/	/	/	/	*/


/* OF ROUTINES	/	/	/	/	/	/	/	/	/	/	/	/	/	*/

void ofApp::setup() {
	// HARDWARE INIT	/	/	/	/	/	/	/	/	/	/	/	/

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
	kinectProjectorToolkit_.loadCalibration("cal.xml", projectorWindow_.getWidth(), projectorWindow_.getHeight());
	projectorWindow_ = ofUtilities::ofVirtualWindow(screenWidth_, 0, projectorWidth_, projectorHeight_);

	// HARDWARE INIT	-	-	-	-	-	-	-	-	-	-	-	-


	// OPEN GL	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	ofDisableAlphaBlending();
	ofSetFrameRate(30);

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
	blobFinder_.init(&ofxKinect_, kinectWidth_, kinectHeight_); // standarized coordinate system: z in the direction of gravity
	//m_blobFinder.setResolution(BF_HIGH_RES);
	blobFinder_.setScale(ofVec3f(toWorldUnits_)); // mm to meters

	// Fold processing
	askFoldComputation_ = false;
	numFolds_ = 1;

	// Physic animations
	std::unique_ptr<garmentAugmentation::garment::animation> lightsEffect(new garmentAugmentation::garment::flyingLights());
	garment_.addAnimation(std::move(lightsEffect));

	// Mesh
	//m_blobMesh = garmentAugmentation::ofSemiImplicitActiveMesh(20, 20);

	// KINECT WORLD SPACE	-	-	-	-	-	-	-	-	-	-	-	-


	// PROJECTOR SCREEN SPACE	/	/	/	/	/	/	/	/	/	/	/

	chessboardImage_.loadImage("chessboard.png");

	// PROJECTOR SCREEN SPACE	-	-	-	-	-	-	-	-	-	-	-


	// GUI	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/	/

	gui_.setup();
	gui_.add(bgLearningCycleGui_.setup("bg learning iterations", 10, 1, 50));
	gui_.add(deformationThres_.setup("canny thres 1", 100, 0, 200));

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
				detectFolds();
				garment_.updateAnimations();
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
			if (askPause_)
				easyCam_.begin();

			ofPushMatrix();
				//ofEnableDepthTest();
				ofTranslate(kinectWidth_, kinectHeight_);
				ofScale(1 / toWorldUnits_, 1 / toWorldUnits_, 1 / toWorldUnits_);
				ofTranslate(0, 0, -garment_.getBlobRef().maxZ.z - 1);
				garment_.drawMesh();
				if (garment_.getFoldsRef().size() != 0) {
					ofSetColor(ofColor::blue);
					for (int i = 0; i < garment_.getFoldsRef().size(); ++i) {
						garment_.getFoldsRef()[i].draw();
					}
					ofSetColor(ofColor::white);
				}
				garment_.drawAnimations();
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
				ofMultMatrix(ofMatrix4x4::getTransposedOf(kinectProjectorToolkit_.getTransformMatrix()));
				ofScale(1 / toWorldUnits_, 1 / toWorldUnits_, 1 / toWorldUnits_);
				garment_.drawAnimations();
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

void ofApp::meshParameterizationLSCM(const int textureSize, ofMesh& mesh) {
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

void ofApp::detectFolds() {

	// Detect deformed areas
	int width = kinectWidth_ / blobFinder_.getResolution();
	int height = kinectHeight_ / blobFinder_.getResolution();

	garmentAugmentation::garment::foldTracker tracker(&garment_, ofRectangle(0, 0, 0, 0));
	float deformation;
	std::vector<ofVec3f> points;

	for (int patchSize = 9; patchSize <= 11; patchSize += 2) {
		for (int y = 0; y < height - 15; y += 20) {
			for (int x = 0; x < width - patchSize; x += ((patchSize-1)/2)) {
				tracker.moveTo(ofRectangle(x, y, patchSize, 10));
				if (tracker.insideMesh()) {
					deformation = tracker.getDeformationPercent();
					if (deformation  > deformationThres_) {
						tracker.setColor(ofColor::red);
						if (askFoldComputation_) {
							std::vector<ofVec3f> patchPts = tracker.getPoints();
							points.insert(points.end(), patchPts.begin(), patchPts.end());
						}
					}
				}
			}
		}
	}

	// Compute folds from deformaed areas
	if (askFoldComputation_) {

		// Computation structure
		typedef struct fold fold;
		struct fold {
			ofVec3f direction;
			ofVec3f meanPt;
			ofVec3f aPt;
			ofVec3f top, bottom;

			int nbPts;
			std::vector<float> distToPts;

			void clear() {
				direction = ofVec3f(0, 0, 0);
				meanPt = ofVec3f(0, 0, 0);
				aPt = ofVec3f(0, 0, 0);
				top.y = std::numeric_limits<float>::infinity();
				bottom.y = -std::numeric_limits<float>::infinity();

				nbPts = 0;
			}

			fold() {
				clear();
			}
		};

		// Initialize the clusters
		std::vector<uchar> clusterLabel(points.size()); // rel. to points' vector
		std::vector<fold> foldClusters(numFolds_);
		if (numFolds_ > 1) {
			float maxX = -std::numeric_limits<float>::infinity();
			float minX = std::numeric_limits<float>::infinity();
			for (int i = 0; i < points.size(); ++i) {
				maxX = maxX < points[i].x ? points[i].x : maxX;
				minX = minX > points[i].x ? points[i].x : minX;
			}

			float deltaX = (maxX - minX) / (numFolds_ - 1);
			for (int i = 0; i < points.size(); ++i) {
				clusterLabel[i] = floor(((points[i].x - minX) / deltaX) + 0.5);
			}
		}
		else {
			for (int i = 0; i < points.size(); ++i) {
				clusterLabel[i] = 0;
			}
		}

		// K-means
		bool change = true;
		float avgDist;
		int ptsNbApprox = points.size() / numFolds_;
		while (change) {
			change = false;

			// Update the clusters
			for (int nCluster = 0; nCluster < foldClusters.size(); ++nCluster) {
				foldClusters[nCluster].clear();
				for (int i = 0; i < points.size(); ++i) {
					if (clusterLabel[i] == nCluster) {
						foldClusters[nCluster].meanPt.x += points[i].x;
						foldClusters[nCluster].meanPt.y += points[i].y;
						foldClusters[nCluster].meanPt.z += points[i].z;
						foldClusters[nCluster].top.y = foldClusters[nCluster].top.y > points[i].y ? points[i].y : foldClusters[nCluster].top.y;
						foldClusters[nCluster].bottom.y = foldClusters[nCluster].bottom.y < points[i].y ? points[i].y : foldClusters[nCluster].bottom.y;
						++foldClusters[nCluster].nbPts;
					}
				}

				if (foldClusters[nCluster].nbPts != 0) {
					foldClusters[nCluster].meanPt /= static_cast<float>(foldClusters[nCluster].nbPts);
					Eigen::MatrixX3f A(foldClusters[nCluster].nbPts, 3);
					int AIdx = 0;
					for (int i = 0; i < points.size(); ++i) {
						if (clusterLabel[i] == nCluster) {
							A(AIdx, 0) = points[i].x - foldClusters[nCluster].meanPt.x;
							A(AIdx, 1) = points[i].y - foldClusters[nCluster].meanPt.y;
							A(AIdx, 2) = points[i].z - foldClusters[nCluster].meanPt.z;
							++AIdx;
						}
					}

					Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinV);
					foldClusters[nCluster].direction.x = svd.matrixV()(0, 0);
					foldClusters[nCluster].direction.y = svd.matrixV()(1, 0);
					foldClusters[nCluster].direction.z = svd.matrixV()(2, 0);
					foldClusters[nCluster].direction.normalize();
					foldClusters[nCluster].aPt = foldClusters[nCluster].meanPt + foldClusters[nCluster].direction;

					// Compute the distance for each point	
					foldClusters[nCluster].distToPts.resize(points.size());
					for (int i = 0; i < points.size(); ++i) {
						foldClusters[nCluster].distToPts[i] = ((points[i] - foldClusters[nCluster].meanPt).cross(points[i] - foldClusters[nCluster].aPt)).length();
					}
				}
			}

			// Assign in regard of the distance
			avgDist = 0;
			for (int i = 0; i < points.size(); ++i) {
				float minDist = std::numeric_limits<float>::infinity();
				int closestCluster = 255;
				for (int nCluster = 0; nCluster < foldClusters.size(); ++nCluster) {
					if (foldClusters[nCluster].nbPts != 0 && foldClusters[nCluster].distToPts[i] < minDist) {
						minDist = foldClusters[nCluster].distToPts[i];
						closestCluster = nCluster;
					}
				}
				avgDist += minDist / points.size();
				if (clusterLabel[i] != closestCluster) {
					change = true;
					clusterLabel[i] = closestCluster;
				}
			}
		}

		if (avgDist > 0.1) {
			numFolds_++;
		}

		garment_.getFoldsRef().clear();
		for (int i = 0; i < foldClusters.size(); ++i) {
			if (foldClusters[i].nbPts != 0) {
				// Compute the segments
				float t = (foldClusters[i].top.y - foldClusters[i].meanPt.y) / foldClusters[i].direction.y;
				foldClusters[i].top = foldClusters[i].meanPt + t * foldClusters[i].direction;

				t = (foldClusters[i].bottom.y - foldClusters[i].meanPt.y) / foldClusters[i].direction.y;
				foldClusters[i].bottom = foldClusters[i].meanPt + t * foldClusters[i].direction;

				// Update the garment folds
				garment_.addFold(garmentAugmentation::garment::fold(foldClusters[i].bottom, foldClusters[i].top));
			}
		}	

		//m_askFoldComputation = false;
	}
}

/* METHODS	-	-	-	-	-	-	-	-	-	-	-	-	-	-	*/