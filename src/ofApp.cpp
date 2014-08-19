#include "ofApp.h"

#include <stdlib.h>
#include "cvHelper.h"
#include "lscm.h"

//--------------------------------------------------------------
void ofApp::setup() {
	// Kinect
	mKcbKinect = mOfxKinect.getHandle();
	mKinectColorImgWidth = 640;
	mKinectColorImgHeight = 480;
	mKinectDepthImgWidth = 640;
	mKinectDepthImgHeight = 480;

	bool initSensor = mOfxKinect.initSensor();
	if (!initSensor) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's sensor";
		exit();
	}

	bool initStreams = mOfxKinect.initColorStream(mKinectColorImgWidth, mKinectColorImgHeight) && mOfxKinect.initDepthStream(mKinectDepthImgWidth, mKinectDepthImgHeight, false, true);
	if (!initStreams) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's color and/or depth streams";
		exit();
	}

	bool initSkeletonStream = mOfxKinect.initSkeletonStream(false);
	if (!initSkeletonStream) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't init the kinect's skeleton stream";
		exit();
	}

	bool kinectStarted = mOfxKinect.start();
	if (!kinectStarted) {
		ofLog(OF_LOG_FATAL_ERROR) << "Couldn't start the kinect";
		exit();
	}

	// Images allocation
	mOfSegmentedImg.allocate(mKinectColorImgWidth, mKinectColorImgHeight, OF_IMAGE_COLOR);
	mCvSegmentedImg = ofxCv::toCv(mOfSegmentedImg);
	mOfGarmentMask.allocate(mKinectColorImgWidth, mKinectColorImgHeight, OF_IMAGE_GRAYSCALE);
	mCvGarmentMask = ofxCv::toCv(mOfGarmentMask);

	// Computer vision tools
	mContourFinder.setAutoThreshold(false);
	mContourFinder.setFindHoles(false);
	mContourFinder.setInvert(false);
	mContourFinder.setSimplify(true);
	mContourFinder.setSortBySize(true);
	mContourFinder.setUseTargetColor(false);

	// Textures
	mChessboardImage.loadImage("chessboard.jpg");

	// OpenGL
	ofDisableAlphaBlending();
	ofSetFrameRate(30);

	// GUI-
	mGui.setup();
	mGui.add(mGarmentSegmentationLowH.setup("low hue thresh", 70, 0, 179));
	mGui.add(mGarmentSegmentationLowS.setup("low saturation thresh", 0, 0, 255));
	mGui.add(mGarmentSegmentationLowV.setup("low value thresh", 40, 0, 255));
	mGui.add(mGarmentSegmentationHighH.setup("high hue thresh", 120, 0, 179));
	mGui.add(mGarmentSegmentationHighS.setup("high saturation thresh", 95, 0, 255));
	mGui.add(mGarmentSegmentationHighV.setup("high value thresh", 255, 0, 255));
	mGui.add(mOpenKernelSize.setup("opening kernel size", 5, 1, 9));
	mGui.add(mCloseKernelSize.setup("closing kernel size", 9, 1, 9));
	mGui.add(mMorphoUseEllipse.setup("ellipse for morpho operations",false));
	mGui.add(mGarmentBodyPercent.setup("percent of clothing on body", 20, 0, 100)); // 30% is a good value for a t-shirt
	mGui.add(mDepthSmoothingKernelSize.setup("depth smoothing size", 3, 1, 9));
	mGui.add(mDepthSmoothingSigma.setup("depth smoothing sigma", 0, 0, 10));
	mGui.add(mMeshGenerationStep.setup("undersampling the polygonal mesh", 7, 1, 20)); // in pixels
	
	// Keys
	mPause = false;
	mSaveMesh = false;
}

//--------------------------------------------------------------
void ofApp::update() {
	if (mPause)
		return;

	// Smooth the depth image
	if (mDepthSmoothingSigma != 0) {
		mOfxKinect.setDepthSmoothing(true, mDepthSmoothingSigma, mDepthSmoothingKernelSize);
	}
	else {
		mOfxKinect.setDepthSmoothing(false);
	}

	mOfxKinect.update();

	// Model ROI
	mModelRoi = ofRectangle(mOfSegmentedImg.width, mOfSegmentedImg.height, 0, 0);
	int modelRoiXBottomRight = 0, modelRoiYBottomRight = 0;
	int modelBodyPixelsCount = 0; // Used to discriminate contours, more robust than bounding box if the model extend their arms, etc.

	// Segmentation from the skeleton
	if (mOfxKinect.isNewSkeleton()) {
		ofShortPixels depthPlayerPixel = mOfxKinect.getDepthPlayerPixelsRef(); // The kinect segmentation map

		for (int x = 0; x < mOfSegmentedImg.width; ++x) {
			for (int y = 0; y < mOfSegmentedImg.height; ++y) {
				USHORT playerId = depthPlayerPixel[x + y*mOfSegmentedImg.width];

				if (playerId != 0) {
					ofColor bgra = mOfxKinect.getColorPixelsRef().getColor(x, y);
					mOfSegmentedImg.setColor(x, y, ofColor(bgra.b, bgra.g, bgra.r));

					// Update the ROI
					mModelRoi.x = (x < mModelRoi.x) ? x : mModelRoi.x;
					mModelRoi.y = (y < mModelRoi.y) ? y : mModelRoi.y;
					modelRoiXBottomRight = (x > modelRoiXBottomRight) ? x : modelRoiXBottomRight;
					modelRoiYBottomRight = (y > modelRoiYBottomRight) ? y : modelRoiYBottomRight;
					modelBodyPixelsCount++;
				}
				else {
					mOfSegmentedImg.setColor(x, y, ofColor::black);
				}
			}
		}
	
		// Cloth segmentation and contour acquisition
		if (modelBodyPixelsCount != 0) {
			// Update the ROI information of the model
			mModelRoi.width = modelRoiXBottomRight - mModelRoi.x;
			mModelRoi.height = modelRoiYBottomRight - mModelRoi.y;
			cv::Mat cvSegmentedImgRoi = mCvSegmentedImg(ofxCv::toCv(mModelRoi));
			cv::Mat cvGarmentMaskRoi = mCvGarmentMask(ofxCv::toCv(mModelRoi));

			// OPENCV
			// Color segmentation
			cv::Mat cvSegmentedImgHsv;
			cv::cvtColor(cvSegmentedImgRoi, cvSegmentedImgHsv, CV_RGB2HSV);
			cv::inRange(cvSegmentedImgHsv, cv::Scalar(mGarmentSegmentationLowH, mGarmentSegmentationLowS, mGarmentSegmentationLowV), cv::Scalar(mGarmentSegmentationHighH, mGarmentSegmentationHighS, mGarmentSegmentationHighV), cvGarmentMaskRoi);

			// Morphological opening (remove small objects) and closing (fill small holes)
			cv::Mat openingOperator = cv::getStructuringElement(mMorphoUseEllipse ? cv::MORPH_ELLIPSE : cv::MORPH_RECT, cv::Size(mOpenKernelSize, mOpenKernelSize));
			cv::Mat closingOperator = cv::getStructuringElement(mMorphoUseEllipse ? cv::MORPH_ELLIPSE : cv::MORPH_RECT, cv::Size(mCloseKernelSize, mCloseKernelSize));
			cv::morphologyEx(cvGarmentMaskRoi, cvGarmentMaskRoi, cv::MORPH_OPEN, openingOperator);
			cv::morphologyEx(cvGarmentMaskRoi, cvGarmentMaskRoi, cv::MORPH_CLOSE, closingOperator);
			// !OPENCV

			// Find the biggest correct contour
			mContourFinder.setMinArea(modelBodyPixelsCount*mGarmentBodyPercent / 100.f);
			mContourFinder.findContours(cvGarmentMaskRoi);
			if (mContourFinder.getContours().size() != 0) {
				mCvGarmentContourModelRoiRel = mContourFinder.getContour(0);
				mOfGarmentContourModelRoiRel = mContourFinder.getPolyline(0);
				mOfGarmentMask.setColor(ofColor::black); // reset the mask as opencv only draw on the ROI
				ofxCv::fillPoly(mCvGarmentContourModelRoiRel, cvGarmentMaskRoi);
			}
			else {
				ofLog(OF_LOG_ERROR) << "Couldn't retrieve big enough contour";
				mCvGarmentContourModelRoiRel.clear();
				mOfGarmentContourModelRoiRel.clear();
			}
		}
		else {
			ofLog(OF_LOG_ERROR) << "Found skeleton but couldn't detect garment";
			mCvGarmentContourModelRoiRel.clear();
			mOfGarmentContourModelRoiRel.clear();
		}

		// Generate the point of cloud
		if (mOfGarmentContourModelRoiRel.size() != 0) {
			// Update the ROI information of the garment
			mGarmentRoi = mOfGarmentContourModelRoiRel.getBoundingBox();
			mGarmentRoi.translate(mModelRoi.getTopLeft());
			generateMesh(mCvGarmentMask(ofxCv::toCv(mGarmentRoi)), mCvGarmentContourModelRoiRel, mGarmentGeneratedMesh, mMeshGenerationStep, mGarmentRoi.getTopLeft());
			//computeNormals(mGarmentGeneratedMesh, true);
			meshParameterizationLSCM(mGarmentGeneratedMesh);

			if (mSaveMesh) {
				mGarmentGeneratedMesh.save("export.ply");
				ofLog() << "Exported the 3D mesh";
				mSaveMesh = false;
			}
		}
		mOfSegmentedImg.update();
		mOfGarmentMask.update();
	} // mOfxKinect.isNewSkeleton()
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (mOfxKinect.isFrameNew()) {
		ofBackground(0);

		// Top Left
		mOfxKinect.draw(0, 0);

		ofPushMatrix();
		ofSetColor(ofColor::red);
		ofTranslate(mModelRoi.getTopLeft());
		mOfGarmentContourModelRoiRel.draw();
		ofPopMatrix();

		ofSetColor(ofColor::white);

		// Top Right
		mOfxKinect.drawDepth(mKinectColorImgWidth, 0);

		// Bottom Left
		//mOfSegmentedImg.draw(0, mKinectColorImgHeight);
		mOfGarmentMask.draw(0, mKinectColorImgHeight);

		// Bottom Right
		//mOfGarmentMask.draw(mKinectColorImgWidth, mKinectColorImgHeight);
		if (mGarmentGeneratedMesh.hasVertices()) {
			glPointSize(1);

			mEasyCam.begin();
			mEasyCam.setTarget(mGarmentGeneratedMesh.getCentroid());
			ofSetColor(ofColor::blue);
			ofPushMatrix();
				ofEnableDepthTest();
				ofScale(-1, -1, 1);
				mChessboardImage.bind();
				mGarmentGeneratedMesh.draw();
				mChessboardImage.unbind();
				ofDisableDepthTest();
			ofPopMatrix();
			mEasyCam.end();
			ofSetColor(ofColor::white);
		}
	}
	mGui.draw();
}

//--------------------------------------------------------------
void ofApp::exit() {
	mOfxKinect.stop();
}

void ofApp::mousePressed(int x, int y, int button) {
	if (button == OF_MOUSE_BUTTON_MIDDLE) {
		mSaveMesh = true;
	}
}

void ofApp::keyPressed(int key) {
	if (key == ' ') {
		mPause = !mPause;
	}
}

void ofApp::generateMesh(cv::Mat& maskImage, const vector<cv::Point>& contour, ofMesh& mesh, int step, ofPoint offset){
	int width = maskImage.cols;
	int height = maskImage.rows;
	int meshRows = height / step;
	int meshCols = width / step;

	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
	mesh.enableIndices();
	mesh.enableTextures();

	// Fill the mesh with the point known by kinect sensor and fill :
	// meshIndexMap, recording which value is known, unknown or not part of the garment
	// mapIndicestoInterpolate, storing all unknown points' position in the matrix
	// mapReferenceIndices, storing all known points' position in the matrix
	cv::Mat meshIndexMap;
	meshIndexMap.create(meshRows, meshCols, CV_32S);
	std::deque<cvHelper::Mat2Pos> mapIndicestoInterpolate;
	std::vector<cvHelper::Mat2Pos> mapReferenceIndices;
	int index = -1;
	int x, y;
	for (int row = 0; row < meshRows; row++) {
		for (int col = 0; col < meshCols; col++) {
			x = col*step;
			y = row*step;

			if (!cvHelper::zeroAt(maskImage, y, x)) {
				ofVec3f worldCoordinates = mOfxKinect.getWorldCoordinates(x + offset.x, y + offset.y);
				if (abs(worldCoordinates.z) > 0.1) {
					mesh.addVertex(worldCoordinates);
					mesh.addTexCoord(ofVec2f(0, 0));
					*(meshIndexMap.ptr<int>(row)+col) = ++index;
					mapReferenceIndices.push_back(cvHelper::Mat2Pos(row, col));
				}
				else {
					*(meshIndexMap.ptr<int>(row)+col) = -2; // No depth value, need interpolation in the mesh
					mapIndicestoInterpolate.push_back(cvHelper::Mat2Pos(row, col));
				}
			} else {
				*(meshIndexMap.ptr<int>(row)+col) = -1; // Discontinuity in the shape
			}
		}
	}

	// Generate an empty texture mapping array
	//mesh.addTexCoords(std::vector<ofVec2f>(mesh.getNumVertices(), ofVec2f(0, 0)));

	// Interpolate missing depths
	cvHelper::Mat2Pos currentInterpolatedCell;
	cvHelper::Mat2Pos neighbour;
	int neighbourValue;
	cvHelper::Mat2Pos radiusSearch[] = { cvHelper::Mat2Pos(0, 1), cvHelper::Mat2Pos(1, 1), cvHelper::Mat2Pos(1, 0), cvHelper::Mat2Pos(1, -1),
		cvHelper::Mat2Pos(0, -1), cvHelper::Mat2Pos(-1, -1), cvHelper::Mat2Pos(-1, 0), cvHelper::Mat2Pos(-1, 1) };
	int loopingCounter =  -1;
	bool detectedLoop = false;
	while (!mapIndicestoInterpolate.empty() && !detectedLoop) {
		currentInterpolatedCell = mapIndicestoInterpolate.front();
		mapIndicestoInterpolate.pop_front();

		// Look in the 8 neighbours
		int i = 0;
		bool found = false;
		while (i < 8 && !found) {
			neighbour = currentInterpolatedCell + radiusSearch[i];
			if (neighbour.row > 0 && neighbour.row < meshRows && neighbour.col > 0 && neighbour.col < meshCols) {
				neighbourValue = *(meshIndexMap.ptr<int>(neighbour.row) + neighbour.col);
				if (neighbourValue >= 0){
					*(meshIndexMap.ptr<int>(currentInterpolatedCell.row) + currentInterpolatedCell.col) = neighbourValue;
					found = true;
					loopingCounter = -1;
				}
			}
			++i;
		}
		if (!found) {
			mapIndicestoInterpolate.push_back(currentInterpolatedCell);
			if (loopingCounter == -1) { // Changes have been made, reinit the infinite loop detection (isolated unknown blob)
				loopingCounter = mapIndicestoInterpolate.size();
			}
			else if (--loopingCounter == 0) {
				detectedLoop = true;
			}
		}
	}

	while (!mapIndicestoInterpolate.empty()) { // Class the eventual isolated unknown blob as a discontinuity
		currentInterpolatedCell = mapIndicestoInterpolate.front();
		mapIndicestoInterpolate.pop_front();
		*(meshIndexMap.ptr<int>(currentInterpolatedCell.row) + currentInterpolatedCell.col) = -1;
	}
	
	for (int i = 0; i < mapReferenceIndices.size(); ++i) {
		if (mapReferenceIndices[i].row < meshRows - 1 && mapReferenceIndices[i].col < meshCols - 1) { // Don't process on the last col and row
			int pointAIdx = *(meshIndexMap.ptr<int>(mapReferenceIndices[i].row) + mapReferenceIndices[i].col);
			int pointBIdx = *(meshIndexMap.ptr<int>(mapReferenceIndices[i].row) + mapReferenceIndices[i].col + 1);
			int pointCIdx = *(meshIndexMap.ptr<int>(mapReferenceIndices[i].row + 1) + mapReferenceIndices[i].col + 1);
			int pointDIdx = *(meshIndexMap.ptr<int>(mapReferenceIndices[i].row + 1) + mapReferenceIndices[i].col);

			if (pointAIdx != -1 && pointBIdx != -1 && pointDIdx != -1 && pointBIdx != pointAIdx) {
				mesh.addTriangle(pointAIdx, pointBIdx, pointDIdx);
			}

			if (pointBIdx != -1 && pointCIdx != -1 && pointDIdx != -1 && pointDIdx != pointCIdx) {
				mesh.addTriangle(pointBIdx, pointCIdx, pointDIdx);
			}
		}
	}
}

void ofApp::computeNormals(ofMesh& mesh, bool bNormalize){
	for (int i = 0; i < mesh.getVertices().size(); i++) mesh.addNormal(ofPoint(0, 0, 0));

	for (int i = 0; i < mesh.getIndices().size(); i += 3){
		const int ia = mesh.getIndices()[i];
		const int ib = mesh.getIndices()[i + 1];
		const int ic = mesh.getIndices()[i + 2];
		ofVec3f a = mesh.getVertices()[ia];
		ofVec3f b = mesh.getVertices()[ib];
		ofVec3f c = mesh.getVertices()[ic];

		ofVec3f e1 = a - b;
		ofVec3f e2 = c - b;
		ofVec3f no = e2;
		no.cross(e1);

		mesh.getNormals()[ia] += no;
		mesh.getNormals()[ib] += no;
		mesh.getNormals()[ic] += no;
	}

	if (bNormalize)
	for (int i = 0; i < mesh.getNormals().size(); i++) {
		mesh.getNormals()[i].normalize();
	}
}

void ofApp::meshParameterizationLSCM(ofMesh& mesh) {
	LSCM lscm(mesh);
	lscm.apply();
	float uRange = std::abs(lscm.umax - lscm.umin);
	float vRange = std::abs(lscm.vmax - lscm.vmin);
	float maxRange = std::max(uRange, vRange);
	ofVec2f outputRange(256 * uRange / maxRange, 256 * vRange / maxRange);

	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		mesh.setTexCoord(i, mapVec2f(mesh.getTexCoord(i), ofVec2f(lscm.umin, lscm.vmin), ofVec2f(lscm.umax, lscm.vmax), ofVec2f(0, 0), outputRange));
	}
}

ofVec2f ofApp::mapVec2f(ofVec2f value, ofVec2f inputMin, ofVec2f inputMax, ofVec2f outputMin, ofVec2f outputMax, bool clamp) {
	float x = ofMap(value.x, inputMin.x, inputMax.x, outputMin.x, outputMax.x, clamp);
	float y = ofMap(value.y, inputMin.y, inputMax.y, outputMin.y, outputMax.y, clamp);

	return ofVec2f(x, y);
}