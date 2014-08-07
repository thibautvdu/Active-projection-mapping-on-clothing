#include <stdlib.h>
#include "ofApp.h"
#include "cvHelper.h"

//--------------------------------------------------------------
void ofApp::setup() {
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

	mOfSegmentedImg.allocate(mKinectColorImgWidth, mKinectColorImgHeight, OF_IMAGE_COLOR_ALPHA);
	mCvSegmentedImg = ofxCv::toCv(mOfSegmentedImg);

	ofDisableAlphaBlending();
	ofSetFrameRate(30);
}

//--------------------------------------------------------------
void ofApp::update() {
	mOfxKinect.update();
	
	if (mOfxKinect.isNewSkeleton()) {
		NUI_DEPTH_IMAGE_PIXEL * pNuiDepthPixel = mOfxKinect.getNuiMappedDepthPixelsRef(); // The kinect segmentation map
		
		mOfSegmentedImg.setColor(ofColor::black);
		int modelPixelCount = 0;
		ofPoint modelRoiPointA(mOfSegmentedImg.width, mOfSegmentedImg.height);
		ofPoint modelRoiPointD(0, 0);
		for (int x = 0; x < mOfSegmentedImg.width; ++x) {
			for (int y = 0; y < mOfSegmentedImg.height; ++y) {
				USHORT playerId = (pNuiDepthPixel + x + y*mOfSegmentedImg.width)->playerIndex;

				if (playerId != 0) {
					mOfSegmentedImg.setColor(x, y, mOfxKinect.getColorPixelsRef().getColor(x,y));
					modelPixelCount++;
					// Update the ROI
					modelRoiPointA.x = (x < modelRoiPointA.x) ? x : modelRoiPointA.x;
					modelRoiPointA.y = (y < modelRoiPointA.y) ? y : modelRoiPointA.y;
					modelRoiPointD.x = (x > modelRoiPointD.x) ? x : modelRoiPointD.x;
					modelRoiPointD.y = (y > modelRoiPointD.y) ? y : modelRoiPointD.y;
				}
			}
		}

		// OPENCV
		/*if (mCvSegmentedImage.type() != CV_8UC4) {
		ofLog(OF_LOG_FATAL_ERROR) << "Expected the kinect's color image to be 8 bytes x 4 channels";
		exit();
		}*/
		cv::Mat mCvSegmentedImgRoi = mCvSegmentedImg(cv::Rect(cv::Point(modelRoiPointA.x, modelRoiPointA.y), cv::Point(modelRoiPointD.x, modelRoiPointD.y)));
		// K-means clustering
		if (modelPixelCount != 0) {
			cv::Mat cvKmeansData = cv::Mat::zeros(modelPixelCount, 5, CV_32F);
			int index = 0;
			cv::Vec4b bgraColor;
			for (int row = 0; row < mCvSegmentedImgRoi.rows; ++row) {
				cv::Vec4b* pRow = mCvSegmentedImgRoi.ptr<cv::Vec4b>(row);
				for (int col = 0; col < mCvSegmentedImgRoi.cols; ++col) {
					if (pRow[col] != cv::Vec4b(0, 0, 0, 255)) {
						bgraColor = mCvSegmentedImg.at<cv::Vec4b>(row, col);
						cvKmeansData.at<float>(index, 0) = row / (float)mCvSegmentedImgRoi.rows;
						cvKmeansData.at<float>(index, 1) = col / (float)mCvSegmentedImgRoi.cols;
						cvKmeansData.at<float>(index, 2) = bgraColor[0] / 255.f;
						cvKmeansData.at<float>(index, 3) = bgraColor[1] / 255.f;
						cvKmeansData.at<float>(index, 4) = bgraColor[2] / 255.f;
						index++;
					}
				}
			}
			cv::Mat intOutLabels;
			cv::kmeans(cvKmeansData, 3, intOutLabels, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.1), 1, cv::KMEANS_RANDOM_CENTERS);

			index = 0;
			for (int row = 0; row < mCvSegmentedImgRoi.rows; ++row) {
				cv::Vec4b* pRow = mCvSegmentedImgRoi.ptr<cv::Vec4b>(row);
				for (int col = 0; col < mCvSegmentedImgRoi.cols; ++col) {
					if (pRow[col] != cv::Vec4b(0, 0, 0, 255)) {
						int clusterId = intOutLabels.at<int>(index, 0);
						switch (clusterId) {
						case 0:
							pRow[col] = cv::Vec4b(255, 0, 0, 0);
							break;
						case 1:
							pRow[col] = cv::Vec4b(0, 255, 0, 0);
							break;
						case 2:
							pRow[col] = cv::Vec4b(0, 0, 255, 0);
							break;
						}
						index++;
					}
				}
			}
			mOfSegmentedImg.update();
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (mOfxKinect.isFrameNew()) {
		ofBackground(0);

		mOfxKinect.draw(0, 0);
		mOfxKinect.drawDepth(640, 0);

		if (mOfSegmentedImg.isAllocated()) {
			mOfSegmentedImg.draw(0, 480);
		}
	}
}

//--------------------------------------------------------------
void ofApp::exit() {
	mOfxKinect.stop();
}