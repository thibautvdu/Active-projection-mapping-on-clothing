#include "ofSemiImplicitActiveMesh.h"
#include "ofxCv.h"

namespace ofDeformationTracking {

	// INITIALIZATION
	void ofSemiImplicitActiveMesh::generateMesh(const ofPolyline& imgContour, ofPixels contourMask, const ofxKinectCommonBridge& ofxKinect) {
		this->clear();
		this->setMode(OF_PRIMITIVE_TRIANGLES);
		this->enableIndices();

		const ofRectangle roi = imgContour.getBoundingBox();

		int rows = mMeshYResolution;
		int columns = mMeshXResolution;
		float stepY = roi.height / (float)(rows-1);
		float stepX = roi.width / (float)(columns-1);

		int* indicesUsed = new int[rows*columns];
		int* kHelperMap = new int[rows*columns];

		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < columns; col++) {
				int index = row*rows + col;

				this->addVertex(ofVec3f(col * stepX + roi.getTopLeft().x, row * stepY + roi.getTopLeft().y, 0));
				this->addColor(ofColor::blue);
				indicesUsed[index] = 0;
				kHelperMap[index] = index;
			}
		}

		for (int row = 0; row < rows - 1; row++) {
			for (int col = 0; col < columns - 1; col++) {

				// The square face
				ofVec3f pointA = this->getVertex((row)*columns + col);
				ofVec3f pointB = this->getVertex((row)*columns + col + 1);
				ofVec3f pointC = this->getVertex((row + 1)*columns + col + 1);
				ofVec3f pointD = this->getVertex((row + 1)*columns + col);
				ofPolyline rectangleFace;
				rectangleFace.addVertex(pointA);
				rectangleFace.addVertex(pointB);
				rectangleFace.addVertex(pointC);
				rectangleFace.addVertex(pointD);
				rectangleFace.close();

				if (intersectionArea(rectangleFace, imgContour) * 100 / rectangleFace.getArea() > mGenerationAreaThresh){
					// Triangle face 1
					this->addTriangle((row)*columns + col, (row)*columns + col + 1, (row + 1)*columns + col);

					// Triangle face 2
					this->addTriangle((row)*columns + col + 1, (row + 1)*columns + col + 1, (row + 1)*columns + col);

					indicesUsed[(row)*columns + col]++;
					indicesUsed[(row)*columns + col + 1]++;
					indicesUsed[(row + 1)*columns + col + 1]++;
					indicesUsed[(row + 1)*columns + col]++;
				}
			}
		}

		// Remove the eventual unused vertices and fill the boundary vertices vector
		mIsBoundaryVertices.resize(this->getNumVertices());
		for (int i = 0; i < this->getNumVertices(); ++i) {
			mIsBoundaryVertices[i] = false;
		}

		for (int i = this->getNumVertices() - 1; i >= 0; --i) {
			if (indicesUsed[i] == 0) {
				this->removeVertex(i);
				kHelperMap[i] = -1;
				this->removeColor(i);
				mIsBoundaryVertices.erase(mIsBoundaryVertices.begin() + i);

				for (int j = 0; j < this->getNumIndices(); ++j) {
					if (this->getIndex(j) > i) {
						this->setIndex(j, this->getIndex(j) - 1);
					}
				}
				for (int k = i+1; k < rows*columns; ++k) {
					kHelperMap[k]--;
				}
			}
			else if(indicesUsed[i] <= 3 ){
				mIsBoundaryVertices[i] = true;
				this->setColor(i, ofColor::red);
			}
		}
		delete[] indicesUsed;

		mKinectRelativeMesh = ofMesh(*this);

		for (int i = 0; i < mIsBoundaryVertices.size(); ++i) {
			if (mIsBoundaryVertices[i])
				mMeshBoundaryIndices.push_back(i);
		}

		// Fill the K matrix
		std::vector<Eigen::Triplet<double> > kValues;
		kValues.reserve((this->getNumVertices() - mIsBoundaryVertices.size()) * 9);
		int index = -1;
		for (int row = 2; row < rows; row++) {
			for (int col = 0; col < columns - 2; col++) {
				++index;
				if (kHelperMap[row*rows + col] > -1 && kHelperMap[(row - 1)*rows + col] > -1 && kHelperMap[(row - 2)*rows + col] > -1) {
					kValues.push_back(Eigen::Triplet<double>(index, kHelperMap[row*rows + col], 1));
					kValues.push_back(Eigen::Triplet<double>(index, kHelperMap[(row - 1)*rows + col], -2));
					kValues.push_back(Eigen::Triplet<double>(index, kHelperMap[(row - 2)*rows + col], 1));
				}

				++index;
				if (kHelperMap[row*rows + col] > -1 && kHelperMap[row*rows + col + 1] > -1 && kHelperMap[row*rows + col + 2] > -1) {
					kValues.push_back(Eigen::Triplet<double>(index, kHelperMap[row*rows + col], 1));
					kValues.push_back(Eigen::Triplet<double>(index, kHelperMap[row*rows + col + 1], -2));
					kValues.push_back(Eigen::Triplet<double>(index, kHelperMap[row*rows + col + 2], 1));
				}

				++index;
				if (kHelperMap[row*rows + col] > -1 && kHelperMap[(row - 1)*rows + col + 1] > -1 && kHelperMap[(row - 2)*rows + col + 2] > -1) {
					kValues.push_back(Eigen::Triplet<double>(index, kHelperMap[row*rows + col], 1));
					kValues.push_back(Eigen::Triplet<double>(index, kHelperMap[(row - 1)*rows + col + 1], -2));
					kValues.push_back(Eigen::Triplet<double>(index, kHelperMap[(row - 2)*rows + col + 2], 1));
				}
			}
		}
		delete[] kHelperMap;

		Eigen::SparseMatrix<double, Eigen::ColMajor> KCols(index+1, this->getNumVertices());
		KCols.setFromTriplets(kValues.begin(), kValues.end());

		mK = KCols.transpose()*KCols;

		computeSolver();

		// Initialize the depths to an average value
		ofShortPixels depthPixels = ofxKinect.getDepthPixels();
		cv::Mat cvDepthPixelsRoi = ofxCv::toCv(depthPixels)(ofxCv::toCv(roi));
		cv::Mat cvContourMaskRoi = ofxCv::toCv(contourMask)(ofxCv::toCv(roi));
		USHORT *p_row, *p_maskRow;
		int nbValues = 0;
		ULONG sumDepth = 0;
		for (int row = 0; row < cvDepthPixelsRoi.rows; row++) {
			p_row = cvDepthPixelsRoi.ptr<USHORT>(row);
			p_maskRow = cvContourMaskRoi.ptr<USHORT>(row);
			for (int col = 0; col < cvDepthPixelsRoi.cols; col++) {
				if (*(p_maskRow + col) != 0 && *(p_row + col) != 0) {
					nbValues++;
					sumDepth += *(p_row + col);
				}
			}
		}
		
		m_meshAvgDepth = sumDepth / nbValues;

		for (int i = 0; i < this->getNumVertices(); ++i) {
			this->setVertex(i, ofVec3f(this->getVertex(i).x, this->getVertex(i).y, m_meshAvgDepth));
		}

		mGenerated = true;
	}

	// Optimize the process with the more little bounding rect
	int ofSemiImplicitActiveMesh::intersectionArea(const ofPolyline& contour1, const ofPolyline& contour2) {
		int area = 0;

		ofRectangle boundingBox1 = contour1.getBoundingBox();
		ofRectangle boundingBox2 = contour2.getBoundingBox();

		ofRectangle roi;
		if (boundingBox1.getArea() < boundingBox2.getArea())
			roi = boundingBox1;
		else
			roi = boundingBox2;

		for (int y = roi.getTopLeft().y; y < roi.height + roi.getTopLeft().y; ++y){
			for (int x = roi.getTopLeft().x; x < roi.width + roi.getTopLeft().x; ++x){
				if (contour1.inside(x, y) && contour2.inside(x, y))
					area++;
			}
		}

		return area;
	}


	void ofSemiImplicitActiveMesh::computeSolver() {
		Eigen::SparseMatrix<double, Eigen::ColMajor> identity(this->getNumVertices(), this->getNumVertices());
		identity.setIdentity();

		mA = mK + mAdaptationRate * identity;
		mpSolver->compute(mA);
		if (mpSolver->info() != Eigen::Success) {
			ofLogError("ofSemiImplicitActiveMesh::generateMesh") << "Couldn't decompose A matrix for the sparse linear solver";
		}

		mNeedComputation = false;
	}

	// COMPUTATION
	void ofSemiImplicitActiveMesh::updateMesh(const ofPolyline& imgContour, ofPixels contourMask, const ofxKinectCommonBridge& ofxKinect) {
		if (mNeedComputation) {
			computeSolver();
		}

		// Left to right flooding on the ROI
		ofRectangle roi = imgContour.getBoundingBox();
		ofShortPixels depthPixelsFilled = ofxKinect.getDepthPixels();
		int depthW = depthPixelsFilled.getWidth(); int depthH = depthPixelsFilled.getHeight();

		USHORT lastValid;
		int startX = min((int)roi.getBottomRight().x + 1, depthW - 1);
		for (int y = roi.getTopLeft().y; y < roi.getBottomRight().y; ++y) {
			lastValid = 0;
			for (int x = startX; x >= 0; --x){
				lastValid = depthPixelsFilled[y*depthW + x] = depthPixelsFilled[y*depthW + x] == 0 ? lastValid : depthPixelsFilled[y*depthW + x];
			}
		}

		ofPolyline depthContour;
		USHORT depth;
		for (int i = 0; i < imgContour.size(); ++i) {
			depth = depthPixelsFilled[(int)imgContour[i].y*depthPixelsFilled.getWidth() + (int)imgContour[i].x];
			depthContour.addVertex(imgContour[i].x, imgContour[i].y, (float)depth);
		}
		depthContour.close();

		// Get closest points for each boundary vertex and construct the mesh contour
		ofPolyline meshDepthContour;
		ofVec3f* closestImgPtForce = new ofVec3f[this->getNumVertices()]; // 0 for non boundary vertices
		unsigned int nearestIdx;
		for (int i = 0; i < this->getNumVertices(); ++i) { closestImgPtForce[i] = ofVec3f::zero(); }
		for (int i = 0; i < mMeshBoundaryIndices.size(); ++i) {
			imgContour.getClosestPoint(ofVec3f(this->getVertex(mMeshBoundaryIndices[i]).x, this->getVertex(mMeshBoundaryIndices[i]).y,0),&nearestIdx);
			closestImgPtForce[mMeshBoundaryIndices[i]] = depthContour[nearestIdx] - this->getVertex(mMeshBoundaryIndices[i]);

			meshDepthContour.addVertex(this->getVertex(mMeshBoundaryIndices[i]));
		}
		meshDepthContour.close();

		// Closests vertices for every image boundary point
		ofVec3f nearestPoint;
		ofVec3f* imgPtToMeshContourForce = new ofVec3f[this->getNumVertices()]; // 0 for non boundary vertices
		for (int i = 0; i < this->getNumVertices(); ++i) { imgPtToMeshContourForce[i] = ofVec3f::zero(); }
		for (int i = 0; i < imgContour.size(); ++i) {
			nearestPoint = meshDepthContour.getClosestPoint(depthContour[i], &nearestIdx);
			nearestIdx = mMeshBoundaryIndices[nearestIdx];
			imgPtToMeshContourForce[nearestIdx] += depthContour[i] - nearestPoint;
		}


		Eigen::VectorXd bX(this->getNumVertices());
		for (int i = 0; i < this->getNumVertices(); ++i) {

			if (mIsBoundaryVertices[i]) {
				bX[i] = mAdaptationRate*this->getVertex(i).x + mBoundaryWeight*(closestImgPtForce[i].x +imgPtToMeshContourForce[i].x);
			}
			else {
				bX[i] = mAdaptationRate*this->getVertex(i).x;
			}
		}

		Eigen::VectorXd bY(this->getNumVertices());
		for (int i = 0; i < this->getNumVertices(); ++i) {

			if (mIsBoundaryVertices[i]) {
				bY[i] = mAdaptationRate*this->getVertex(i).y + mBoundaryWeight*(closestImgPtForce[i].y + imgPtToMeshContourForce[i].y);
			}
			else {
				bY[i] = mAdaptationRate*this->getVertex(i).y;
			}
		}

		Eigen::VectorXd X, Y;
		X = mpSolver->solve(bX);
		if (!mpSolver->info() == Eigen::Success) {
			ofLogError("ofSemiImplicitActiveMesh::generateMesh") << "Couldn't solve bX";
		}
		Y = mpSolver->solve(bY);
		if (!mpSolver->info() == Eigen::Success) {
			ofLogError("ofSemiImplicitActiveMesh::generateMesh") << "Couldn't solve bY";
		}
		
		Eigen::VectorXd bZ(this->getNumVertices());
		USHORT newDepth = 0;
		ofVec3f projectedPoint;
		for (int i = 0; i < this->getNumVertices(); ++i) {
			bZ[i] = mAdaptationRate*this->getVertex(i).z;

			if (mIsBoundaryVertices[i]) {
				bZ[i] += mBoundaryWeight*(closestImgPtForce[i].z + imgPtToMeshContourForce[i].z);
			}

			newDepth = depthPixelsFilled[(int)Y[i] * depthPixelsFilled.getWidth() + (int)X[i]];
			if (newDepth != 0 && abs(newDepth - m_meshAvgDepth) < m_maxDepthDiff) {
				bZ[i] += mDepthWeight*((float)newDepth - this->getVertex(i).z);
			}
		}

		Eigen::VectorXd Z = mpSolver->solve(bZ);
		if (!mpSolver->info() == Eigen::Success) {
			ofLogError("ofSemiImplicitActiveMesh::generateMesh") << "Couldn't solve bZ";
		}

		delete[] closestImgPtForce;
		delete[] imgPtToMeshContourForce;

		m_minDepth = Z[0];
		m_maxDepth = Z[0];
		for (int i = 0; i < this->getNumVertices(); ++i) {
			this->setVertex(i, ofVec3f(X[i], Y[i], Z[i]));
			m_minDepth = min((float)Z[i], m_minDepth);
			m_maxDepth = max((float)Z[i], m_maxDepth);
		}
	}

	void ofSemiImplicitActiveMesh::computeWorldMesh(const ofxKinectCommonBridge &ofxKinect) {
		ofColor depthColor;
		for (int i = 0; i < mKinectRelativeMesh.getNumVertices(); ++i) {
			mKinectRelativeMesh.setVertex(i, ofxKinect.getWorldCoordinates((int)this->getVertex(i).x, (int)this->getVertex(i).y, this->getVertex(i).z));
			if (m_minDepth != m_maxDepth) {
				depthColor = ofColor(ofMap(this->getVertex(i).z, m_minDepth, m_minDepth + (m_maxDepth - m_minDepth) / 2, 0, 255, true), ofMap(this->getVertex(i).z, m_minDepth + (m_maxDepth - m_minDepth) / 2, m_maxDepth, 0, 255, true), 255);
			}
			mKinectRelativeMesh.setColor(i, depthColor);
		}
	}
}