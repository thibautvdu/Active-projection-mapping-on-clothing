#include "ofSemiImplicitActiveMesh.h"

namespace ofDeformationTracking {

	// INITIALIZATION
	void ofSemiImplicitActiveMesh::generateMesh(const ofPolyline& imageContour) {
		this->clear();
		this->setMode(OF_PRIMITIVE_TRIANGLES);
		this->enableIndices();

		const ofRectangle roi = imageContour.getBoundingBox();

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

				if (intersectionArea(rectangleFace, imageContour) * 100 / rectangleFace.getArea() > mGenerationAreaThresh){
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
	}

	// COMPUTATION
	void ofSemiImplicitActiveMesh::updateMesh(const ofPolyline& imgContourRef, const ofxKinectCommonBridge& ofxKinect) {
		if (mNeedComputation) {
			computeSolver();
		}

		ofPolyline imgContour(imgContourRef);
		float depth;
		for (int i = 0; i < imgContour.size(); ++i) {
			depth = ofxKinect.getDepthAt((int)imgContour[i].x, (int)imgContour[i].y);
			if (abs(depth) > 0.1)
				imgContour[i].z = depth;
		}

		// Get closest points for each boundary vertex and construct the mesh contour
		ofPolyline meshImgContour;
		float* closestImgPtForceX = new float[this->getNumVertices()]; // 0 for non boundary vertices
		float* closestImgPtForceY = new float[this->getNumVertices()]; // 0 for non boundary vertices
		float* closestImgPtForceZ = new float[this->getNumVertices()]; // 0 for non boundary vertices
		ofVec3f closestImgContourPoint;
		for (int i = 0; i < this->getNumVertices(); ++i) { closestImgPtForceX[i] = 0; closestImgPtForceY[i] = 0; closestImgPtForceZ[i] = 0; }
		for (int i = 0; i < mMeshBoundaryIndices.size(); ++i) {
			closestImgContourPoint = imgContour.getClosestPoint(this->getVertex(mMeshBoundaryIndices[i]));
			closestImgPtForceX[mMeshBoundaryIndices[i]] = closestImgContourPoint.x - this->getVertex(mMeshBoundaryIndices[i]).x;
			closestImgPtForceY[mMeshBoundaryIndices[i]] = closestImgContourPoint.y - this->getVertex(mMeshBoundaryIndices[i]).y;

			if (abs(closestImgContourPoint.z) > 0.1) {
				closestImgPtForceZ[mMeshBoundaryIndices[i]] = closestImgContourPoint.z - this->getVertex(mMeshBoundaryIndices[i]).z;
			}

			meshImgContour.addVertex(this->getVertex(mMeshBoundaryIndices[i]));
		}
		meshImgContour.close();

		// Closests vertices for every image boundary point
		unsigned int nearestIdx; ofVec3f nearestPoint;
		float* imgPtToMeshContourForceX = new float[this->getNumVertices()]; // 0 for non boundary vertices
		float* imgPtToMeshContourForceY = new float[this->getNumVertices()]; // 0 for non boundary vertices
		float* imgPtToMeshContourForceZ = new float[this->getNumVertices()]; // 0 for non boundary vertices
		for (int i = 0; i < this->getNumVertices(); ++i) { imgPtToMeshContourForceX[i] = 0; imgPtToMeshContourForceY[i] = 0; imgPtToMeshContourForceZ[i] = 0; }
		for (int i = 0; i < imgContour.size(); ++i) {
			nearestPoint = meshImgContour.getClosestPoint(imgContour[i], &nearestIdx);
			nearestIdx = mMeshBoundaryIndices[nearestIdx];
			imgPtToMeshContourForceX[nearestIdx] += imgContour[i].x - nearestPoint.x;
			imgPtToMeshContourForceY[nearestIdx] += imgContour[i].y - nearestPoint.y;

			if (abs(imgContour[i].z) > 0.1) {
				imgPtToMeshContourForceZ[nearestIdx] += imgContour[i].z - nearestPoint.z;
			}
		}


		Eigen::VectorXd bX(this->getNumVertices());
		for (int i = 0; i < this->getNumVertices(); ++i) {

			if (mIsBoundaryVertices[i]) {
				bX[i] = mAdaptationRate*this->getVertex(i).x + mBoundaryWeight*(closestImgPtForceX[i] +imgPtToMeshContourForceX[i]);
			}
			else {
				bX[i] = mAdaptationRate*this->getVertex(i).x;
			}
		}

		Eigen::VectorXd bY(this->getNumVertices());
		for (int i = 0; i < this->getNumVertices(); ++i) {

			if (mIsBoundaryVertices[i]) {
				bY[i] = mAdaptationRate*this->getVertex(i).y + mBoundaryWeight*(closestImgPtForceY[i] + imgPtToMeshContourForceY[i]);
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
		float newDepth = 0;
		for (int i = 0; i < this->getNumVertices(); ++i) {

			if (mIsBoundaryVertices[i]) {
				bZ[i] = mAdaptationRate*this->getVertex(i).z + (mBoundaryWeight + mDepthWeight)*(closestImgPtForceZ[i] + imgPtToMeshContourForceZ[i]);
			}
			else {
				newDepth = ofxKinect.getDepthAt(X[i], Y[i]);

				if (abs(newDepth) > 0.1) {
					bZ[i] = mAdaptationRate*this->getVertex(i).z + mDepthWeight*(newDepth - this->getVertex(i).z);
				}
			}
		}

		Eigen::VectorXd Z = mpSolver->solve(bZ);
		if (!mpSolver->info() == Eigen::Success) {
			ofLogError("ofSemiImplicitActiveMesh::generateMesh") << "Couldn't solve bZ";
		}

		delete[] closestImgPtForceX; delete[] closestImgPtForceY; delete[] closestImgPtForceZ;
		delete[] imgPtToMeshContourForceX; delete[] imgPtToMeshContourForceY; delete[] imgPtToMeshContourForceZ;

		for (int i = 0; i < this->getNumVertices(); ++i) {
			this->setVertex(i, ofVec3f(X[i], Y[i], Z[i]));
		}
	}
}