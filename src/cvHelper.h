#ifndef FLEXIBLE_SURFACE_AUGMENTATION_CV_HELPER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_CV_HELPER_H_

#include <string>
#include "ofxCv.h"

namespace cvHelper {

	class Mat2Pos {
		public:
			int row, col;

			Mat2Pos() {
				row = 0;
				col = 0;
			}

			Mat2Pos(int row, int col) : row(row), col(col) {};

			inline Mat2Pos& operator=(const Mat2Pos& rhs) {
				if (&rhs == this)
					return *this;

				row = rhs.row;
				col = rhs.col;
				return *this;
			}

			inline Mat2Pos& operator+=(const Mat2Pos& rhs) {
				row += rhs.row;
				col += rhs.col;
				return *this;
			}

			inline Mat2Pos operator+(const Mat2Pos& rhs) {
				Mat2Pos copy = *this;
				copy += rhs;
				return copy;
			}
	};

	std::string getImageType(const cv::Mat& image);
	bool zeroAt(cv::Mat& mat, int row, int col);
}

#endif // FLEXIBLE_SURFACE_AUGMENTATION_CV_HELPER_H_