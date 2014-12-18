// A few helper functions when developping with openCV

#ifndef FLEXIBLE_SURFACE_AUGMENTATION_CV_HELPER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_CV_HELPER_H_

#include <string>
#include "ofxCv.h"

namespace cv_helper {
	std::string GetImageType(const cv::Mat& image);
}

#endif // FLEXIBLE_SURFACE_AUGMENTATION_CV_HELPER_H_