#ifndef FLEXIBLE_SURFACE_AUGMENTATION_CV_HELPER_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_CV_HELPER_H_

#include <string>
#include "ofxCv.h"

namespace cvHelper {

	std::string getImageType(const cv::Mat& image);

}

#endif // FLEXIBLE_SURFACE_AUGMENTATION_CV_HELPER_H_