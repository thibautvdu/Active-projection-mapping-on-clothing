#include "cv_helper.h"

namespace cv_helper {

	std::string GetImageType(const cv::Mat& image) {
		// find type
		int imgTypeInt = image.type() % 8;
		std::string imgTypeString;

		switch (imgTypeInt) {
		case 0:
			imgTypeString = "8U";
			break;
		case 1:
			imgTypeString = "8S";
			break;
		case 2:
			imgTypeString = "16U";
			break;
		case 3:
			imgTypeString = "16S";
			break;
		case 4:
			imgTypeString = "32S";
			break;
		case 5:
			imgTypeString = "32F";
			break;
		case 6:
			imgTypeString = "64F";
			break;
		default:
			break;
		}

		// find channel
		int channel = (image.type() / 8) + 1;

		std::stringstream type;
		type << "CV_" << imgTypeString << "C" << channel;

		return type.str();
	}

}