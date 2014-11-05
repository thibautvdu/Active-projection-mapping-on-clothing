#ifndef FLEXIBLE_SURFACE_AUGMENTATION_FOLD_VIDEO_TEXTURE_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_FOLD_VIDEO_TEXTURE_H_

#include "animation.h"
#include "fold.h"

namespace garment_augmentation {
namespace garment {

class FoldVideoTexture : public Animation {
	public:

		FoldVideoTexture(const char* file_path) {
			video_player_.loadMovie(file_path);
			width_height_ratio_ = 0;
		}

		void Update();
		void Update(const std::vector<Fold> &folds);
		void Update(const std::vector<ofVec3f> &contour_points) {};
		void Draw();

		inline bool RelativeToModel() { return true; }

	private:
		ofVideoPlayer video_player_;
		float width_height_ratio_;
		ofMesh fold_drawing_box_;
};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_FOLD_VIDEO_TEXTURE_H_