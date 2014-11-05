#include "fold_video_texture.h"

namespace garment_augmentation {
namespace garment {

	void FoldVideoTexture::Update() {
		if (!video_player_.isPlaying() && video_player_.isLoaded()) {
			video_player_.play();
			width_height_ratio_ = video_player_.width / video_player_.height;
		}
		video_player_.update();
	}

	void FoldVideoTexture::Update(const std::vector<Fold> &folds) {
		if (!folds.empty()) {
			fold_drawing_box_.clear();
			fold_drawing_box_.setMode(OF_PRIMITIVE_TRIANGLES);

			float video_world_width = width_height_ratio_ * folds[0].GetLength(); 
			float offset = video_world_width * 0.4 / 4; // 0.4 : % of the video width we wanna display
			ofVec3f middle = folds[0].GetMiddle();

			fold_drawing_box_.addVertex(folds[0].a_pt() - ofVec3f(offset, 0, 0));
			fold_drawing_box_.addTexCoord(ofVec2f(0, 0.3));

			fold_drawing_box_.addVertex(folds[0].a_pt() + ofVec3f(offset, 0, 0));
			fold_drawing_box_.addTexCoord(ofVec2f(0, 0.7));

			fold_drawing_box_.addVertex(middle - ofVec3f(offset, 0, 0));
			fold_drawing_box_.addTexCoord(ofVec2f(1, 0.3));

			fold_drawing_box_.addVertex(middle + ofVec3f(offset, 0, 0));
			fold_drawing_box_.addTexCoord(ofVec2f(1, 0.7));

			fold_drawing_box_.addVertex(middle - ofVec3f(offset, 0, 0));
			fold_drawing_box_.addTexCoord(ofVec2f(0, 0.3));

			fold_drawing_box_.addVertex(middle + ofVec3f(offset, 0, 0));
			fold_drawing_box_.addTexCoord(ofVec2f(0, 0.7));

			fold_drawing_box_.addVertex(folds[0].b_pt() - ofVec3f(offset, 0, 0));
			fold_drawing_box_.addTexCoord(ofVec2f(1, 0.3));

			fold_drawing_box_.addVertex(folds[0].b_pt() + ofVec3f(offset, 0, 0));
			fold_drawing_box_.addTexCoord(ofVec2f(1, 0.7));

			fold_drawing_box_.addIndex(0);
			fold_drawing_box_.addIndex(1);
			fold_drawing_box_.addIndex(2);
			fold_drawing_box_.addIndex(2);
			fold_drawing_box_.addIndex(1);
			fold_drawing_box_.addIndex(3);

			fold_drawing_box_.addIndex(4);
			fold_drawing_box_.addIndex(5);
			fold_drawing_box_.addIndex(6);
			fold_drawing_box_.addIndex(6);
			fold_drawing_box_.addIndex(5);
			fold_drawing_box_.addIndex(7);
		}
	}

	void FoldVideoTexture::Draw() {
		ofTexture &tex = video_player_.getTextureReference();
		tex.bind();

		ofSetMatrixMode(OF_MATRIX_TEXTURE);
		ofPushMatrix();
		ofLoadIdentityMatrix();

		ofTextureData texData = tex.getTextureData();
		if (texData.textureTarget == GL_TEXTURE_RECTANGLE_ARB) {
			ofScale(tex.getWidth(), tex.getHeight(), 1.0f);
		}
		else {
			ofScale(tex.getWidth() / texData.tex_w, tex.getHeight() / texData.tex_h, 1.0f);
		}

		ofSetMatrixMode(OF_MATRIX_MODELVIEW);
		fold_drawing_box_.draw();

		tex.unbind();

		ofSetMatrixMode(OF_MATRIX_TEXTURE);
		ofPopMatrix();
		ofSetMatrixMode(OF_MATRIX_MODELVIEW);
	}

} // namespace garment
} // namespace garment_augmentation