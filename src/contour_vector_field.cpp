#include "contour_vector_field.h"

namespace garment_augmentation {
namespace garment {

	ContourVectorField::ContourVectorField() {
		initialized_contour_ = false;

		custom_shader_.load("shaders/contour_vector_field.vert", "shaders/contour_vector_field.frag", "shaders/contour_vector_field.geom");
		position_attrib_idx_ = custom_shader_.getAttributeLocation("position");

		custom_vbo_.setAttributeData(position_attrib_idx_, NULL, 3, k_max_contour_vertices_, GL_DYNAMIC_DRAW);

		// TBO setting
		texture_buffer_object_idx_ = custom_shader_.getAttributeLocation("contour");
		glGenBuffers(1, &texture_buffer_object_);
		glBindBuffer(GL_TEXTURE_BUFFER, texture_buffer_object_);
		glBufferData(GL_TEXTURE_BUFFER, k_max_contour_vertices_ * 3 * sizeof(GLfloat), NULL, GL_DYNAMIC_DRAW);
		glGenTextures(1, &texture_buffer_);
		glBindTexture(GL_TEXTURE_BUFFER, texture_buffer_);
		glTexBuffer(GL_TEXTURE_BUFFER, GL_RGB32F, texture_buffer_object_);
		glBindTexture(GL_TEXTURE_BUFFER, 0);
		glBindBuffer(GL_TEXTURE_BUFFER, 0);
	}

	ContourVectorField::~ContourVectorField() {
		glBindTexture(GL_TEXTURE_BUFFER, texture_buffer_);
		glDeleteTextures(1, &texture_buffer_);
		texture_buffer_ = 0;
		glBindTexture(GL_TEXTURE_BUFFER, 0);

		glBindBuffer(GL_TEXTURE_BUFFER, texture_buffer_object_);
		glDeleteBuffers(1, &texture_buffer_object_);
		texture_buffer_object_ = 0;
		glBindBuffer(GL_TEXTURE_BUFFER, 0);
	}

	void ContourVectorField::Update(const std::vector<ofVec3f> &contour_points) {
		if (contour_points.size() > k_max_contour_vertices_) {
			ofLogFatalError("ContourVectorField::Update") << "Contour size exceed shader limitations";
			return;
		}

		if (contour_points.empty()) {
			ofLogError("ContourVectorField::Update") << "Received an empty contour";
			return;
		}

		if (!initialized_contour_) {
			current_contour_ = contour_points;
			initialized_contour_ = true;
		}
		else {
			std::swap(previous_contour_,current_contour_); // save the last current contour as the previous contour

			current_contour_ = contour_points;

			custom_vbo_.updateAttributeData(position_attrib_idx_, &current_contour_[0].x, current_contour_.size());
			FillTbo(previous_contour_);
		}
	}

	void ContourVectorField::Draw() {
		if (initialized_contour_) {
			custom_shader_.begin();

			custom_shader_.setUniform1i("contour_size", previous_contour_.size());

			custom_vbo_.bind();
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_BUFFER, texture_buffer_);
				glDrawArrays(GL_POINTS, 0, current_contour_.size());
			glBindTexture(GL_TEXTURE_BUFFER, 0);
			glBindBuffer(GL_TEXTURE_BUFFER, 0);
			custom_vbo_.unbind();

			custom_shader_.end();
		}
	}

	void ContourVectorField::FillTbo(const std::vector<ofVec3f> &vector) {
		glBindBuffer(GL_TEXTURE_BUFFER, texture_buffer_object_);

		GLfloat* p = (GLfloat*)glMapBuffer(GL_TEXTURE_BUFFER, GL_WRITE_ONLY);

		for (int i = 0; i < vector.size(); ++i) {
			p[i * 3] = vector[i].x;
			p[i * 3 + 1] = vector[i].y;
			p[i * 3 + 2] = vector[i].z;
		}

		glUnmapBuffer(GL_TEXTURE_BUFFER);

		glBindBuffer(GL_TEXTURE_BUFFER, 0);
	}

} // namespace garment
} // namespace garment_augmentation