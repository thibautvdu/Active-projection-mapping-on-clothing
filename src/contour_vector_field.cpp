#include "contour_vector_field.h"

namespace garment_augmentation {
namespace garment {

	ContourVectorField::ContourVectorField() {
		stored_a_previous_contour_ = false;

		custom_ubo_shader_.load("shaders/contour_vector_field.vert", "shaders/contour_vector_field.frag", "shaders/contour_vector_field.geom");
		position_attrib_idx_ = custom_ubo_shader_.getAttributeLocation("position");

		custom_vbo_.setAttributeData(position_attrib_idx_, NULL, 3, k_max_contour_vertices_, GL_DYNAMIC_DRAW);
	}

	void ContourVectorField::Update(const std::vector<ofVec3f> &contour_points) {
		if (contour_points.size() > k_max_contour_vertices_) {
			ofLogFatalError("ContourVectorField::Update") << "Contour size exceed shader limitations";
			return;
		}

		if (!stored_a_previous_contour_) {
			previous_contour_.size = contour_points.size();
			for (int i = 0; i < previous_contour_.size; ++i) {
				previous_contour_.data[i] = contour_points[i];
				current_contour_.data[i] = previous_contour_.data[i];
			}
			stored_a_previous_contour_ = true;
		}
		else {
			std::swap(previous_contour_.data,current_contour_.data); // save the last current contour as the previous contour
			previous_contour_.size = current_contour_.size;

			current_contour_.size = contour_points.size();
			for (int i = 0; i < current_contour_.size; ++i) {
				current_contour_.data[i] = contour_points[i];
			}

			custom_vbo_.updateAttributeData(position_attrib_idx_, &current_contour_.data[0].x, current_contour_.size);
		}
	}

	void ContourVectorField::Draw() {
		custom_ubo_shader_.begin();

		custom_ubo_shader_.setUniformBuffer("ContourBlock", previous_contour_);

		custom_vbo_.bind();
			glDrawArrays(GL_POINTS, 0, current_contour_.size);
		custom_vbo_.unbind();

		custom_ubo_shader_.end();
	}

} // namespace garment
} // namespace garment_augmentation