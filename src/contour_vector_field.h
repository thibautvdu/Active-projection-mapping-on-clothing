#ifndef FLEXIBLE_SURFACE_AUGMENTATION_CONTOUR_VECTOR_FIELD_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_CONTOUR_VECTOR_FIELD_H_

#include "animation.h"

namespace garment_augmentation {
namespace garment {

	class ContourVectorField : public Animation {

		public  :
		
			ContourVectorField();
			~ContourVectorField();

			void Update() {};
			void Update(const std::vector<Fold> &folds) {};
			void Update(const std::vector<ofVec3f> &contour_points);
			void Draw();

			bool RelativeToModel() { return false; }

		private :
			void FillTbo(const std::vector<ofVec3f> &vector);

			ofVbo custom_vbo_;
			ofShader custom_shader_;
			GLuint position_attrib_idx_;
			GLuint texture_buffer_object_, texture_buffer_object_idx_, texture_buffer_;
			static const int k_max_contour_vertices_ = 3000;
			std::vector<ofVec3f> previous_contour_, current_contour_;

			bool initialized_contour_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_CONTOUR_VECTOR_FIELD_H_