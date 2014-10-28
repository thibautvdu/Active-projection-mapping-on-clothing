#ifndef FLEXIBLE_SURFACE_AUGMENTATION_CONTOUR_VECTOR_FIELD_H_
#define FLEXIBLE_SURFACE_AUGMENTATION_CONTOUR_VECTOR_FIELD_H_

#include "animation.h"
#include "ofxUbo.h"

namespace garment_augmentation {
namespace garment {

	class ContourVectorField : public Animation {

		public  :
		
			ContourVectorField();

			void Update() {};
			void Update(const std::vector<Fold> &folds) {};
			void Update(const std::vector<ofVec3f> &contour_points);
			void Draw();

		private :
			ofVbo custom_vbo_;
			ofxUboShader custom_ubo_shader_;
			GLuint position_attrib_idx_;
			static const int k_max_contour_vertices_ = 1000; // Must also be changed in the geometry shader !
			struct ContourBlock {
				ofVec3f data[k_max_contour_vertices_];
				int size;
			};
			struct ContourBlock previous_contour_, current_contour_;

			bool stored_a_previous_contour_;
	};

} // namespace garment
} // namespace garment_augmentation

#endif // FLEXIBLE_SURFACE_AUGMENTATION_CONTOUR_VECTOR_FIELD_H_