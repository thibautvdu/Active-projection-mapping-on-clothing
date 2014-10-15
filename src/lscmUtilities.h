#ifndef FLEXIBLE_SURFACE_AUGMENTATION_LSCM_UTILITIES_H
#define FLEXIBLE_SURFACE_AUGMENTATION_LSCM_UTILITIES_H

#include <iostream>

#include <ofVec2f.h>
#include <ofVec3f.h>

/* Basic geometric types */
class Vector2 {
	public:
		Vector2(double x_in, double y_in) : x(x_in), y(y_in) {}
		Vector2(ofVec2f ofVec) : x(ofVec.x), y(ofVec.y) {}
		Vector2() : x(0), y(0) {}

		Vector2 operator+(const Vector2& v2) const;
		Vector2 operator-(const Vector2& v2) const;
		Vector2& operator=(const ofVec2f& ofVec);

		std::ostream& operator<<(std::ostream& out) const;
		std::istream& operator>>(std::istream& in);

		double x;
		double y;
};

class Vector3 {
	public:
		Vector3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}
		Vector3(ofVec3f ofVec) : x(ofVec.x), y(ofVec.y), z(ofVec.z) {}
		Vector3() : x(0), y(0), z(0) {}
		double length() const;
		void normalize();

		double operator*(const Vector3& v2) const; // dot product
		Vector3 operator^(const Vector3& v2) const; // cross product
		Vector3 operator+(const Vector3& v2) const;
		Vector3 operator-(const Vector3& v2) const;
		Vector3& operator=(const ofVec3f& ofVec);

		std::ostream& operator<<(std::ostream& out) const;
		std::istream& operator>>(std::istream& in);

		double x;
		double y;
		double z;
};

class ofVec3fExt : public ofVec3f {
	ofVec3fExt& operator=(const Vector3& vec);
	ofVec3f& operator=(const ofVec3fExt& vec);
};

#endif // FLEXIBLE_SURFACE_AUGMENTATION_LSCM_UTILITIES_H