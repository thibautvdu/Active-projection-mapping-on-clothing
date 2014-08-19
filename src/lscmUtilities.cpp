#include "lscmUtilities.h"

// Vector2
Vector2 Vector2::operator + (const Vector2& v2) const {
	return Vector2(
		x + v2.x,
		y + v2.y
		);
}

Vector2 Vector2::operator - (const Vector2& v2) const {
	return Vector2(
		x - v2.x,
		y - v2.y
		);
}

Vector2& Vector2::operator = (const ofVec2f& ofVec) {
	x = ofVec.x;
	y = ofVec.y;

	return *this;
}

std::ostream& Vector2::operator<<(std::ostream& out) const {
	return out << x << " " << y;
}

std::istream& Vector2::operator>>(std::istream& in) {
	return in >> x >> y;
}

// Vector3
double Vector3::length() const {
	return sqrt(x*x + y*y + z*z);
}

void Vector3::normalize() {
	double l = length();
	x /= l; y /= l; z /= l;
}

double Vector3::operator*(const Vector3& v2) const {
	return x * v2.x + y * v2.y + z * v2.z;
}

Vector3 Vector3::operator^(const Vector3& v2) const {
	return Vector3(
		y*v2.z - v2.y*z,
		z*v2.x - v2.z*x,
		x*v2.y - v2.x*y
		);
}

Vector3 Vector3::operator+(const Vector3& v2) const {
	return Vector3(
		x + v2.x,
		y + v2.y,
		z + v2.z
		);
}

Vector3 Vector3::operator-(const Vector3& v2) const {
	return Vector3(
		x - v2.x,
		y - v2.y,
		z - v2.z
		);
}

Vector3& Vector3::operator = (const ofVec3f& ofVec) {
	x = ofVec.x;
	y = ofVec.y;
	z = ofVec.z;

	return *this;
}

std::ostream& Vector3::operator<<(std::ostream& out) const {
	return out << x << " " << y << " " << z;
}

std::istream& Vector3::operator>>(std::istream& in) {
	return in >> x >> y >> z;
}

// Extension of ofVec3
ofVec3fExt& ofVec3fExt::operator=(const Vector3& vec) {
	x = vec.x;
	y = vec.y;
	z = vec.z;

	return *this;
}

ofVec3f& ofVec3fExt::operator=(const ofVec3fExt& vec) {
	x = vec.x;
	y = vec.y;
	z = vec.z;

	return *this;
}
