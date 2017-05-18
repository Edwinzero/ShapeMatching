#pragma once
#ifndef _EIGEN_OP_H
#define _EIGEN_OP_H
#include <iostream>
#include <vector>
#include <Eigen\Core>
#include <Eigen/Geometry>
#include <Eigen\Dense>
#include <glm\glm.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace std;
using namespace Eigen;

Eigen::Matrix3f Rot_x_mat(float rad) {
	float c = cos(rad);
	float s = sin(rad);
	Eigen::Matrix3f m;
	m << 1, 0, 0,
		 0, c, -s,
		 0, s, c;
	return m;
}
Eigen::Matrix3f Rot_y_mat(float rad) {
	float c = cos(rad);
	float s = sin(rad);
	Eigen::Matrix3f m;
	m << c, 0, s,
		 0, 1, 0,
		-s, 0, c;
	return m;
}
Eigen::Matrix3f Rot_z_mat(float rad) {
	float c = cos(rad);
	float s = sin(rad);
	Eigen::Matrix3f m;
	m << c, -s, 0,
		 s,  c, 0,
		 0,  0, 1;
	return m;
}

// R(z) * R(y) * R(x)
Eigen::Matrix3f Euler_to_mat(const Eigen::Vector3f &euler) {
	return Rot_z_mat(euler.z()) * Rot_y_mat(euler.y()) * Rot_x_mat(euler.x());
}
// R(z) * R(y) * R(x)
Eigen::Matrix4f Euler_to_mat(const Eigen::Vector4f &euler) {
	Eigen::Matrix3f m = Rot_z_mat(euler.z()) * Rot_y_mat(euler.y()) * Rot_x_mat(euler.x());
	Eigen::Matrix4f mm;
	mm << m(0), m(1), m(2), 0,
		m(3), m(4), m(5), 0,
		m(6), m(7), m(8), 0,
		0, 0, 0, 1;
	return mm;
}

void AffineTransformPointsFromAngle(std::vector< Eigen::Vector3f > &points, const Eigen::Vector3f &euler_angle, const Eigen::Vector3f &t) {
	if (points.empty()) {
		return;
	}
	const float RAD = 3.1415926f / 180.0f;
	Eigen::Vector3f rad = Eigen::Vector3f(euler_angle(0) * RAD, euler_angle(1) * RAD, euler_angle(2) * RAD);
	Eigen::Matrix3f rot = Euler_to_mat(rad);
	for (std::vector<Eigen::Vector3f>::iterator it = points.begin(); it < points.end(); it++) {
		*it = rot * *it + t;
	}
}
void AffineTransformPointsFromAngle(std::vector< Eigen::Vector4f > &points, const Eigen::Vector3f &euler_angle, const Eigen::Vector3f &t) {
	if (points.empty()) {
		return;
	}
	const float RAD = 3.1415926f / 180.0f;
	Eigen::Vector4f rad = Eigen::Vector4f(euler_angle(0) * RAD, euler_angle(1) * RAD, euler_angle(2) * RAD, 0.0f);
	Eigen::Matrix4f rot = Euler_to_mat(rad);
	Eigen::Vector4f trans = Eigen::Vector4f(t(0), t(1), t(2), 0.0f);
	for (std::vector<Eigen::Vector4f>::iterator it = points.begin(); it < points.end(); it++) {
		*it = rot * *it + trans;
	}
}
void AffineTransfomrPointsFromMat(std::vector< Eigen::Vector3f > &points, const Eigen::Matrix4f &mat) {
	if (points.empty()) {
		return;
	}
	for (std::vector<Eigen::Vector3f>::iterator it = points.begin(); it < points.end(); it++) {
		Eigen::Vector4f tmp = mat * Eigen::Vector4f(it->x(), it->y(), it->z(), 1.0f);
		*it = Eigen::Vector3f(tmp.x(), tmp.y(), tmp.z());
	}
}

void ScalePoints(std::vector<Eigen::Vector4f> &points, const float scale = 1.0f) {
	if (points.empty()) {
		return;
	}
	for (std::vector<Eigen::Vector4f>::iterator it = points.begin(); it < points.end(); it++) {
		*it = *it * scale;
		it->w() /= scale;
	}
}
void ScalePoints(std::vector<Eigen::Vector3f> &points, const float scale = 1.0f) {
	if (points.empty()) {
		return;
	}
	for (std::vector<Eigen::Vector3f>::iterator it = points.begin(); it < points.end(); it++) {
		*it = *it * scale;
	}
}

// skew( w ) * v = cross( w, v )
// Infinitesimal rotations = skew( w ) + I
Eigen::Matrix3f Skew_mat(const Eigen::Vector3f &v) {
	float x = v.x();
	float y = v.y();
	float z = v.z();

	Eigen::Matrix3f sk;
	sk << 0, -z, +y,
		 +z, 0, -x,
		 -y, +x, 0;
	return sk;
}


void ConstructMatEigenToGlm4(glm::mat4 &out, Eigen::Matrix3f &rot, Eigen::Vector3f &t) {
	// Eigen matrix is column major, need to transpose first
	Eigen::Matrix3f rott = rot.transpose();
	for (int i = 0; i < 3; i++) {
		glm::value_ptr(out)[4 * i + 0] = rott(3 * i + 0);
		glm::value_ptr(out)[4 * i + 1] = rott(3 * i + 1);
		glm::value_ptr(out)[4 * i + 2] = rott(3 * i + 2);
		glm::value_ptr(out)[4 * i + 3] = t(i);
	}
	glm::value_ptr(out)[12] = 0.0f; glm::value_ptr(out)[13] = 0.0f;
	glm::value_ptr(out)[14] = 0.0f; glm::value_ptr(out)[15] = 1.0f;
}

void ConstructMatEigenToEigen4(Eigen::Matrix4f &out, Eigen::Matrix3f &rot, Eigen::Vector3f &t) {
	// Eigen matrix is column major, need to transpose first(Eigen to Eigen doesn't need)
	Eigen::Matrix3f rott = rot;
	for (int i = 0; i < 3; i++) {
		out(4 * i + 0) = rott(3 * i + 0);
		out(4 * i + 1) = rott(3 * i + 1);
		out(4 * i + 2) = rott(3 * i + 2);
		out(4 * i + 3) = 0.0f;
	}
	out(12) = t(0); out(13) = t(1);
	out(14) = t(2); out(15) = 1.0f;
}


// E(v)
Eigen::Vector3f Mean(const std::vector< Eigen::Vector3f > &points) {
	if (points.empty()) {
		return Eigen::Vector3f::Zero();
	}

	Eigen::Vector3f e;
	e.setZero();
	for (int i = 0; i < points.size(); i++) {
		e += points[i];
	}
	e /= (float)(points.size());
	return e;
}

// sqrt( E( | dst - r * src - t |^2 ) )
float Rme(const std::vector< Eigen::Vector3f > &src, const std::vector< Eigen::Vector3f > &dst, const Eigen::Matrix3f &r, const Eigen::Vector3f &t) {
	if (src.empty()) {
		return 0.0f;
	}

	float e = 0.0f;
	for (int i = 0; i < src.size(); i++) {
		Eigen::Vector3f d = dst[i] - r * src[i] - t;
		e += d.dot(d);
	}
	e /= (float)(src.size());

	return sqrt(e);
}

Eigen::Matrix4f createMat4(vector<float> &val) {
	Eigen::Matrix4f res = Eigen::Matrix4f::Identity();
	res << val[0], val[1], val[2], val[3],
		val[4], val[5], val[6], val[7],
		val[8], val[9], val[10], val[11],
		val[12], val[13], val[14], val[15];
	return res;
}


#endif // !_EIGEN_OP_H
