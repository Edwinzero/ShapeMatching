#pragma once
#ifndef __ICP_H
#define __ICP_H
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

// R(x)
Eigen::Matrix3f rot_x_mat(float rad) {
	float c = cos(rad);
	float s = sin(rad);

	Eigen::Matrix3f m;
	m <<
		1, 0, 0,
		0, c, -s,
		0, s, c
		;

	return m;
}

// R(y)
Eigen::Matrix3f rot_y_mat(float rad) {
	float c = cos(rad);
	float s = sin(rad);

	Eigen::Matrix3f m;
	m <<
		c, 0, s,
		0, 1, 0,
		-s, 0, c
		;

	return m;
}

// R(z)
Eigen::Matrix3f rot_z_mat(float rad) {
	float c = cos(rad);
	float s = sin(rad);

	Eigen::Matrix3f m;
	m <<
		c, -s, 0,
		s, c, 0,
		0, 0, 1
		;

	return m;
}

// skew( w ) * v = cross( w, v )
// Infinitesimal rotations = skew( w ) + I
// Aprroximation
Eigen::Matrix3f skew_mat(const Eigen::Vector3f &v) {
	float x = v.x();
	float y = v.y();
	float z = v.z();

	Eigen::Matrix3f sk;
	sk <<
		0, -z, +y,
		+z, 0, -x,
		-y, +x, 0
		;

	return sk;
}

// R(z) * R(y) * R(x)
Eigen::Matrix3f euler_to_mat(const Eigen::Vector3f &euler) {
	return rot_z_mat(euler.z()) * rot_y_mat(euler.y()) * rot_x_mat(euler.x());
}

Eigen::Vector3f mean(std::vector<Eigen::Vector3f> data) {
	Eigen::Vector3f sum = Eigen::Vector3f::Zero();
	for (int i = 0; i < data.size(); i++) {
		sum += data[i];
	}
	return sum / (float)data.size();
}
Eigen::Vector3f mean(std::vector<Eigen::Vector4f> data) {
	Eigen::Vector4f sum4 = Eigen::Vector4f::Zero();
	for (int i = 0; i < data.size(); i++) {
		sum4 += data[i];
	}
	sum4 =  sum4 / (float)data.size();
	return Eigen::Vector3f(sum4(0), sum4(1), sum4(2));
}

void PointToPoint_ICP(std::vector<Eigen::Vector3f> &src, std::vector<Eigen::Vector3f> &tar,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation, float &err) {
	//1. compute weighted average
	Eigen::Vector3f m_src = mean(src);
	Eigen::Vector3f m_tar = mean(tar);
	
	//2. compute centered vec p = pave
	//3. compute S = Y *W *X = Y*X
	// EIGEN VARIABLE MUST BE INITIALIZED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
	Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
	for (int i = 0; i < src.size(); i++) {
		Eigen::Vector3f p = src[i] - m_src;
		Eigen::Vector3f q = tar[i] - m_tar;
		S += p * q.transpose();
	}

	//4. Using SVD to get U * sigma * V
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);

	//5. Compute R = U * I' * V
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
	float neg_detuv = (svd.matrixV()*svd.matrixU().transpose()).determinant();
	I(2, 2) = neg_detuv;
	rotation = svd.matrixV() * I * svd.matrixU().transpose();

	//6. Compute t = m_tar - R * m_src
	translation = m_tar - rotation * m_src;

	//7. evaluate || q - (Rp + t)||^2
	float error = 0.0f;
	for (int i = 0; i < src.size(); i++) {
		Eigen::Vector3f newp = tar[i] - rotation*src[i] - translation;
		error += newp.dot(newp);
	}
	error /= (float)(src.size());
	err = sqrtf(error);
}
void PointToPoint_ICP(std::vector<Eigen::Vector4f> &src4, std::vector<Eigen::Vector4f> &tar4,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation, float &err) {
	//1. compute weighted average
	Eigen::Vector3f m_src = mean(src4);
	Eigen::Vector3f m_tar = mean(tar4);

	//2. compute centered vec p = pave
	//3. compute S = Y *W *X = Y*X
	// EIGEN VARIABLE MUST BE INITIALIZED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
	Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
	for (int i = 0; i < src4.size(); i++) {
		Eigen::Vector3f src = Eigen::Vector3f(src4[i](0), src4[i](1), src4[i](2));
		Eigen::Vector3f tar = Eigen::Vector3f(tar4[i](0), tar4[i](1), tar4[i](2));
		Eigen::Vector3f p = src - m_src;
		Eigen::Vector3f q = tar - m_tar;
		S += p * q.transpose();
	}

	//4. Using SVD to get U * sigma * V
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);

	//5. Compute R = U * I' * V
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
	float neg_detuv = (svd.matrixV()*svd.matrixU().transpose()).determinant();
	I(2, 2) = neg_detuv;
	rotation = svd.matrixV() * I * svd.matrixU().transpose();

	//6. Compute t = m_tar - R * m_src
	translation = m_tar - rotation * m_src;

	//7. evaluate || q - (Rp + t)||^2
	float error = 0.0f;
	for (int i = 0; i < src4.size(); i++) {
		Eigen::Vector3f src = Eigen::Vector3f(src4[i](0), src4[i](1), src4[i](2));
		Eigen::Vector3f tar = Eigen::Vector3f(tar4[i](0), tar4[i](1), tar4[i](2));
		Eigen::Vector3f newp = tar - rotation*src - translation;
		error += newp.dot(newp);
	}
	error /= (float)(src4.size());
	err = sqrtf(error);
}

void PointToPoint_ICP(std::vector<Eigen::Vector4f> &src4, std::vector<Eigen::Vector4f> &tar4, std::vector<std::pair<int, int>> &corres,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation, float &err) {
	//1. compute weighted average
	Eigen::Vector3f m_src = mean(src4);
	Eigen::Vector3f m_tar = mean(tar4);

	//2. compute centered vec p = pave
	//3. compute S = Y *W *X = Y*X
	// EIGEN VARIABLE MUST BE INITIALIZED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
	Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
	for (int i = 0; i < corres.size(); i++) {
		int f = corres[i].first;
		int s = corres[i].second;
		Eigen::Vector3f src = Eigen::Vector3f(src4[f](0), src4[f](1), src4[f](2));
		Eigen::Vector3f tar = Eigen::Vector3f(tar4[s](0), tar4[s](1), tar4[s](2));
		Eigen::Vector3f p = src - m_src;
		Eigen::Vector3f q = tar - m_tar;
		S += p * q.transpose();
	}

	//4. Using SVD to get U * sigma * V
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);

	//5. Compute R = U * I' * V
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
	float neg_detuv = (svd.matrixV()*svd.matrixU().transpose()).determinant();
	I(2, 2) = neg_detuv;
	rotation = svd.matrixV() * I * svd.matrixU().transpose();

	//6. Compute t = m_tar - R * m_src
	translation = m_tar - rotation * m_src;

	//7. evaluate || q - (Rp + t)||^2
	float error = 0.0f;
	for (int i = 0; i < corres.size(); i++) {
		int f = corres[i].first;
		int s = corres[i].second;
		Eigen::Vector3f src = Eigen::Vector3f(src4[f](0), src4[f](1), src4[f](2));
		Eigen::Vector3f tar = Eigen::Vector3f(tar4[s](0), tar4[s](1), tar4[s](2));
		Eigen::Vector3f newp = tar - rotation*src - translation;
		error += newp.dot(newp);
	}
	error /= (float)(src4.size());
	err = sqrtf(error);
}



void PointToPoint_iterICP(std::vector<Eigen::Vector3f> &src, std::vector<Eigen::Vector3f> &tar,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation, float &err) {
	Eigen::MatrixXf A(6, 6);
	A.setZero();

	Eigen::VectorXf b(6);
	b.setZero();

}

void PointToPlane_ICP(std::vector<Eigen::Vector3f> &src, std::vector<Eigen::Vector3f> &tar, std::vector<Eigen::Vector3f> &tar_normal,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation, float &err) {
	Eigen::MatrixXf A(6, 6);
	A.setZero();

	Eigen::VectorXf b(6);
	b.setZero();

	//A * x = b
	// A = [ <pi x ni> < ni> ]
	// x = [ rx ry rz tx ty tz]
	// b = [ -(pi - qi) * ni ]
	// AtAx = Atb
	for (int i = 0; i < src.size(); i++) {
		Eigen::Vector3f pt = rotation * src[i] + translation;
		Eigen::Vector3f n = tar_normal[i];

		// residual
		float residual = (tar[i] - pt).dot(n);
		Eigen::Vector3f pxn = src[i].cross(n);
		Eigen::VectorXf a(6);

		a(0) = pxn.x();
		a(1) = pxn.y();
		a(2) = pxn.z();
		a(3) = n.x();
		a(4) = n.y();
		a(5) = n.z();

		// JTJ
		A += a * a.transpose();
		// JTr
		b += a * residual;
	}

	// solve A * x = b
	Eigen::VectorXf w(6);
	w = A.fullPivHouseholderQr().solve(b);

	//construct R and t
	Eigen::Vector3f angle;
	angle << w(0), w(1), w(2);
	Eigen::Matrix3f rot = euler_to_mat(angle);

	Eigen::Vector3f t;
	t << w(3), w(4), w(5);

	// update R and T
	// tar = rot * ( rotation * src + translation ) + t;
	rotation = rot * rotation;
	translation = rot * translation + t;
}

void PointToPlane_ICP(std::vector<Eigen::Vector4f> &src4, std::vector<Eigen::Vector4f> &tar4, std::vector<Eigen::Vector4f> &tar_normal,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation, float &err) {
	Eigen::MatrixXf A(6, 6);
	A.setZero();

	Eigen::VectorXf b(6);
	b.setZero();

	//A * x = b
	// A = [ <pi x ni> < ni> ]
	// x = [ rx ry rz tx ty tz]
	// b = [ -(pi - qi) * ni ]
	// AtAx = Atb
	for (int i = 0; i < src4.size(); i++) {
		Eigen::Vector3f src = Eigen::Vector3f(src4[i](0), src4[i](1), src4[i](2));
		Eigen::Vector3f pt = rotation * src  + translation;
		Eigen::Vector3f n = Eigen::Vector3f(tar_normal[i](0), tar_normal[i](1), tar_normal[i](2));

		// residual
		Eigen::Vector3f tar = Eigen::Vector3f(tar4[i](0), tar4[i](1), tar4[i](2));
		float residual = (tar - pt).dot(n);
		Eigen::Vector3f pxn = src.cross(n);
		Eigen::VectorXf a(6);

		a(0) = pxn.x();
		a(1) = pxn.y();
		a(2) = pxn.z();
		a(3) = n.x();
		a(4) = n.y();
		a(5) = n.z();

		// JTJ
		A += a * a.transpose();
		// JTr
		b += a * residual;
	}

	// solve A * x = b
	Eigen::VectorXf w(6);
	w = A.fullPivHouseholderQr().solve(b);

	//construct R and t
	Eigen::Vector3f angle;
	angle << w(0), w(1), w(2);
	Eigen::Matrix3f rot = euler_to_mat(angle);

	Eigen::Vector3f t;
	t << w(3), w(4), w(5);

	// update R and T
	// tar = rot * ( rotation * src + translation ) + t;
	rotation = rot * rotation;
	translation = rot * translation + t;
}




#endif