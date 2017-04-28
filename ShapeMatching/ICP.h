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

// sqrt( E( | dst - r * src - t |^2 ) )
float rme(const std::vector< Eigen::Vector3f > &src, const std::vector< Eigen::Vector3f > &dst, const Eigen::Matrix3f &r, const Eigen::Vector3f &t) {
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
// sqrt( E( | dst - r * src - t |^2 ) )
float rme(const std::vector< Eigen::Vector4f > &src, const std::vector< Eigen::Vector4f > &dst, const Eigen::Matrix3f &r, const Eigen::Vector3f &t) {
	if (src.empty()) {
		return 0.0f;
	}

	float e = 0.0f;
	for (int i = 0; i < src.size(); i++) {
		Eigen::Vector3f sp = Eigen::Vector3f(src[i](0), src[i](1), src[i](2));
		Eigen::Vector3f tp = Eigen::Vector3f(dst[i](0), dst[i](1), dst[i](2));
		Eigen::Vector3f d = tp - r * sp - t;
		e += d.dot(d);
	}
	e /= (float)(src.size());

	return sqrt(e);
}


float PointToPoint_ICP(std::vector<Eigen::Vector3f> &src, std::vector<Eigen::Vector3f> &tar,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation) {
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
	return sqrtf(error);
	
}


float PointToPoint_ICP(std::vector<Eigen::Vector4f> &src4, std::vector<Eigen::Vector4f> &tar4, std::vector<std::pair<int, int>> &corres,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation) {
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
	return sqrtf(error);
}

float PointToPoint_ICP(std::vector<Eigen::Vector4f> &src4, std::vector<Eigen::Vector4f> &tar4,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation) {
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
	return sqrtf(error);
}


float PointToPoint_iterICP(std::vector<Eigen::Vector3f> &src, std::vector<Eigen::Vector3f> &tar,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation, float &err) {
	if (src.empty()) {
		rotation.setIdentity();
		translation .setZero();
		return 0.0f;
	}

	Eigen::MatrixXf A(6, 6);
	A.setZero();

	Eigen::VectorXf b(6);
	b.setZero();

	// A * x = b

	// A = Sum( ( S', I )' * ( S', I ) )
	// x = ( w, t )
	// b = Sum( ( m, d )' )

	// d = dst - src
	// m = cross( src, d )
	A.block(3, 3, 3, 3) = Eigen::Matrix3f::Identity() * src.size();
	for (int i = 0; i < src.size(); i++) {
		Eigen::Vector3f sp = rotation * src[i] + translation;
		Eigen::Vector3f dp = tar[i];

		Eigen::Matrix3f S = skew_mat(sp);
		Eigen::Vector3f d = dp - sp;
		Eigen::Vector3f m = sp.cross(d);

		A.block(0, 0, 3, 3) += S * -S;
		A.block(0, 3, 3, 3) += S;
		A.block(3, 0, 3, 3) += -S;

		b.block(0, 0, 3, 1) += m;
		b.block(3, 0, 3, 1) += d;
	}
	// solve A * x = b
	Eigen::VectorXf wt(6);
	wt = A.fullPivHouseholderQr().solve(b);

	// rotation
	Eigen::Vector3f euler;
	euler << wt[0], wt[1], wt[2];
	Eigen::Matrix3f r = euler_to_mat(euler);

	// translation
	Eigen::Vector3f t;
	t << wt[3], wt[4], wt[5];

	// update transform
	// dst = r * ( out_r * src + out_t ) + t;
	rotation = r * rotation;
	translation = r * translation + t;

	// rme
	return rme(src, tar, rotation, translation);
}

float PointToPoint_iterICP(std::vector<Eigen::Vector4f> &src4, std::vector<Eigen::Vector4f> &tar4,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation) {
	if (src4.empty()) {
		rotation.setIdentity();
		translation.setZero();
		return 0.0f;
	}

	Eigen::MatrixXf A(6, 6);
	A.setZero();

	Eigen::VectorXf b(6);
	b.setZero();

	// A * x = b

	// A = Sum( ( S', I )' * ( S', I ) )
	// x = ( w, t )
	// b = Sum( ( m, d )' )

	// d = dst - src
	// m = cross( src, d )
	A.block(3, 3, 3, 3) = Eigen::Matrix3f::Identity() * src4.size();
	for (int i = 0; i < src4.size(); i++) {
		Eigen::Vector3f pt = Eigen::Vector3f(src4[i](0), src4[i](1), src4[i](2));
		Eigen::Vector3f src = rotation * pt + translation;
		Eigen::Vector3f dst = Eigen::Vector3f(tar4[i](0), tar4[i](1), tar4[i](2));

		Eigen::Matrix3f S = skew_mat(src);
		Eigen::Vector3f d = dst - src;
		Eigen::Vector3f m = src.cross(d);

		A.block(0, 0, 3, 3) += S * -S;
		A.block(0, 3, 3, 3) += S;
		A.block(3, 0, 3, 3) += -S;

		b.block(0, 0, 3, 1) += m;
		b.block(3, 0, 3, 1) += d;
	}
	// solve A * x = b
	Eigen::VectorXf wt(6);
	wt = A.fullPivHouseholderQr().solve(b);

	// rotation
	Eigen::Vector3f euler;
	euler << wt[0], wt[1], wt[2];
	Eigen::Matrix3f r = euler_to_mat(euler);

	// translation
	Eigen::Vector3f t;
	t << wt[3], wt[4], wt[5];

	// update transform
	// dst = r * ( out_r * src + out_t ) + t;
	rotation = r * rotation;
	translation = r * translation + t;

	// rme
	return rme(src4, tar4, rotation, translation);
}

float PointToPlaneIter_ICP(std::vector<Eigen::Vector3f> &src, std::vector<Eigen::Vector3f> &tar, std::vector<Eigen::Vector3f> &tar_normal,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation) {
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

	return rme(src, tar, rotation, translation);
}

float PointToPlaneIter_ICP(std::vector<Eigen::Vector4f> &src4, std::vector<Eigen::Vector4f> &tar4, std::vector<Eigen::Vector4f> &tar_normal,
	Eigen::Matrix3f &rotation, Eigen::Vector3f &translation) {
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
		Eigen::Vector3f pt = rotation * src + translation;
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

	return rme(src4, tar4, rotation, translation);
}


#endif