#pragma once
#ifndef _CORRESPONDENCE_FINDING_H
#define _CORRESPONDENCE_FINDING_H
#include <SIFTmatching.h>
#include <GMSmatching.h>
#include <Sensor.h>
#include <Eigen_op.h>
#include <cv_op.h>
#include <opencv2\core\eigen.hpp>

namespace CORRES {
	// corres ( srcID, dstID )  src = current new frame, dst = target frame
	void ProjectiveCorresondence(std::vector<Eigen::Vector4f> &srcP, std::vector<Eigen::Vector4f> &srcN, std::vector<Eigen::Vector4f> &dstP, std::vector<Eigen::Vector4f> &dstN, 
		Eigen::Matrix4f &msrc, Eigen::Matrix4f &mdst,
		std::vector<pair<int, int>> &corres, Sensor &sensor) {
		corres.clear();
		cv::Mat intr = sensor.cali_ir.intr.IntrVec();
		float fx = intr.at<double>(0);
		float fy = intr.at<double>(1);
		float cx = intr.at<double>(2);
		float cy = intr.at<double>(3);

		// extract R and T from mdst
		Eigen::Matrix4f srcToDst = Eigen::Matrix4f::Identity();
		srcToDst = mdst.inverse() * msrc;
		Eigen::Matrix3f R = srcToDst.block<3, 3>(0, 0);
		Eigen::Vector3f t = srcToDst.block<3, 1>(0, 3);

		// for each pixel in depthP
		for (int y = 0; y < 424; y++) {
			for (int x = 0; x < 512; x++) {
				int did = 512 * y + x;
				// if valid depth
				if (srcP[did](2) <= 0) {
					continue;
				}

				// perspective project vertex
				Vector3f dst = (R * Vector3f(dstP[did](0), dstP[did](1), dstP[did](2)) + t);		// transform to camera space
				float u = fx * (dst(0) / dst(2)) + cx;
				float v = fy * (dst(1) / dst(2)) + cy;
				int px = static_cast<int>(u );
				int py = static_cast<int>(v );																// compute uv coordinate
				if (px < 0 || px >= 512 || py < 0 || py >= 424) {
					continue;
				}

				// compute v and n for src
				// divide scale
				int sid = 512 * py + px;
				Vector3f sv = (R * Vector3f(srcP[sid](0), srcP[sid](1), srcP[sid](2)) + t);
				Vector3f sn = R * Vector3f(srcN[sid](0), srcN[sid](1), srcN[sid](2));
				sn.normalize();
				if (sn(0) == 0 && sn(1) == 0 && sn(2) == 0) {
					continue;
				}

				printf(" >> did y: %d, x: %d\n", y, x);
				printf(" >> sid y: %d, x: %d\n", py, px);
				//std::cout << " did normal of srcN: " << srcN[did] << std::endl;
				//std::cout << " sid normal of srcN: " << srcN[sid] << std::endl;

				float dThres = 0.25f *2.0f;
				float nThres = 0.65f;
				Vector3f diffv = sv - dst;  // TODO: dst or dp? (cam space or world space?)
				Vector3f dn = Vector3f(dstN[did](0), dstN[did](1), dstN[did](2));
				dn.normalize();
				//std::cout << "normal of srcN: " << srcN[sid] << std::endl;
				//std::cout << "normal of dstN: " << dstN[did] << std::endl;
				printf("mag: %f,  dot: %f \n", diffv.norm(), sn.dot(dn));
				if (diffv.norm() < dThres && sn.dot(dn) < nThres){// && n.dot(dstN[did].head<3>()) > 0) {
					//std::cout << "normal of srcN: " << srcN[sid] << std::endl;
					//std::cout << "normal of dstN: " << dstN[did] << std::endl;
					printf("mag: %f,  dot: %f \n", diffv.norm(), sn.dot(dn));
					corres.push_back(std::pair<int, int>(sid, did));
				}
			}
		}
	}



	void ProjectiveCorresondence(cv::Mat srcD, std::vector<Eigen::Vector4f> &srcP, std::vector<Eigen::Vector4f> &srcN, std::vector<Eigen::Vector4f> &dstP, std::vector<Eigen::Vector4f> &dstN,
		Eigen::Matrix4f &msrc, Eigen::Matrix4f &mdst,
		std::vector<pair<int, int>> &corres, Sensor &sensor, float scale = 1.0f) {
		corres.clear();
		cv::Mat intr = sensor.cali_ir.intr.IntrVec();
		float fx = intr.at<double>(0);
		float fy = intr.at<double>(1);
		float cx = intr.at<double>(2);
		float cy = intr.at<double>(3);

		// extract R and T from mdst
		Eigen::Matrix3f R = mdst.block<3, 3>(0, 0);
		Eigen::Vector3f t = mdst.block<3, 1>(0, 3);

		// for each pixel in depthP
		for (int y = 0; y < 424; y++) {
			for (int x = 0; x < 512; x++) {

			}
		}
	}
};


#endif // !_CORRESPONDENCE_FINDING_H

/*
int sid = y * 512 + x;
cv::Point3f src(srcP[sid](0), srcP[sid](1), srcP[sid](2)); // world space
Eigen::Vector4f ev = msrc * srcP[sid];
cv::Point3f vsrc = cv::Point3f(ev(0), ev(1), ev(2));
int px = (vsrc.x / vsrc.z) * fx + cx;
int py = (vsrc.y / vsrc.z) * fy + cy;
if (px < 0 || px >= 512 || py < 0 || py >= 424) {
continue;
}
// if p is in vertex map Vnew
int did = py * 512 + px;
// vcur = Tcur * dstP
ev = mdst * dstP[did];
cv::Point3f vcur(ev(0), ev(1), ev(2));
// ncur = Rcur * dstN
Eigen::Matrix3f R = mdst.block<3, 3>(0, 0);
Eigen::Vector3f en = R * Eigen::Vector3f(dstN[did](0), dstN[did](1), dstN[did](2));
//cv::Point3f ncur = normalize(cv::Point3f(en(0), en(1), en(2)));
cv::Point3f ncur = normalize(cv::Point3f(dstN[did](0), dstN[did](1), dstN[did](2)));
// if vcur depth out of range of interest
if (vcur.z < 0.05f*scale || vcur.z > 2.0f*scale) {
continue;
}

cv::Point3f nsrc = normalize(cv::Point3f(srcN[sid](0), srcN[sid](1), srcN[sid](2)));;
float dThres = 0.25f*scale;
float nThres = 0.65f;
if (magnitude(src - vcur) < dThres && abs(ncur.dot(nsrc)) < nThres) {
std::cout << "normal of dstN: " << dstN[did] << std::endl;
std::cout << "normal of srcN: " << srcN[sid] << std::endl;
printf("mag: %f,  dot: %f \n", magnitude(src - vcur), abs(ncur.dot(nsrc)));
corres.push_back(std::pair<int, int>(sid, did));
}


int did = y * 512 + x;
// if valid depth
if (srcP[did](2) <= 0) {
continue;
}

// perspective project vertex
Vector3f dst = R * dstP[did].head<3>() + t; // R * Vector3f(dstP[id](0), dstP[id](1), dstP[id](2)) + t;
int px = fx * (dst(0) / dst(2)) + cx;
int py = fy * (dst(1) / dst(2)) + cy;
if (px < 0 || px >= 512 || py < 0 || py >= 424) {
continue;
}

// compute v and n for src
int sid = py * 512 + px;
Vector3f v = R * srcP[sid].head<3>() + t; // R * Vector3f(srcP[did](0), srcP[did](1), srcP[did](2)) + t;
Vector3f n = R * srcN[sid].head<3>();	  // Vector3f(srcN[did](0), srcN[did](1), srcN[did](2));
n.normalize();
float dThres = 0.25f * scale;
float nThres = 0.65f;
if ((v - dstP[did].head<3>()).norm() < dThres && n.dot(dstN[did].head<3>()) < nThres && n.dot(dstN[did].head<3>()) > 0) {
std::cout << "normal of srcN: " << srcN[sid] << std::endl;
std::cout << "normal of dstN: " << dstN[did] << std::endl;
printf("mag: %f,  dot: %f \n", (v - dstP[did].head<3>()).norm(), n.dot(dstN[did].head<3>()));
corres.push_back(std::pair<int, int>(sid, did));
}

*/