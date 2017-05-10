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
				int did = y * 512 + x;
				// if valid depth
				if (srcP[did](2) <= 0) {
					continue;
				}

				// perspective project vertex
				Vector3f dst = R * Vector3f(dstP[did](0), dstP[did](1), dstP[did](2)) + t;
				// TODO : PERSPECTIVE PROJECTION ERROR!!!!
				int px = fx * (dst(0) / dst(2)) + cx;
				int py = fy * (dst(1) / dst(2)) + cy;
				if (px < 0 || px >= 512 || py < 0 || py >= 424) {
					continue;
				}

				// compute v and n for src
				int sid = py * 512 + px;
				Vector3f v = R * Vector3f(srcP[sid](0), srcP[sid](1), srcP[sid](2)) + t;
				Vector3f n = R * Vector3f(srcN[sid](0), srcN[sid](1), srcN[sid](2));
				n.normalize();
				if (n(0) == 0 && n(1) == 0 && n(2) == 0) {
					continue;
				}

				float dThres = 0.25f * scale*2000.0f;
				float nThres = 0.65f;
				Vector3f diffv = v - dst;  // TODO: dst or dp? (cam space or world space?)
				Vector3f dn = Vector3f(dstN[did](0), dstN[did](1), dstN[did](2));
				std::cout << "normal of srcN: " << srcN[sid] << std::endl;
				std::cout << "normal of dstN: " << dstN[did] << std::endl;
				printf("mag: %f,  dot: %f \n", diffv.norm(), n.dot(dstN[did].head<3>()));
				if (diffv.norm() < dThres && n.dot(dn) < nThres){// && n.dot(dstN[did].head<3>()) > 0) {
					std::cout << "normal of srcN: " << srcN[sid] << std::endl;
					std::cout << "normal of dstN: " << dstN[did] << std::endl;
					printf("mag: %f,  dot: %f \n", diffv.norm(), n.dot(dn));
					corres.push_back(std::pair<int, int>(sid, did));
				}
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