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
	// corres ( srcID, dstID )
	void ProjectiveCorresondence(std::vector<Eigen::Vector4f> &srcP, std::vector<Eigen::Vector4f> &srcN, std::vector<Eigen::Vector4f> &dstP, std::vector<Eigen::Vector4f> &dstN, 
		Eigen::Matrix4f &msrc, Eigen::Matrix4f &mdst,
		std::vector<pair<int, int>> &corres, Sensor &sensrc, float scale = 1.0f) {
		corres.clear();
		cv::Mat intr = sensrc.cali_ir.intr.IntrVec();
		float fx = intr.at<double>(0);
		float fy = intr.at<double>(1);
		float cx = intr.at<double>(2);
		float cy = intr.at<double>(3);

		// for each pixel in depthP
		for (int y = 0; y < 424; y++) {
			for (int x = 0; x < 512; x++) {
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
				//std::cout << "normal of dstN: " << dstN[did] << std::endl;
				//std::cout << "normal of srcN: " << srcN[sid] << std::endl;
				cv::Point3f nsrc = normalize(cv::Point3f(srcN[sid](0), srcN[sid](1), srcN[sid](2)));;
				float dThres = 0.25f*scale;
				float nThres = 0.65f;
				printf("mag: %f,  dot: %f \n", magnitude(src - vcur), abs(ncur.dot(nsrc)));
				if (magnitude(src - vcur) < dThres && abs(ncur.dot(nsrc)) < nThres) {
					corres.push_back(std::pair<int, int>(sid, did));
				}
			}
		}
	}


	void Color3DCorrespondence(cv::Mat &src, cv::Mat &dst) {
		
	}

	void ColorDepthCorresMatchRaw(cv::Mat &depth, cv::Mat &color, 
		std::vector<std::pair<cv::Point2f, cv::Point2f>> &corres, Sensor &sensor) {
		
	}

	void ColorDepthCorresMatch(cv::Mat &mapped, cv::Mat &color,
		std::vector<std::pair<cv::Point2f, cv::Point2f>> &corres, Sensor &sensor) {

	}

};


#endif // !_CORRESPONDENCE_FINDING_H

