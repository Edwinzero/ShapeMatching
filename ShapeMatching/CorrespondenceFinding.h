#pragma once
#ifndef _CORRESPONDENCE_FINDING_H
#define _CORRESPONDENCE_FINDING_H
#include <SIFTmatching.h>
#include <Sensor.h>
#include <Eigen_op.h>
#include <cv_op.h>

namespace CORRES {
	void ProjectiveCorresondence(std::vector<Eigen::Vector4f> &srcP, std::vector<Eigen::Vector4f> &srcN, std::vector<Eigen::Vector4f> &dstP, std::vector<Eigen::Vector4f> &dstN, 
		std::vector<pair<int, int>> &corres, Sensor &sensrc, Sensor &sendst) {
		corres.clear();
		cv::Mat src2dst = sendst.dep_to_gl.inv() * sensrc.dep_to_gl; // wrong.....
		cv::Mat intr = sendst.cali_ir.intr.IntrVec();
		float fx = intr.at<double>(0);
		float fy = intr.at<double>(1);
		float cx = intr.at<double>(2);
		float cy = intr.at<double>(3);
		for (int y = 0; y < 424; y++) {
			for (int x = 0; x < 512; x++) {
				cv::Point3f src(srcP[y * 512 + x](0), srcP[y * 512 + x](1), srcP[y * 512 + x](2)); // world space
				cv::Point3f vsrc = src2dst * src;
				vsrc.x /= vsrc.z;
				vsrc.y /= vsrc.z;
				vsrc.x = vsrc.x * fx + cx;
				vsrc.y = vsrc.y * fy + cy;
				if (vsrc.x < 0 || vsrc.x >= 512 || vsrc.y < 0 || vsrc.y >= 424) {
					continue;
				}
				int id = (int)(vsrc.y) * 512 + (int)(vsrc.x);
				cv::Point3f vdst(dstP[id](0), dstP[id](1), dstP[id](2));	
				// normal not store to buffer yet TODO
				cv::Point3f ndst(dstN[id](0), dstN[id](1), dstN[id](2));	// already in world space			
				cv::Point3f nsrc(srcN[y * 512 + x](0), srcN[y * 512 + x](1), srcN[y * 512 + x](2));
				
				float dThres = 0.8f;
				float nThres = 0.3f;
				//printf("mag: %f,  dot: %f \n", magnitude(src - vdst), abs(ndst.dot(nsrc)));
				if (magnitude(src - vdst)*0.001f < dThres && abs(ndst.dot(nsrc)) < nThres) {
					corres.push_back(std::pair<int, int>(y * 512 + x, id));
				}
			}
		}
	}


	void Color3DCorrespondence(cv::Mat &src, cv::Mat &dst) {
		//ExtractSIFTpointsRANSACFLANN(src, dst, corres_src, corres_dst, 400);
	}

};


#endif // !_CORRESPONDENCE_FINDING_H

