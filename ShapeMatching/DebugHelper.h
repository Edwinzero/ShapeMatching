#pragma once
#ifndef _DEBUG_HELPER_H
#define _DEBUG_HELPER_H
namespace DHELPER{
	void PointCloudStatisticalResult(std::vector<Eigen::Vector4f> points) {
		Eigen::Vector4f mean;
		mean.setZero();
		float maxx = -1, minx = 1000, maxy = -1, miny = 1000, maxz = -1, minz = 1000;
		int size = points.size();
		//for (std::vector<Eigen::Vector4f>::iterator it = points.begin(); it < points.end(); it++) {
		for(int i = 0; i < size; i++){
			if (points[i](2) > 5000 || points[i](2) < 0) {
				continue;
			}
			mean += points[i];
			if (maxx < points[i](0)) {
				maxx = points[i](0);
			}
			if (minx > points[i](0)) {
				minx = points[i](0);
			}
			if (maxy < points[i](1)) {
				maxy = points[i](1);
			}
			if (miny > points[i](1)) {
				miny = points[i](1);
			}
			if (maxz < points[i](2)) {
				maxz = points[i](2);
			}
			if (minz > points[i](2)) {
				minz = points[i](2);
			}
		}
		mean /= size;
		printf("PointCloud data info: \n");
		printf("Mean : %f, %f, %f\n", mean(0), mean(1), mean(2));
		printf("[ x ] MIN: %f, Max: %f \n", minx, maxx);
		printf("[ y ] MIN: %f, Max: %f \n", miny, maxy);
		printf("[ z ] MIN: %f, Max: %f \n", minz, maxz);
	}

	void PointCloudValidNormal(std::vector<Eigen::Vector4f> normals) {
		int size = normals.size();
		int valid = 0;
		for (int i = 0; i < size; i++) {
			if (normals[i](0) == 0 && normals[i](1) == 0 && normals[i](2) == 0) {
				continue;
			}
			if (normals[i].norm() > 0.999999f && normals[i].norm() < 1.000001f) {
				printf("not normalized!!\n");
				continue;
			}
			valid++;
		}
		printf("PointCloud valid normal info:  %d\n", valid);
	}

	int convertColor(float v) {
		v += 1.0f;  // [-1, 1] -> [0, 2]
		v *= 0.5f;  // ->[0, 1]
		return (int)(v*255.0f);
	}
	void CheckNormalMap(std::vector<Eigen::Vector4f> normals) {
		int size = normals.size();
		cv::Mat res(cv::Size(512, 424), CV_8UC3, cv::Scalar(0,0,0));
		for (int y = 0; y < res.rows; y++) {
			for (int x = 0; x < res.cols; x++) {
				int id = (y*res.cols + x);
				int B = convertColor(normals[id](0));  if (B < 0 || B > 255) continue;
				int G = convertColor(normals[id](1));  if (G < 0 || G > 255) continue;
				int R = convertColor(normals[id](2));  if (R < 0 || R > 255) continue;
				res.at<cv::Vec3b>(y, x)[0] = B; // B 
				res.at<cv::Vec3b>(y, x)[1] = G; // G
				res.at<cv::Vec3b>(y, x)[2] = R; // R
			}
		}

		ImgShow("check normal", res, 512, 424);
	}

};


#endif // !_DEBUG_HELPER_H
