#pragma once
#ifndef _RGBD_MAPPING_H
#define _RGBD_MAPPING_H
#include <Sensor.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
double round(double d)
{
	return floor(d + 0.5);
}
namespace RGBD {
	// img * intr (transfer image to 3D points cam space)
	cv::Point3f ImgToPoint(cv::Mat &intr, int x, int y, float d, const float scalar = 1) {
		cv::Point3f res;
		// here x indicates row, y indicates columns
		float fx = intr.at<double>(0) * scalar;
		float fy = intr.at<double>(1) * scalar;
		float cx = intr.at<double>(2) * scalar;
		float cy = intr.at<double>(3) * scalar;
		res.x = (float)((x - cx) * d / fx);
		res.y = (float)((y - cy) * d / fy);
		res.z = d;
		return res;
	}
	// transfer image from cam space to sensor space then to other cam space for the same Kinect
	cv::Point2i PointToImg(const cv::Mat &trans, cv::Mat &discoefs, cv::Mat &rgb_intr, cv::Point3f &pt) {
		cv::Point3f color_pt = trans * pt;
		float x = color_pt.x / color_pt.z;
		float y = color_pt.y / color_pt.z;
		float k1 = discoefs.at<double>(0);
		float k2 = discoefs.at<double>(1);
		float k3 = discoefs.at<double>(4);
		float p1 = discoefs.at<double>(2);
		float p2 = discoefs.at<double>(3);
		float r2 = x*x + y*y;
		float _2xy = 2 * x * y;
		float _kr = (1.0f + r2*(k1 + r2*(k2 + r2 * k3)));
		float xx = x * _kr + p1 * _2xy + p2 * (r2 + 2 * x*x);
		float yy = y * _kr + p2 * _2xy + p1 * (r2 + 2 * y*y);

		cv::Point2i correlation;
		correlation.x = (int)round((x * (float)rgb_intr.at<double>(0)) + (float)rgb_intr.at<double>(2)) + 5;
		correlation.y = (int)round((y * (float)rgb_intr.at<double>(1)) + (float)rgb_intr.at<double>(3));
		return correlation;
	}

	// Map depth to RGB
	void DepthToRGBMapping(Sensor &sensor, cv::Mat &rgb, cv::Mat &dep, cv::Mat &res) {
		res = cv::Mat(dep.size(), CV_8UC3, cv::Scalar(0));
		cv::Mat trans = sensor.cali_rgb.extr * sensor.cali_ir.extr.inv();
		//unsigned short *row_ptr;
		for (int y = 0; y < res.rows; y++) {
			//row_ptr = dep.ptr<unsigned short>(y);
			for (int x = 0; x < res.cols; x++) {
				int id = y * res.cols + x; // row_ptr[x];
				if (dep.at<unsigned short>(id) <= 0) { //|| dep.at<unsigned short>(id)> 9000) {
					continue;
				}
				float d = dep.at<unsigned short>(id) * 0.001f;
				cv::Point3f dpt = ImgToPoint(sensor.cali_ir.intr.IntrVec(), x, y, d);
				cv::Point2i uv = PointToImg(trans, sensor.cali_rgb.intr.distCoeffs, sensor.cali_rgb.intr.IntrVec(), dpt);
				if (uv.x < 0 || uv.y < 0 || uv.x >= rgb.cols || uv.y >= rgb.rows) {
					continue;
				}

				res.at<cv::Vec3b>(y, x) = rgb.at<cv::Vec3b>(uv);
			}
		}
	}


};

void LoadFrame(cv::Mat &depth, char* path) {
	FILE *fp = fopen(path, "rb");
	if (!fp) {
		return;
	}

	int width = 512, height = 424;
	depth = cv::Mat(height, width, CV_16UC1);
	fread(depth.ptr(), sizeof(unsigned short), width * height, fp);

	cout << "load frame " << endl;
	cout << width << " " << height << endl;

	fclose(fp);
}

void LoadKernelText(vector<char> &str, const char *path) {
	FILE *fp = fopen(path, "rb");
	if (!fp) {
		printf("Load Kernel data fails... \n");
		return;
	}

	fseek(fp, 0, SEEK_END);
	size_t size = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	str.resize(size + 1);
	fread(str.data(), 1, size, fp);
	str[size] = '\0';

	fclose(fp);
}
#endif // !_RGBD_MAPPING_H
