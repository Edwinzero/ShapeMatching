#pragma once
#ifndef _GAUSSIAN_NOISE_H
#define _GAUSSIAN_NOISE_H
#include <iostream>
#include <opencv2\opencv.hpp>

class Gaussian {
public:
	int size;
public:
	Gaussian(){}
	

	
};



namespace FILTER {
	// Genreal setting
	const int KERNEL_LENGTH = 9;
	const int HALF_KERNEL_LENGTH = (KERNEL_LENGTH - 1) * 0.5;
	cv::Mat CreateGuassianKernel(const float sigma) {
		// Create kernel
		double s = 2.0f * sigma * sigma;
		double r2 = 0;
		double sum = 0;
		cv::Mat kernel = cv::Mat(KERNEL_LENGTH, KERNEL_LENGTH, CV_64F);
		for (int x = -HALF_KERNEL_LENGTH; x <= HALF_KERNEL_LENGTH; x++) {
			for (int y = -HALF_KERNEL_LENGTH; y <= HALF_KERNEL_LENGTH; y++) {
				r2 = sqrt(static_cast<double>(x*x + y*y));
				r2 *= r2;
				kernel.at<double>(x + HALF_KERNEL_LENGTH, y + HALF_KERNEL_LENGTH) = exp(-(r2 / s)) / CV_PI * s;
				sum += kernel.at<double>(x + HALF_KERNEL_LENGTH, y + HALF_KERNEL_LENGTH);
			}
		}
		// normalize kernel
		for (int i = 0; i < KERNEL_LENGTH; i++) {
			for (int j = 0; j < KERNEL_LENGTH; j++) {
				kernel.at<double>(i, j) /= sum;
			}
		}
		return kernel;
	}
	cv::Mat GaussianFilter(const cv::Mat &img, int diameter, const cv::Mat &kernel) {
		cv::Mat res = cv::Mat(img.size(), img.type(), cv::Scalar(0));
		for (int y = 0; y < img.rows; y++) {
			for (int x = 0; x < img.cols; x++) {
				double val = 0;
				// Window 5 by 5
				for (int offy = -HALF_KERNEL_LENGTH; offy <= HALF_KERNEL_LENGTH; offy++) {
					if (y + offy < 0 || y + offy >= img.rows) {
						continue;
					}
					for (int offx = -HALF_KERNEL_LENGTH; offx <= HALF_KERNEL_LENGTH; offx++) {
						if (x + offx < 0 || x + offx >= img.cols) {
							continue;
						}
						double weight = kernel.at<double>(HALF_KERNEL_LENGTH + offy, HALF_KERNEL_LENGTH + offx);
						val += weight * static_cast<double>(img.at<uchar>(y + offy, x + offx));
					}
				}
				res.at<uchar>(y * img.cols + x) = static_cast<uchar>(val);
			}
		}
		return res;
	}

	cv::Mat BilateralFilter1(const cv::Mat &img) {
		cv::Mat in;
		img.convertTo(in, CV_32FC1);
		cv::Mat tmp = cv::Mat(img.rows, img.cols, CV_32FC1);
		cv::bilateralFilter(img, tmp, 6, 20, 20);
		cv::Mat res;
		tmp.convertTo(res, CV_16UC1);
		return res;
	}

	cv::Mat BilateralFilter(const cv::Mat &img, int diameter, float intensity_sigma, float space_sigma) {
		// prepare
		float invSpace_sigma = 0.5f / (space_sigma * space_sigma);
		float invIntensity_sigma = 0.5f / (intensity_sigma * intensity_sigma);
		cv::Mat input;
		img.convertTo(input, CV_32FC1);

		const int halfKerLen = (diameter - 1) * 0.5;

		cv::Mat res = cv::Mat(img.rows, img.cols, CV_32FC1);
		for (int y = 0; y < res.rows; y++) {
			for (int x = 0; x < res.cols; x++) {
				float val = 0;
				float sumWeight = 0;
				// process in window
				for (int offy = -halfKerLen; offy <= halfKerLen; offy++) {
					if (y + offy < 0 || y + offy >= input.rows) {
						continue;
					}
					for (int offx = -halfKerLen; offx <= halfKerLen; offx++) {
						if (x + offx < 0 || x + offx >= input.cols) {
							continue;
						}
						// compute weigting
						float valij = input.at<float>(y, x) * 0.001f;
						float valkl = input.at<float>(y + offy, x + offx) * 0.001f;
						float expG = -(offx*offx + offy*offy) * invSpace_sigma;
						float expC = -((valij - valkl)*(valij - valkl)) * invIntensity_sigma;
						float weight = exp(expG + expC);
						sumWeight += weight;
						val += weight * input.at<float>(y + offy, x + offx);
					}
				}
				// normalize final value
				res.at<float>(y, x) = val / sumWeight;
			}
		}
		// convert back to CV_16UC1
		cv::Mat ret = cv::Mat(img.rows, img.cols, img.type());
		res.convertTo(ret, img.type());
		return ret;
	}
};

#endif // !_GAUSSIAN_NOISE_H
