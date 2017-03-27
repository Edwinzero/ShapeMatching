#pragma once
#ifndef _SIFT_MATCHING_H
#define _SIFT_MATCHING_H
#include <cv_op.h>
#include <opencv2\features2d\features2d.hpp>
#include "opencv2/nonfree/features2d.hpp"
using namespace std;
void ExtractSIFTpoints(cv::Mat &src, cv::Mat &dst, vector<cv::Point2f> &corres_src, vector<cv::Point2f> &corres_dst, int minHessian = 400) {
	if (!src.data || !dst.data) {
		return;
	}

	// detect keypoint using sift detector
	cv::SiftFeatureDetector detector;
	std::vector<cv::KeyPoint> kp_src, kp_dst;
	detector.detect(src, kp_src);
	detector.detect(dst, kp_dst);

	// compute descriptors( feature vec )
	cv::SiftDescriptorExtractor extractor;
	cv::Mat descpritor_src, descriptor_dst;
	extractor.compute(src, kp_src, descpritor_src);
	extractor.compute(dst, kp_dst, descriptor_dst);

	// matching descriptor vectors with bruteforce matcher
	cv::BFMatcher matcher(cv::NORM_L2);
	std::vector< cv::DMatch > matches;
	matcher.match(descpritor_src, descriptor_dst, matches);

	int matchsize = matches.size();
	corres_src.resize(matchsize);
	corres_dst.resize(matchsize);
	for (int i = 0; i < matchsize; i++) {
		int srcid = matches[i].trainIdx;
		int dstid = matches[i].queryIdx;
		//corres_src[i] = kp_src[srcid].pt;
		//corres_dst[i] = kp_dst[dstid].pt;
	}

	// draw matches
	cv::Mat img_matches;
	drawMatches(src, kp_src, dst, kp_dst, matches, img_matches);

	//-- Show detected matches
	cv::imshow("SIFT Matches", img_matches);
	cv::waitKey(1);
}

void ExtractSURFpoints(cv::Mat &src, cv::Mat &dst, vector<cv::Point2f> &corres_src, vector<cv::Point2f> &corres_dst, int minHessian = 400) {
	if (!src.data || !dst.data) {
		return;
	}

	// detect keypoint using sift detector
	cv::SurfFeatureDetector detector(400);
	std::vector<cv::KeyPoint> kp_src, kp_dst;
	detector.detect(src, kp_src);
	detector.detect(dst, kp_dst);

	// compute descriptors( feature vec )
	cv::SurfDescriptorExtractor extractor;
	cv::Mat descpritor_src, descriptor_dst;
	extractor.compute(src, kp_src, descpritor_src);
	extractor.compute(dst, kp_dst, descriptor_dst);

	// matching descriptor vectors with bruteforce matcher
	cv::BFMatcher matcher(cv::NORM_L2);
	std::vector< cv::DMatch > matches;
	matcher.match(descpritor_src, descriptor_dst, matches);

	int matchsize = matches.size();
	corres_src.resize(matchsize);
	corres_dst.resize(matchsize);
	for (int i = 0; i < matchsize; i++) {
		int srcid = matches[i].trainIdx;
		int dstid = matches[i].queryIdx;
		//corres_src[i] = kp_src[srcid].pt;
		//corres_dst[i] = kp_dst[dstid].pt;
	}

	// draw matches
	cv::Mat img_matches;
	drawMatches(src, kp_src, dst, kp_dst, matches, img_matches);

	//-- Show detected matches
	cv::imshow("SURF Matches", img_matches);
	cv::waitKey(1);
}

// http://www.coldvision.io/2016/06/27/object-detection-surf-knn-flann-opencv-3-x-cuda/
// http://clopinet.com/fextract-book/IntroFS.pdf  // intro to feature extract
void Sample_ExtractSIFT() {
	cv::Mat src = cv::imread("Data/img_color_1_object1.jpg", cv::IMREAD_GRAYSCALE);
	cv::Mat dst = cv::imread("Data/img_color_1.jpg", cv::IMREAD_GRAYSCALE);
	std::vector<cv::Point2f> corres_src, corres_dst;
	ExtractSIFTpoints(src, dst, corres_src, corres_dst, 400);
	ExtractSURFpoints(src, dst, corres_src, corres_dst, 400);
}
#endif // !_SIFT_MATCHING_H