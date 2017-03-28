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
	for (int i = 0; i < matchsize; i++) {
		int srcid = matches[i].trainIdx;
		int dstid = matches[i].queryIdx;
		if (srcid >= kp_src.size() || dstid >= kp_dst.size()) {
			continue;
		}
		corres_src.push_back(kp_src[srcid].pt);
		corres_dst.push_back(kp_dst[dstid].pt);
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
	for (int i = 0; i < matchsize; i++) {
		int srcid = matches[i].trainIdx;
		int dstid = matches[i].queryIdx;
		if (srcid >= kp_src.size() || dstid >= kp_dst.size()) {
			continue;
		}
		corres_src.push_back(kp_src[srcid].pt);
		corres_dst.push_back(kp_dst[dstid].pt);
	}

	// draw matches
	cv::Mat img_matches;
	drawMatches(src, kp_src, dst, kp_dst, matches, img_matches);

	//-- Show detected matches
	cv::imshow("SURF Matches", img_matches);
	cv::waitKey(1);
}

void ExtractSIFTpointsFLANN(cv::Mat &src, cv::Mat &dst, vector<cv::Point2f> &corres_src, vector<cv::Point2f> &corres_dst, int minHessian = 400) {
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
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	matcher.match(descpritor_src, descriptor_dst, matches);

	// compute min and max dist
	double min_dist = 100;
	double max_dist = 0;
	for (int i = 0; i < descpritor_src.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist);
	printf("-- Min dist : %f \n", min_dist);
	// only use good match
	std::vector< cv::DMatch > good_matches;
	for (int i = 0; i < descpritor_src.rows; i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 0.02))
		{
			good_matches.push_back(matches[i]);
		}
	}

	int matchsize = good_matches.size();
	for (int i = 0; i < matchsize; i++) {
		int srcid = matches[i].trainIdx;
		int dstid = matches[i].queryIdx;
		if (srcid >= kp_src.size() || dstid >= kp_dst.size()) {
			continue;
		}
		corres_src.push_back(kp_src[srcid].pt);
		corres_dst.push_back(kp_dst[dstid].pt);
	}

	// draw matches
	cv::Mat img_matches;
	drawMatches(src, kp_src, dst, kp_dst, good_matches, img_matches);

	//-- Show detected matches
	cv::imshow("SIFT FLANN Matches", img_matches);
	cv::waitKey(1);
}


void ExtractSURFpointsFLANN(cv::Mat &src, cv::Mat &dst, vector<cv::Point2f> &corres_src, vector<cv::Point2f> &corres_dst, int minHessian = 400) {
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
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	matcher.match(descpritor_src, descriptor_dst, matches);

	// compute min and max dist
	double min_dist = 100;
	double max_dist = 0;
	for (int i = 0; i < descpritor_src.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist);
	printf("-- Min dist : %f \n", min_dist);
	// only use good match
	std::vector< cv::DMatch > good_matches;
	for (int i = 0; i < descpritor_src.rows; i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 0.02))
		{
			good_matches.push_back(matches[i]);
		}
	}

	int matchsize = good_matches.size();
	for (int i = 0; i < matchsize; i++) {
		int srcid = matches[i].trainIdx;
		int dstid = matches[i].queryIdx;
		if (srcid >= kp_src.size() || dstid >= kp_dst.size()) {
			continue;
		}
		corres_src.push_back(kp_src[srcid].pt);
		corres_dst.push_back(kp_dst[dstid].pt);
	}


	// draw matches
	cv::Mat img_matches;
	drawMatches(src, kp_src, dst, kp_dst, good_matches, img_matches);

	//-- Show detected matches
	cv::imshow("SURF FLANN Matches", img_matches);
	cv::waitKey(1);
}


// http://www.coldvision.io/2016/06/27/object-detection-surf-knn-flann-opencv-3-x-cuda/
// http://clopinet.com/fextract-book/IntroFS.pdf  // intro to feature extract
void Sample_ExtractSIFT() {
	cv::Mat src = cv::imread("Data/img_color_1_object1.jpg", cv::IMREAD_GRAYSCALE);
	cv::Mat dst = cv::imread("Data/img_color_1.jpg", cv::IMREAD_GRAYSCALE);
	std::vector<cv::Point2f> corres_src, corres_dst;
	ExtractSIFTpoints(src, dst, corres_src, corres_dst, 400);
	ExtractSIFTpointsFLANN(src, dst, corres_src, corres_dst, 400);
	ExtractSURFpoints(src, dst, corres_src, corres_dst, 400);
	ExtractSURFpointsFLANN(src, dst, corres_src, corres_dst, 400);
}
#endif // !_SIFT_MATCHING_H