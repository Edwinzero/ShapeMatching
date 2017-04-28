#pragma once
#include <ThirdParty\GMS-Feature-Matcher-master\include\GMS.h>

void GridMatch(cv::Mat &img1, cv::Mat &img2) {
	cv::pyrDown(img1, img1);
	cv::pyrDown(img2, img2);
	vector<cv::KeyPoint> kp1, kp2;
	cv::Mat d1, d2;
	vector<cv::DMatch> matches_all, matches_grid;

	cv::Ptr<cv::ORB> orb = cv::ORB::create(10000);
	orb->setFastThreshold(0);
	orb->detectAndCompute(img1, cv::Mat(), kp1, d1);
	orb->detectAndCompute(img2, cv::Mat(), kp2, d2);

#ifdef USE_GPU
	GpuMat gd1(d1), gd2(d2);
	Ptr<cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
	matcher->match(gd1, gd2, matches_all);
#else
	cv::BFMatcher matcher(cv::NORM_HAMMING);
	matcher.match(d1, d2, matches_all);
#endif

	// GMS filter
	GMS gms;
	gms.init(img1.size(), img2.size(), kp1, kp2, matches_all);
	gms.setParameter(20, 20);
	matches_grid = gms.getInlier(0);

	cout << "Get total " << matches_grid.size() << " matches." << endl;

	cv::Mat show = DrawInlier(img1, img2, kp1, kp2, matches_grid, 1);
	cv::imshow("show", show);
	cv::waitKey(0);
}


void GenCorrespondenceFromGridMatch(cv::Mat &img1, cv::Mat &img2, std::vector<std::pair<int, int>> &corres) {
	vector<cv::KeyPoint> kp1, kp2;
	cv::Mat d1, d2;
	vector<cv::DMatch> matches_all, matches_grid;

	cv::Ptr<cv::ORB> orb = cv::ORB::create(10000);
	orb->setFastThreshold(0);
	orb->detectAndCompute(img1, cv::Mat(), kp1, d1);
	orb->detectAndCompute(img2, cv::Mat(), kp2, d2);

#ifdef USE_GPU
	cv::GpuMat gd1(d1), gd2(d2);
	cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
	matcher->match(gd1, gd2, matches_all);
#else
	cv::BFMatcher matcher(cv::NORM_HAMMING);
	matcher.match(d1, d2, matches_all);
#endif

	// GMS filter
	GMS gms;
	gms.init(img1.size(), img2.size(), kp1, kp2, matches_all);
	gms.setParameter(20, 20);
	matches_grid = gms.getInlier(0);

	cout << "Get total " << matches_grid.size() << " matches." << endl;
	cv::Mat show = DrawInlier(img1, img2, kp1, kp2, matches_grid, 1);
	cv::imshow("show", show);
	cv::waitKey(0);

	// save to corres
	corres.resize(matches_grid.size());
	int size = corres.size();
	for (int i = 0; i < size; i++) {
		corres[i].first = kp1[matches_grid[i].queryIdx].pt.y * 512 + kp1[matches_grid[i].queryIdx].pt.x;
		corres[i].second = kp2[matches_grid[i].trainIdx].pt.y * 512 + kp2[matches_grid[i].trainIdx].pt.x;
	}
	
}

// display the color and depth matching
void GenCorrespondencMatchingBetweenRGBDEP(cv::Mat &depth, cv::Mat color, std::vector<std::pair<int, int>> &corres, const int p) {

}

// reject false color match coordinates with depth image
void FalseMatchingRejection(cv::Mat &dsrc, cv::Mat &ddst, std::vector<std::pair<int, int>> &corres) {

}