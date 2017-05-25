#pragma once
#include <ThirdParty\GMS-Feature-Matcher-master\include\GMS.h>
#include <random>

void GridMatch(cv::Mat &img1, cv::Mat &img2) {
	//cv::pyrDown(img1, img1);
	//cv::pyrDown(img2, img2);
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

	std::vector<cv::DMatch> show_grid(200);
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 eng(rd()); // seed the generator
	std::uniform_int_distribution<> distr(1700, 2000); // define the range
	for (int i = 0; i < 200; i++) {
		int id = distr(eng);
		show_grid[i] = matches_grid[id];
	}
	//cv::Mat show = DrawInlier(img1, img2, kp1, kp2, matches_grid, 1);
	cv::Mat show = DrawInlier(img1, img2, kp1, kp2, show_grid, 2);
	cv::imshow("show", show);
	cv::imwrite("GMS_match.png", show);
	cv::waitKey(0);
}

/*
	Generate reliable and rich color correspondence set
*/
inline void GenCorrespondenceFromGridMatch(cv::Mat &img1, cv::Mat &img2, std::vector<std::pair<cv::Point2f, cv::Point2f>> &corres, int isshow = 0) {
	corres.clear();
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
	//cv::FlannBasedMatcher matcher;
	//matcher.match(d1, d2, matches_all);
#endif

	// GMS filter
	GMS gms;
	gms.init(img1.size(), img2.size(), kp1, kp2, matches_all);
	gms.setParameter(20, 20);
	matches_grid = gms.getInlier(0);

	cout << "Get total " << matches_grid.size() << " matches." << endl;
	cv::Mat show = DrawInlier(img1, img2, kp1, kp2, matches_grid, 2);
	if (isshow) {
		ImgShow("matching result", show, 1024, 424);
	}
	// save to corres
	corres.resize(matches_grid.size());
	int size = corres.size();
	for (int i = 0; i < size; i++) {
		corres[i].first = kp1[matches_grid[i].queryIdx].pt;
		corres[i].second = kp2[matches_grid[i].trainIdx].pt;
	}
	
}

// utility
inline cv::Mat VerifyDrawInlier(cv::Mat &src1, cv::Mat &src2, std::vector<std::pair<cv::Point2f, cv::Point2f>> &corres, int type = 2) {
	const int height = max(src1.rows, src2.rows);
	const int width = src1.cols + src2.cols;
	cv::Mat output(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
	src1.copyTo(output(cv::Rect(0, 0, src1.cols, src1.rows)));
	src2.copyTo(output(cv::Rect(src1.cols, 0, src2.cols, src2.rows)));

	if (type == 1)
	{
		for (size_t i = 0; i < corres.size(); i++)
		{
			cv::Point2f left(corres[i].first);
			cv::Point2f right = (cv::Point2f(corres[i].second) + cv::Point2f((float)src1.cols, 0.f));
			line(output, left, right, cv::Scalar(0, 255, 255));
		}
	}
	else if (type == 2)
	{
		for (size_t i = 0; i < corres.size(); i++)
		{
			cv::Point2f left(corres[i].first);
			cv::Point2f right = (cv::Point2f(corres[i].second) + cv::Point2f((float)src1.cols, 0.f));
			//line(output, left, right, cv::Scalar(255, 0, 0));
			circle(output, left, 1, cv::Scalar(0, 255, 255), 2);
			circle(output, right, 1, cv::Scalar(0, 255, 0), 2);
		}
	}
	return output;
}

/*
	using point feature histogram to filtering the color correspondence 3D point set to provide accurate correspondence
	IN: src point+normal, dst point+normal, corres, sensor
	Out: filtered correspondence set
*/
inline void PointFeatureHistogramFiltering(std::vector<Eigen::Vector4f> &srcP, std::vector<Eigen::Vector4f> &srcN, std::vector<Eigen::Vector4f> &dstP, std::vector<Eigen::Vector4f> &dstN,
	std::vector<std::pair<cv::Point2f, cv::Point2f>> &corres,
	std::vector<std::pair<cv::Point2f, cv::Point2f>> &result, int isshow = 0) {

}

