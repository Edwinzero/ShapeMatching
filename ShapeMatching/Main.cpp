#include <RenderPipeline.h>
#if 1
int main(int argc, char **argv) {
	//Sample_KDtree();
	//Sample_ExtractSIFT();
	Run_Render(argc, argv, "Test");
	return 0;
}
//*/
#else
int main(void)
{
	cv::Mat distortMat = (cv::Mat_<double>(1, 5) << 0.039814, -0.000789, -0.004186, 0.001102, -0.044900);
	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1064.968952, 0.000000, 956.643087, 0.000000, 1063.149460, 529.584358, 0.000000, 0.000000, 1.000000);

	cv::Mat img = cv::imread("rgb_color.jpeg", 1);
	cv::cvtColor(img, img, CV_BGR2GRAY);
	img.convertTo(img, CV_8UC1);
	//cv::Mat uPhoto = img.clone();
	cv::Mat uPhoto = cv::Mat(1080, 1920, CV_8UC1, cv::Scalar(0));

	double k1 = distortMat.at<double>(0, 0);
	double k2 = distortMat.at<double>(0, 1);
	double p1 = distortMat.at<double>(0, 2);
	double p2 = distortMat.at<double>(0, 3);
	double k3 = distortMat.at<double>(0, 4);
	double  fx = cameraMatrix.at<double>(0, 0);
	double  cx = cameraMatrix.at<double>(0, 2);
	double  fy = cameraMatrix.at<double>(1, 1);
	double  cy = cameraMatrix.at<double>(1, 2);
	double z = 1.;

	for (int i = 0; i < img.cols; i++)
	{
		for (int j = 0; j < img.rows; j++)
		{
			/* Solved by removing this...
			double x = (double)i*fx + cx*z;
			double y = (double)j*fy + cy*z;
			double r2 = x*x + y*y;

			double dx = 2 * p1*i*j + p2*(r2 + 2 * i*i);
			double dy = p1*(r2 + 2 * j*j) + 2 * p2*i*j;
			double scale = 1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2;

			double xCorr = x*scale + dx;
			double yCorr = y*scale + dy;*/

			// ...and adding this:
			double x = (i - cx) / fx;
			double y = (j - cy) / fy;
			double r2 = x*x + y*y;

			double dx = 2 * p1*x*y + p2*(r2 + 2 * x*x);
			double dy = p1*(r2 + 2 * y*y) + 2 * p2*x*y;
			double scale = (1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);

			double xBis = x*scale + dx;
			double yBis = y*scale + dy;

			double xCorr = xBis*fx + cx;
			double yCorr = yBis*fy + cy;

			if (xCorr >= 0 && xCorr < uPhoto.cols && yCorr >= 0 && yCorr < uPhoto.rows)
			{
				uPhoto.at<uchar>(yCorr, xCorr) = img.at<uchar>(j, i);
			}
		}
	}

	ImgShow("correced img", uPhoto, 960, 540);

	cv::Mat uPhotoAuto;
	cv::undistort(img, uPhotoAuto, cameraMatrix, distortMat);
	ImgShow("cv correced img", uPhotoAuto, 960, 540);
	cv::waitKey(0);

	return 0;
}
#endif