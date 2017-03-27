#pragma once
#ifndef _OPTICAL_FLOW_H
#define _OPTICAL_FLOW_H
#include <opencv2\opencv.hpp>
#include <opencv2\features2d\features2d.hpp>
using namespace std;
class OpticalFlow {
public:
	vector<cv::Point2i> result;
public:
	OpticalFlow() {

	}

	void OpticalFlow() {

	}
};

// https://github.com/sahakorn/Python-optical-flow-tracking/blob/master/optical_flow.py
// http://vision.middlebury.edu/flow/floweval-ijcv2011.pdf
// http://courses.cs.washington.edu/courses/cse576/book/ch11.pdf ***** important
void SampleOpticalFlow() {

}
#endif // !_OPTICAL_FLOW_H
