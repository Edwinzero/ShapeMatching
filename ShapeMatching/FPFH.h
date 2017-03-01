#pragma once
#ifndef _FPFH_H
#define _FPFH_H
#include <vector>
#include <Eigen_op.h>
#include <KDtree.h>
using namespace std;
using namespace Eigen;

class FPFH {
public:

public:

};

// procedure:
// 1. input points + normal
// 2. construct KD tree
// 3. Run KNN search method
// 4. for each point p in cloud: get the nearest neighbors of p; for each pair of neighbors, compute the three angular features; bin all result into one output histogram
class PFH {
public:
	std::vector<Vector3f> points;
	std::vector<Vector3f> normals;
	float knn_radius;

	std::vector<VectorXf> feature;

public:
	PFH() : knn_radius(0){	}

	void SetProcessingData(std::vector<Vector3f> &p, std::vector<Vector3f> &n) {
		points = p;
		normals = n;
	}

	void SetKNNsearch() {

	}

	void PFHestimation(std::vector<VectorXf> &res) {

	}
};
#endif // !_FPFH_H
