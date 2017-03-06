#pragma once
#ifndef _FPFH_H
#define _FPFH_H
#include <vector>
#include <Eigen_op.h>
#include <KDtree.h>
using namespace std;
using namespace Eigen;


class PFHfeature {
	double alpha;
	double phi;
	double theta;

	void computeFeature(Vector3f* ps, Vector3f* ns, Vector3f* pd, Vector3f* nd) {
		Eigen::Vector3f u, v, w, unitDist;
		u = *ns;
		unitDist = *pd - *ps;
		unitDist.normalize();
		v = unitDist.cross(u);
		w = u.cross(v);

		alpha = v.dot(*nd);
		phi = u.dot(unitDist);
		theta = atan2(u.dot(*nd), w.dot(*nd));
	}

public:
	// src_p, src_n, dst_p, dst_n
	PFHfeature(Vector3f* ps, Vector3f* ns, Vector3f* pd, Vector3f* nd) {
		// compute dot to distinguish source and dst point
	}
	
	double a() {
		return alpha;
	}
	double ph() {
		return phi;
	}
	double the() {
		return theta;
	}
};

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
	std::vector<Vector3f*> pointsPtr;
	std::vector<Vector3f> normals;
	float knn_radius;

	std::vector<VectorXf> feature;
	KDnode *root;

public:
	PFH() : knn_radius(0) {	}
	~PFH() { delete root; }

	void SetProcessingData(std::vector<Vector3f> &p, std::vector<Vector3f> &n) {
		points = p;
		normals = n;
		if (!points.empty()) {
			pointsPtr.resize(points.size());
			for (int i = 0; i < points.size(); i++){
				pointsPtr[i] = &points[i];
			}

			root->ConstructTree(pointsPtr);
		}
	}

	void SetKNNsearch() {

	}

	void PFHestimation(std::vector<VectorXf> &res) {

	}
};
#endif // !_FPFH_H
