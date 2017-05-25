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
		double dots = ps->dot(*ns);
		double dott = pd->dot(*nd);
		// put small dot product point to source
		if (dots > dott) {
			computeFeature(pd, nd, ps, ns);
		}
		else {
			computeFeature(ps, ns, pd, nd);
		}
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

class PFHhistogram {
protected:
	double *hist_alpha, *hist_phi, *hist_theta;
	double min, max;
	double binSize;
	int histSize;
public:
	PFHhistogram() {}
	PFHhistogram(double min, double max, double bin) : min(min), max(max), binSize(bin) {
		histSize = ceil((max - min) / binSize);
		hist_alpha = (double*)calloc(histSize, sizeof(double));
		hist_phi = (double*)calloc(histSize, sizeof(double));
		hist_theta = (double*)calloc(histSize, sizeof(double));
	}
	~PFHhistogram() {
		free(hist_alpha);
		free(hist_phi);
		free(hist_theta);
	}

	double* hist_a() {
		return hist_alpha;
	}

	double* hist_ph() {
		return hist_phi;
	}

	double* hist_the() {
		return hist_theta;
	}
};

class SPFH : public PFHhistogram {
public:
	SPFH(double min, double max, double bin) : PFHhistogram(min, max, bin) {

	}
	virtual ~SPFH(){}

	void AddToHist(PFHfeature *f) {
		if (f->a() >= min && f->a() <= max) {
			int aID = (int)(f->a() - min) / binSize;
			hist_alpha[aID]++;
		}
		if (f->ph() >= min && f->ph() <= max) {
			int phID = (int)(f->ph() - min) / binSize;
			hist_phi[phID]++;
		}
		if (f->the() >= min && f->the() <= max){
			int theID = (int)(f->the() - min) / binSize;
			hist_theta[theID]++;
		}
	}
};

// FPFH( p) = SPFH(p) +  (1/k)*SUM((1/wk)*SPFH(pk))
class FPFH : public PFHhistogram {
public:
	int numberOfEntries;
public:
	FPFH(double min, double max, double bin) : PFHhistogram(min, max, bin) {

	}
	virtual ~FPFH() {}

	void AddToHist(SPFH* spfh, int k, double w) {
		int count = 0;
		// compute histogram for FPFH by using k neighbors and weight as w
		for (int i = 0; i < histSize; i++) {
			hist_alpha[i] += (spfh->hist_a()[i] / w);
			hist_phi[i] += (spfh->hist_ph()[i] / w);
			hist_theta[i] += (spfh->hist_the()[i] / w);
			if (!hist_alpha[i] || !hist_phi[i] || !hist_theta[i]) {
				count++;
			}
		}
		for (int i = 0; i < histSize; i++) {
			hist_alpha[i] /= k;
			hist_phi[i] /= k;
			hist_theta[i] /= k;
		}
		numberOfEntries = count;
	}
};

// procedure:
// 1. input points + normal
// 2. construct KD tree
// 3. Run KNN search method
// 4. for each point p in cloud: get the nearest neighbors of p; for each pair of neighbors, compute the three angular features; bin all result into one output histogram
class PFH{
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

	void SetProcessingData(std::vector<Vector4f> &p, std::vector<Vector4f> &n) {
		// cpy pt and normal
		points.resize(p.size());
		normals.resize(n.size());
		for (int i = 0; i < points.size(); i++) {
			Vector3f tp(p[i](0), p[i](1), p[i](2));
			Vector3f tn(n[i](0), n[i](1), n[i](2));
			points[i] = tp;
			normals[i] = tn;
		}
		if (!points.empty()) {
			pointsPtr.resize(points.size());
			for (int i = 0; i < points.size(); i++){
				pointsPtr[i] = &points[i];
			}

			root->ConstructTree(pointsPtr);
		}
	}

	void GenerateHistogram() {

	}

	
};
#endif // !_FPFH_H
