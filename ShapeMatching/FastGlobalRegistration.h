#pragma once
#ifndef _FAST_GLOBAL_REIGISTRATION_H
#define _FAST_GLOBAL_REIGISTRATION_H
#include <vector>
#include <flann\flann.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen\Dense>
#include <Eigen\Cholesky>

using namespace std;
using namespace Eigen;
//typedef vector<Vector3f> Points;
//typedef vector<VectorXf> Feature;
#define DIV_FACTOR			1.4		// Division factor used for graduated non-convexity
#define USE_ABSOLUTE_SCALE	0		// Measure distance in absolute scale (1) or in scale relative to the diameter of the model (0)
#define MAX_CORR_DIST		0.025	// Maximum correspondence distance (also see comment of USE_ABSOLUTE_SCALE)
#define ITERATION_NUMBER	64		// Maximum number of iteration
#define TUPLE_SCALE			0.95	// Similarity measure used for tuples of feature points.
#define TUPLE_MAX_CNT		1000	// Maximum tuple numbers.

class FastGlobalReg {
public:
	// container
	vector < vector<Vector3f> >	points;
	vector < vector<VectorXf> > features;
	Matrix4f resTransform;
	vector < pair<int, int> > corres;

	// data normalization
	vector< Vector3f > means;
	float globalScale;
	float StartScale;
public:
	void LoadFeature(const vector<Vector3f> &points, const vector<VectorXf> &features);
	void ReadFeature(const char *filepath);
	void AdvancedMatching();
	void NormalizePoints();
	double OptimizePairwise(bool decrease_mu, int numIter);
	Matrix4f GetRes();

	// some internal functions
	void ReadFeature(const char* filepath, vector<Vector3f>& points, vector<VectorXf>& feature);

	void SearchFLANNTree(flann::Index<flann::L2<float>>* index,
		VectorXf& input,
		std::vector<int>& indices,
		std::vector<float>& dists,
		int nn);
};

inline void FastGlobalReg::LoadFeature(const vector<Vector3f>& points, const vector<VectorXf>& features)
{
}

inline void FastGlobalReg::ReadFeature(const char * filepath)
{
}

inline void FastGlobalReg::AdvancedMatching()
{
}

inline void FastGlobalReg::NormalizePoints()
{
}

inline double FastGlobalReg::OptimizePairwise(bool decrease_mu, int numIter)
{
	return 0.0;
}

inline Matrix4f FastGlobalReg::GetRes()
{
	return Matrix4f();
}

inline void FastGlobalReg::ReadFeature(const char * filepath, vector<Vector3f>& points, vector<VectorXf>& feature)
{
}

inline void FastGlobalReg::SearchFLANNTree(flann::Index<flann::L2<float>>* index, VectorXf& input, std::vector<int>& indices, std::vector<float>& dists, int nn)
{
}
#endif // !_FAST_GLOBAL_REIGISTRATION_H
