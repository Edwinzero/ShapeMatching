#pragma once
#ifndef _FAST_GLOBAL_REIGISTRATION_H
#define _FAST_GLOBAL_REIGISTRATION_H
#include <vector>
#include <flann\flann.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen\Dense>
#include <Eigen\Cholesky>
#include <FPFH.h>

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
	vector < pair<int, int> > correspondence;

	// data normalization
	vector< Vector3f > means;
	float globalScale;
	float startScale;
public:
	void LoadFeature(const vector<Vector3f> &points, const vector<VectorXf> &features);
	void ReadFeature(const char *filepath);
	void LoadCorrespondence(const vector<Vector3f> &points);
	void LoadPoints(const vector<Vector3f> &src, const vector<Vector3f> &dst);
	void AdvancedMatching();
	void NormalizePoints();
	double OptimizePairwise(bool decrease_mu, int numIter, int src, int dst);
	Matrix4f GetRes();

	// some internal functions
	void ReadFeature(const char* filepath, vector<Vector3f>& points, vector<VectorXf>& feature);

	void SearchFLANNTree(flann::Index<flann::L2<float>>* index,
		VectorXf& input,
		std::vector<int>& indices,
		std::vector<float>& dists,
		int nn);
};

inline void FastGlobalReg::ReadFeature(const char * filepath)
{
	printf("[FGR] Read Featrue...\n");
	vector<Vector3f> ps;
	vector<VectorXf> fs;
	ReadFeature(filepath, ps, fs);
	LoadFeature(ps, fs);
}

inline void FastGlobalReg::LoadFeature(const vector<Vector3f>& points, const vector<VectorXf>& features)
{
	printf("[FGR] Load Featrue...\n");
	this->points.push_back(points);
	this->features.push_back(features);
}

// only work when src = dst
inline void FastGlobalReg::LoadCorrespondence(const vector<Vector3f> &points) {
	int size = points.size();
	correspondence.clear();
	correspondence.resize(size);
	for (int i = 0; i < size; i++) {
		correspondence[i] = std::pair<int, int>(i, i);
	}
}

// only work when src = dst
inline void FastGlobalReg::LoadPoints(const vector<Vector3f>& src, const vector<Vector3f>& dst)
{
	int size = src.size();
	points.clear();
	points.resize(2);
	points[0].resize(size);
	for (int i = 0; i < size; i++) {
		points[0][i] = src[i];
	}
	points[1].resize(size);
	for (int i = 0; i < size; i++) {
		points[1][i] = dst[i];
	}
}

// Correspondence test (3 steps)
inline void FastGlobalReg::AdvancedMatching()
{
	// Make the bigger size be source set
	int fi = 0; int fj = 1;
	printf("[FGR] Advanced matching : [%d - %d]\n", fi, fj);
	bool swapped = false;
	if (points[fj].size() > points[fi].size()) {
		int temp = fi;
		fi = fj;
		fj = temp;
		swapped = true;
	}
	int nPti = points[fi].size();
	int nPtj = points[fj].size();

	//  BUILD FLANNTREE
	// build FLANNTree - fi
	int rows = features[fi].size();
	int dim = features[fi][0].size();

	std::vector<float> dataset_fi(rows * dim);
	flann::Matrix<float> dataset_mat_fi(&dataset_fi[0], rows, dim);

	for (int y = 0; y < rows; y++) {
		for(int x = 0; x < dim; x++){
			dataset_fi[x + dim*y] = features[fi][y][x];
		}
	}
	flann::Index<flann::L2<float>> feature_tree_i(dataset_mat_fi, flann::KDTreeSingleIndexParams(15));
	feature_tree_i.buildIndex();

	// build FLANNTree - fj
	rows = features[fj].size();
	dim = features[fj][0].size();

	std::vector<float> dataset_fj(rows * dim);
	flann::Matrix<float> dataset_mat_fj(&dataset_fj[0], rows, dim);

	for (int y = 0; y < rows; y++) {
		for (int x = 0; x < dim; x++) {
			dataset_fj[x + dim*y] = features[fj][y][x];
		}
	}
	flann::Index<flann::L2<float>> feature_tree_j(dataset_mat_fj, flann::KDTreeSingleIndexParams(15));
	feature_tree_j.buildIndex();

	bool crosscheck = true;		// flag for second test
	bool tuple = true;			// flag for third test

	std::vector<int> corres_K, corres_K2;
	std::vector<float> dis;
	std::vector<int> ind;

	std::vector<std::pair<int, int>> corres;
	std::vector<std::pair<int, int>> corres_cross;
	std::vector<std::pair<int, int>> corres_ij;
	std::vector<std::pair<int, int>> corres_ji;

	// INITIAL MATCHING
	std::vector<int> i_to_j(nPti, -1);
	for (int j = 0; j < nPtj; ++j) {
		SearchFLANNTree(&feature_tree_i, features[fj][j], corres_K, dis, 1);
		int i = corres_K[0];
		if (i_to_j[i] == -1) { // means there is no correspondence stored yet
			SearchFLANNTree(&feature_tree_j, features[fi][i], corres_K, dis, 1);
			i_to_j[i] = corres_K[0];
		}
		corres_ji.push_back(std::pair<int, int>(i, j));
	}

	for (int i = 0; i < nPti; i++) {
		if (i_to_j[i] != -1) {
			corres_ij.push_back(std::pair<int, int>(i, i_to_j[i]));
		}
	}
	int ncorres_ij = corres_ij.size();
	int ncorres_ji = corres_ji.size();

	for (int i = 0; i < ncorres_ij; ++i) {
		corres.push_back(std::pair<int, int>(corres_ij[i].first, corres_ij[i].second));
	}
	for (int j = 0; j < ncorres_ji; ++j) {
		corres.push_back(std::pair<int, int>(corres_ji[j].first, corres_ji[j].second));
	}
	printf("points are remained : %d\n", (int)corres.size());

	// CROSS CHECK
	// input : corres_ij, corres_ji
	// output : corres
	if (crosscheck)
	{
		printf("\t[cross check] ");
		// build data structure for cross check
		corres.clear();
		corres_cross.clear();
		std::vector<std::vector<int>> Mi(nPti);
		std::vector<std::vector<int>> Mj(nPtj);
		int ci, cj;
		for (int i = 0; i < ncorres_ij; ++i) {
			ci = corres_ij[i].first;
			cj = corres_ij[i].second;
			Mi[ci].push_back(cj);
		}
		for (int j = 0; j < ncorres_ji; ++j) {
			ci = corres_ji[j].first;
			cj = corres_ji[j].second;
			Mj[cj].push_back(ci);
		}

		// cross check
		for (int i = 0; i < nPti; ++i){
			for (int ii = 0; ii < Mi[i].size(); ++ii){
				int j = Mi[i][ii];
				for (int jj = 0; jj < Mj[j].size(); ++jj){
					if (Mj[j][jj] == i){
						corres.push_back(std::pair<int, int>(i, j));
						corres_cross.push_back(std::pair<int, int>(i, j));
					}
				}
			}
		}
		printf("points are remained : %d\n", (int)corres.size());
	}

	// TUPLE CONSTRAINT
	// input : corres
	// output : corres
	if (tuple)
	{
		srand(time(NULL));
		printf("\t[tuple constraint] ");
		int rand0, rand1, rand2;
		int idi0, idi1, idi2;
		int idj0, idj1, idj2;
		float scale = TUPLE_SCALE;
		int ncorr = corres.size();
		int number_of_trial = ncorr * 100;
		std::vector<std::pair<int, int>> corres_tuple;

		int cnt = 0;
		int i;
		for (i = 0; i < number_of_trial; i++) {
			rand0 = rand() % ncorr;
			rand1 = rand() % ncorr;
			rand2 = rand() % ncorr;

			idi0 = corres[rand0].first;
			idj0 = corres[rand0].second;
			idi1 = corres[rand1].first;
			idj1 = corres[rand1].second;
			idi2 = corres[rand2].first;
			idj2 = corres[rand2].second;

			// collect 3 points from i-th fragment
			Eigen::Vector3f pti0 = points[fi][idi0];
			Eigen::Vector3f pti1 = points[fi][idi1];
			Eigen::Vector3f pti2 = points[fi][idi2];

			float li0 = (pti0 - pti1).norm();
			float li1 = (pti1 - pti2).norm();
			float li2 = (pti2 - pti0).norm();

			// collect 3 points from j-th fragment
			Eigen::Vector3f ptj0 = points[fj][idj0];
			Eigen::Vector3f ptj1 = points[fj][idj1];
			Eigen::Vector3f ptj2 = points[fj][idj2];

			float lj0 = (ptj0 - ptj1).norm();
			float lj1 = (ptj1 - ptj2).norm();
			float lj2 = (ptj2 - ptj0).norm();

			if ((li0 * scale < lj0) && (lj0 < li0 / scale) &&
				(li1 * scale < lj1) && (lj1 < li1 / scale) &&
				(li2 * scale < lj2) && (lj2 < li2 / scale))
			{
				corres_tuple.push_back(std::pair<int, int>(idi0, idj0));
				corres_tuple.push_back(std::pair<int, int>(idi1, idj1));
				corres_tuple.push_back(std::pair<int, int>(idi2, idj2));
				cnt++;
			}

			if (cnt >= TUPLE_MAX_CNT)
				break;
		}

		printf("%d tuples (%d trial, %d actual).\n", cnt, number_of_trial, i);
		corres.clear();

		for (int i = 0; i < corres_tuple.size(); ++i){
			corres.push_back(std::pair<int, int>(corres_tuple[i].first, corres_tuple[i].second));
		}
	}

	if (swapped){
		std::vector<std::pair<int, int>> temp;
		for (int i = 0; i < corres.size(); i++)
			temp.push_back(std::pair<int, int>(corres[i].second, corres[i].first));
		corres.clear();
		corres = temp;
	}

	printf("\t[final] matches %d.\n", (int)corres.size());
	correspondence = corres;
}

// Normalize scale of points.
// X' = (X-\mu)/scale
inline void FastGlobalReg::NormalizePoints()
{
	int num = 2;
	float scale = 0.0f;
	means.clear();
	for (int i = 0; i < num; ++i) {
		float max_scale = 0.0f;
		// compute mean
		Vector3f mean;
		mean.setZero();
		int ptsize = points[i].size();	// each sample set size
		for (int ii = 0; ii < ptsize; ++ii) {
			Vector3f p(points[i][ii](0), points[i][ii](1), points[i][ii](2));
			mean = mean + p;
		}
		mean = mean / ptsize;
		means.push_back(mean);
		printf("[FGR] normalize points :: mean[%d] = [%f %f %f]\n", i, mean(0), mean(1), mean(2));

		for (int ii = 0; ii < ptsize; ++ii) {
			points[i][ii](0) -= mean(0);
			points[i][ii](1) -= mean(1);
			points[i][ii](2) -= mean(2);
		}

		// compute scale
		for (int ii = 0; ii < ptsize; ++ii) {
			Vector3f p(points[i][ii](0), points[i][ii](1), points[i][ii](2));
			float temp = p.norm();	// extract mean in the previous stage
			if (temp > max_scale) {
				max_scale = temp;
			}
		}
		if (max_scale > scale) {
			scale = max_scale;
		}
	}

	// mean of the scale variation
	if (USE_ABSOLUTE_SCALE) {
		globalScale = 1.0f;
		startScale = scale;
	}
	else {
		globalScale = scale; // keep the maximum scale.
		startScale = 1.0f;
	}
	printf("[FGR] normalize points :: global scale : %f\n", globalScale);

	for (int i = 0; i < num; ++i){
		int ptsize = points[i].size();
		for (int ii = 0; ii < ptsize; ++ii){
			points[i][ii](0) /= globalScale;
			points[i][ii](1) /= globalScale;
			points[i][ii](2) /= globalScale;
		}
	}
}

inline double FastGlobalReg::OptimizePairwise(bool decrease_mu, int numIter, int src, int dst)
{
	printf("[FGR] Pairwise rigid pose optimization\n");
	double miu;
	int nIter = numIter;
	resTransform = Eigen::Matrix4f::Identity();
	miu = startScale;

	// make another copy of points[d];
	int npdst = points[dst].size();
	vector<Vector3f> pdst_copy(npdst);
	for (int i = 0; i < npdst; i++) {
		pdst_copy[i] = points[dst][i];
	}
	if (correspondence.size() < 10) { return -1.0; }

	std::vector<double> lpq(correspondence.size(), 1.0);	// lpq initial to all 1
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();	// temp res
	const int nvariable = 6;
	for (int i = 0; i < numIter; i++) {
		// graduated non-convexity
		if (decrease_mu) {
			if (i % 4 == 0 && miu > MAX_CORR_DIST) {
				miu /= DIV_FACTOR;
			}
		}

		Eigen::MatrixXd JTJ(nvariable, nvariable);
		Eigen::MatrixXd JTr(nvariable, 1);
		Eigen::MatrixXd J(nvariable, 1);
		JTJ.setZero();
		JTr.setZero();

		double r;
		double r2 = 0.0;
		for (int c = 0; c < correspondence.size(); c++) {
			int ii = correspondence[c].first;
			int jj = correspondence[c].second;
			Eigen::Vector3f p = points[src][ii];
			Eigen::Vector3f q = pdst_copy[jj];
			Eigen::Vector3f rpq = p - q;

			// compute lpq for sample c
			float sqrt_lpq = miu / (rpq.dot(rpq) + miu);
			lpq[c] = sqrt_lpq * sqrt_lpq;

			// set parameters for Jacobian matrix
			J.setZero();
			J(1) = -q(2);
			J(2) = q(1);
			J(3) = -1;
			r = rpq(0);
			JTJ += J * J.transpose() * lpq[c];
			JTr += J * r * lpq[c];
			r2 += r * r * lpq[c];

			J.setZero();
			J(0) = q(2);
			J(2) = -q(0);
			J(4) = -1;
			r = rpq(1);
			JTJ += J * J.transpose() * lpq[c];
			JTr += J * r * lpq[c];
			r2 += r * r * lpq[c];
		
			J.setZero();
			J(0) = -q(1);
			J(1) = q(0);
			J(5) = -1;
			r = rpq(2);
			JTJ += J * J.transpose() * lpq[c];
			JTr += J * r * lpq[c];
			r2 += r * r * lpq[c];

			r2 += (miu * (1.0 - sqrt(lpq[c])) * (1.0 - sqrt(lpq[c])));
		}// end construct linear system

		Eigen::MatrixXd result(nvariable, 1);
		result = -JTJ.llt().solve(JTr);

		Eigen::Affine3d affmat;
		affmat.linear() = (Eigen::Matrix3d)Eigen::AngleAxisd(result(2), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(result(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(result(0), Eigen::Vector3d::UnitX());
		affmat.translation() = Eigen::Vector3d(result(3), result(4), result(5));
		Eigen::Matrix4f delta = affmat.matrix().cast<float>();		// use cast to transfer double to float

		trans = delta * trans;

		// transform point cloud
		Eigen::Matrix3f R = delta.block<3, 3>(0, 0);
		Eigen::Vector3f t = delta.block<3, 1>(0, 3);
		for (int cnt = 0; cnt < npdst; cnt++) {
			pdst_copy[cnt] = R * pdst_copy[cnt] + t;
		}
	}

	resTransform = trans * resTransform;
	return miu;
}

inline Matrix4f FastGlobalReg::GetRes()
{

	Eigen::Matrix3f R;
	Eigen::Vector3f t;
	R = resTransform.block<3, 3>(0, 0);
	t = resTransform.block<3, 1>(0, 3);

	Eigen::Matrix4f transtemp;
	transtemp.fill(0.0f);

	// need to transform result from normalized space to global space scale
	transtemp.block<3, 3>(0, 0) = R;
	transtemp.block<3, 1>(0, 3) = -R*means[1] + t*globalScale + means[0];
	transtemp(3, 3) = 1;

	return transtemp;
}

inline void FastGlobalReg::ReadFeature(const char * filepath, vector<Vector3f>& points, vector<VectorXf>& feature)
{
	points.clear();
	feature.clear();

	FILE *fp = fopen(filepath, "rb");
	int numvert;
	fread(&numvert, sizeof(int), 1, fp);
	int numdim;
	fread(&numdim, sizeof(int), 1, fp);

	// read from binary feature file
	for (int i = 0; i < numvert; i++) {
		Vector3f point_v;
		fread(&point_v(0), sizeof(float), 3, fp);

		VectorXf feature_v(numdim);
		fread(&feature_v, sizeof(float), numdim, fp);
		points.push_back(point_v);
		feature.push_back(feature_v);
	}
	fclose(fp);
}

inline void FastGlobalReg::SearchFLANNTree(flann::Index<flann::L2<float>>* index, VectorXf& input, std::vector<int>& indices, std::vector<float>& dists, int nn)
{
	int rows_t = 1;
	int dim = input.size();

	std::vector<float> query;
	query.resize(rows_t*dim);
	for (int i = 0; i < dim; i++)
		query[i] = input(i);
	flann::Matrix<float> query_mat(&query[0], rows_t, dim);

	indices.resize(rows_t*nn);
	dists.resize(rows_t*nn);
	flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
	flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);

	index->knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(128));
}
#endif // !_FAST_GLOBAL_REIGISTRATION_H
