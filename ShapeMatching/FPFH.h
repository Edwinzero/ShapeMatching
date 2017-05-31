#pragma once
#ifndef _FPFH_H
#define _FPFH_H
#include <vector>
#include <Eigen_op.h>
#include <KDtree.h>
#include <flann\flann.hpp>
using namespace std;
using namespace Eigen;

class PFHfeature {
	double alpha;
	double phi;
	double theta;

	void computeFeature(Vector4f ps, Vector4f ns, Vector4f pd, Vector4f nd) {
		Eigen::Vector3f u, v, w, unitDist;
		u = Eigen::Vector3f(ns(0), ns(1), ns(2));
		Vector4f pdps = pd - ps;
		unitDist = Eigen::Vector3f(pdps(0), pdps(1), pdps(2));
		unitDist.normalize();
		v = unitDist.cross(u);
		w = u.cross(v);

		Eigen::Vector3f nd3 = Eigen::Vector3f(nd(0), nd(1), nd(2));
		alpha = v.dot(nd3);
		phi = u.dot(unitDist);
		theta = atan2(u.dot(nd3), w.dot(nd3));
	}

public:
	// src_p, src_n, dst_p, dst_n
	PFHfeature(Vector4f ps, Vector4f ns, Vector4f pd, Vector4f nd) {
		// compute dot to distinguish source and dst point
		double dots = ps.dot(ns);
		double dott = pd.dot(nd);
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
public:
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
	SPFH(){}
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
	FPFH(){}
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
class PFHEsitmator{
public:

	std::vector<Vector4f> pointsPtr;
	std::vector<Vector4f> normalsPtr;
	float knn_radius_;
	int  k_;

	std::vector<SPFH> spfhs;
	std::vector<FPFH> fpfhs;

	std::vector<VectorXf> feature;

	VectorXf fpfh_histogram_;
	Eigen::MatrixXf  hist_t_, hist_a_, hist_p_;

public:
	PFHEsitmator() : knn_radius_(0), k_(0) {	}
	~PFHEsitmator() {  }

	void SetProcessingData(std::vector<Vector4f> &p, std::vector<Vector4f> &n) {
		if (p.empty() || n.empty() || p.size() != n.size()) {
			return;
		}
		// cpy to ptr buf
		pointsPtr.resize(p.size());
		normalsPtr.resize(n.size());
		for (int i = 0; i < p.size(); i++){
			pointsPtr[i] = p[i];
			normalsPtr[i] = n[i];
		}
	}

	void ComputeSPFHfeatures() {
		// construct kdtree
		int rows = pointsPtr.size();
		int dim = pointsPtr[0].size();

		std::vector<float> dataset_fi(rows * dim);
		::flann::Matrix<float> dataset_mat_fi(&dataset_fi[0], rows, dim);

		for (int y = 0; y < rows; y++) {
			for (int x = 0; x < dim; x++) {
				dataset_fi[x + dim*y] = pointsPtr[y][x];
			}
		}
		::flann::KDTreeSingleIndex<::flann::L2<float>> feature_tree(dataset_mat_fi, ::flann::KDTreeSingleIndexParams(15));
		feature_tree.buildIndex();

		spfhs.resize(rows);
		float binsize = 0.4f;
		std::vector<int> neighbors;
		std::vector<float> dists;
		// search K nearest neigbhors
		for (int i = 0; i < rows; i++) {
			SearchFLANNTree(&feature_tree, pointsPtr[i], neighbors, dists, 5);
			SPFH *tmp = new SPFH(-3.1415926, 3.1415926, binsize);
			for (int j = 0; j < neighbors.size(); j++) {
				PFHfeature p(pointsPtr[i], normalsPtr[i], pointsPtr[neighbors[j]], normalsPtr[neighbors[j]]);
				printf("PFHfeature %d: %f, %f, %f\n", i, p.a(), p.the(), p.ph());
				tmp->AddToHist(&p);
			}
			spfhs[i] = *tmp;

			// check
			checkSPFH(&spfhs[i], neighbors.size());
		}
	}

	void GenerateHistogram() {
		

	}


	inline void SearchFLANNTree(::flann::KDTreeSingleIndex<::flann::L2<float>>* index, Vector4f& input, std::vector<int>& indices, std::vector<float>& dists, int nn)
	{
		int rows_t = 1;
		int dim = input.size();

		std::vector<float> query;
		query.resize(rows_t*dim);
		for (int i = 0; i < dim; i++)
			query[i] = input(i);
		::flann::Matrix<float> query_mat(&query[0], rows_t, dim);

		indices.resize(rows_t*nn);
		dists.resize(rows_t*nn);
		::flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
		::flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);

		index->knnSearch(query_mat, indices_mat, dists_mat, nn, ::flann::SearchParams(128));
	}
	

	void checkSPFH(SPFH *s, int size) {
		double* alpha = s->hist_a();
		double* theta = s->hist_the();
		double* phi = s->hist_ph();

		int countAlpha = 0;
		int countTheta = 0;
		int countPhi = 0;

		for (int i = 0; i < s->histSize; i++) {
			if (isnan(alpha[i])) std::cout << "NAN in alpha spfh" << std::endl;
			if (isnan(theta[i])) std::cout << "NAN in theta spfh" << std::endl;
			if (isnan(phi[i])) std::cout << "NAN in phi spfh" << std::endl;

			countAlpha += alpha[i];
			countTheta += theta[i];
			countPhi += phi[i];
		}

		if (size != countAlpha) printf("Entry missing in alpha SPFH histogram\n"); 
		if (size != countTheta) printf("Entry missing in theta SPFH histogram\n");
		if (size != countPhi) printf("Entry missing in phi SPFH histogram\n");
	}

	/**
	 compute FPFH feature and return the feature set correspondent to the input points data
	 [out] feature set
	**/
	void computeFeature(std::vector<Eigen::VectorXf> &output) {
		std::vector<int> nn_indices(k_);
		std::vector<float> nn_dists(k_);
		std::vector<int> spfh_hist_lookup;
		ComputeSPFHSignatures(spfh_hist_lookup, hist_t_, hist_a_, hist_p_);

		for (int idx = 0; idx < pointsPtr.size(); idx++) {
			// ... and remap the nn_indices values so that they represent row indices in the spfh_hist_* matrices 
			// instead of indices into surface_->points
			for (size_t i = 0; i < nn_indices.size(); ++i)
				nn_indices[i] = spfh_hist_lookup[nn_indices[i]];
			// Compute the FPFH signature (i.e. compute a weighted combination of local SPFH signatures) ...
			WeightPointSPFHSignature(hist_t_, hist_a_, hist_p_, nn_indices, nn_dists, fpfh_histogram_);

			// ...and copy it into the output cloud
			for (int d = 0; d < fpfh_histogram_.size(); ++d)
				output.push_back(fpfh_histogram_);
		}
	}


private:
	/** \brief Compute the 4-tuple representation containing the three angles and one distance between two points
	* represented by Cartesian coordinates and normals.
	* \note For explanations about the features, please see the literature mentioned above (the order of the
	* features might be different).
	* \param[in] p1 the first XYZ point
	* \param[in] n1 the first surface normal
	* \param[in] p2 the second XYZ point
	* \param[in] n2 the second surface normal
	* \param[out] f1 the first angular feature (angle between the projection of nq_idx and u) (theta)
	* \param[out] f2 the second angular feature (angle between nq_idx and v)				  (alpha)
	* \param[out] f3 the third angular feature (angle between np_idx and |p_idx - q_idx|)	  (phi)
	* \param[out] f4 the distance feature (p_idx - q_idx)
	**/
	void ComputePairFeature(const Eigen::Vector4f &p1, const Eigen::Vector4f &n1,
		const Eigen::Vector4f &p2, const Eigen::Vector4f &n2,
		float &f1, float &f2, float &f3, float &f4) {
		Eigen::Vector3f u, v, w, unitDist;
		u = Eigen::Vector3f(n1(0), n1(1), n1(2));
		Vector4f pdps = p2 - p1;
		unitDist = Eigen::Vector3f(pdps(0), pdps(1), pdps(2));
		f4 = unitDist.norm();
		unitDist.normalize();
		v = unitDist.cross(u);
		w = u.cross(v);

		Eigen::Vector3f nd = Eigen::Vector3f(n2(0), n2(1), n2(2));
		f1 = u.dot(unitDist);					// theta
		f2 = v.dot(nd);							// alpha
		f3 = atan2(u.dot(nd), w.dot(nd));		// phi
	}

	//https://pdfs.semanticscholar.org/5aee/411f0b4228ba63c85df0e8ed64cab5844aed.pdf
	//https://github.com/PointCloudLibrary/pcl/wiki/Overview-and-Comparison-of-Features
	//https://github.com/PointCloudLibrary/pcl/blob/master/features/include/pcl/features/impl/fpfh.hpp
	void ComputePointSPFHSignature(int pid, int row, const vector<int> &nnIdx,
		Eigen::MatrixXf &hist_t, Eigen::MatrixXf &hist_a, Eigen::MatrixXf &hist_p) {
		Eigen::Vector4f pfh_tuple;
		// get num of bins from histgram size
		int numbin_a = static_cast<int>(hist_a.cols());		// alpha
		int numbin_t = static_cast<int>(hist_t.cols());		// theta
		int numbin_p = static_cast<int>(hist_p.cols());		// phi

		float hist_incr = 100.0f / static_cast<float>(nnIdx.size() - 1);

		// Iterate over all the points in the neighborhood
		/// \brief Float constant = 1.0 / (2.0 * M_PI) 
		float d_pi = 1.0f / (2.0f * static_cast<float> (3.1415926f));
		for (int i = 0; i < nnIdx.size(); i++) {
			if (pid == nnIdx[i]) {
				continue;
			}
			// Compute the pair P to NNiv
			ComputePairFeature(pointsPtr[pid], normalsPtr[pid], pointsPtr[nnIdx[i]], normalsPtr[nnIdx[i]],
				pfh_tuple[0], pfh_tuple[1], pfh_tuple[2], pfh_tuple[3]);
			// Normalize the f1, f2, f3 features and push them in the histogram
			// f1: theta
			int hidx = static_cast<int>(floor(numbin_t * ((pfh_tuple[0] + 3.1415926f) * d_pi)));
			if (hidx < 0)           hidx = 0;
			if (hidx >= numbin_t) hidx = numbin_t - 1;
			hist_t(row, hidx) += hist_incr;
			// f2: alpha
			hidx = static_cast<int>(floor(numbin_a * ((pfh_tuple[0] + 1.0f) * 0.5f)));
			if (hidx < 0)           hidx = 0;
			if (hidx >= numbin_a) hidx = numbin_a - 1;
			hist_a(row, hidx) += hist_incr;
			// f3: phi
			hidx = static_cast<int>(floor(numbin_p * ((pfh_tuple[0] + 1.0f) * 0.5f)));
			if (hidx < 0)           hidx = 0;
			if (hidx >= numbin_p) hidx = numbin_p - 1;
			hist_p(row, hidx) += hist_incr;
		}
	}


	void WeightPointSPFHSignature(const Eigen::MatrixXf &hist_t, const Eigen::MatrixXf &hist_a, const Eigen::MatrixXf &hist_p,
		const std::vector<int> &indices, const std::vector<float> &dists, Eigen::VectorXf &fpfh_histogram) {
		if (indices.size() != dists.size()) {
			printf("<<WeightPointSPFHsignature>>  ==>  indice size not equal to dists size\n");
			return;
		}
		double sum_t = 0.0, sum_a = 0.0, sum_p = 0.0;  // sum theta, alpha, phi
		float weight = 0.0f, val_t, val_a, val_p;

		// get num of bins from histogram size
		int binTheta = static_cast<int>(hist_t.cols());
		int binAlpha = static_cast<int>(hist_a.cols());
		int binPhi = static_cast<int>(hist_p.cols());
		int binThetaAlpha = binTheta + binAlpha;

		// clear the histogram
		fpfh_histogram.setZero(binTheta + binAlpha + binPhi);

		// use the entire patch
		for (int idx = 0; idx < indices.size(); idx++) {
			if (dists[idx] == 0.0f) {
				continue;
			}
			// standard weighting function
			weight = 1.0f / dists[idx];
			// weight SPFH of the query point with the SPFH of its neighbors
			for (int ti = 0; ti < binTheta; ti++) {
				val_t = hist_t(indices[idx], ti) * weight;
				sum_t += val_t;
				fpfh_histogram[ti] += val_t;
			}
			for (int ai = 0; ai < binAlpha; ai++) {
				val_a = hist_a(indices[idx], ai) * weight;
				sum_a += val_a;
				fpfh_histogram[ai + binTheta] += val_a;
			}
			for (int pi = 0; pi < binTheta; pi++) {
				val_p = hist_p(indices[idx], pi) * weight;
				sum_p += val_p;
				fpfh_histogram[pi + binThetaAlpha] += val_p;
			}
		}

		if (sum_t != 0)
			sum_t = 100.0 / sum_t;           // histogram values sum up to 100
		if (sum_a != 0)
			sum_a = 100.0 / sum_a;           // histogram values sum up to 100
		if (sum_p != 0)
			sum_p = 100.0 / sum_p;           // histogram values sum up to 100

		// Adjust final FPFH values
		for (int f1_i = 0; f1_i < binTheta; ++f1_i)
			fpfh_histogram[f1_i] *= static_cast<float> (sum_t);
		for (int f2_i = 0; f2_i < binAlpha; ++f2_i)
			fpfh_histogram[f2_i + binTheta] *= static_cast<float> (sum_a);
		for (int f3_i = 0; f3_i < binPhi; ++f3_i)
			fpfh_histogram[f3_i + binThetaAlpha] *= static_cast<float> (sum_p);
	}

	void ComputeSPFHSignatures(std::vector<int> &spfh_hist_lookup,
		Eigen::MatrixXf &hist_t, Eigen::MatrixXf &hist_a, Eigen::MatrixXf &hist_p) {
		// allocate space to hold the NN search results
		std::vector<int> nn_indices(k_);
		std::vector<float> nn_dists(k_);

		std::set<int> spfh_indices;
		spfh_hist_lookup.resize(pointsPtr.size());

		// Build a list of (unique) indices for which we will need to compute SPFH signatures
		// (We need an SPFH signature for every point that is a neighbor of any point in input_[indices_])
		// construct kdtree
		int rows = pointsPtr.size();
		int dim = pointsPtr[0].size();

		std::vector<float> dataset_fi(rows * dim);
		::flann::Matrix<float> dataset_mat_fi(&dataset_fi[0], rows, dim);

		for (int y = 0; y < rows; y++) {
			for (int x = 0; x < dim; x++) {
				dataset_fi[x + dim*y] = pointsPtr[y][x];
			}
		}
		::flann::KDTreeSingleIndex<::flann::L2<float>> feature_tree(dataset_mat_fi, ::flann::KDTreeSingleIndexParams(15));
		feature_tree.buildIndex();
		for (int i = 0; i < pointsPtr.size(); i++) {  // iterate all indices
			SearchFLANNTree(&feature_tree, pointsPtr[i], nn_indices, nn_dists, 5);
			spfh_indices.insert(nn_indices.begin(), nn_indices.end());
		}

		// initial array that will store the SPFH signatures
		size_t data_size = spfh_indices.size();
		hist_t.setZero(data_size, 11);			// default is 11
		hist_a.setZero(data_size, 11);
		hist_p.setZero(data_size, 11);

		// Compute SPFH signatures for every point that needs them
		std::set<int>::iterator spfh_indice_itr = spfh_indices.begin();
		for (int i = 0; i < static_cast<int>(spfh_indices.size()); ++i) {
			int pid = *spfh_indice_itr;
			++spfh_indice_itr;

			// find neighbor around pidx
			SearchFLANNTree(&feature_tree, pointsPtr[pid], nn_indices, nn_dists, 5);
			// Estimate the SPFH signature around pidx
			ComputePointSPFHSignature(pid, i, nn_indices, hist_t, hist_a, hist_p);
			// Populate a lookup table for converting a point index to its corresponding row in the spfh_hist_* matrices
			spfh_hist_lookup[pid] = i;
		}
	}
};
#endif // !_FPFH_H
