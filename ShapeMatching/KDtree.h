#pragma once
#ifndef _KDTREE_H
#define _KDTREE_H
#include <iostream>
#include <vector>
#include <Eigen_op.h>
using namespace std;
using namespace Eigen;

class KDtree {
private:
	typedef struct KDnode {
		KDnode *left, *right;
		bool leaf;
		int splitAxis;
		float splitValue;
		float tmin, tmax;
		Vector3f point;

	public:
		KDnode():splitAxis(-1), tmin(1.0f), tmax(-1.0f), splitValue(0.0f), leaf(false){
			left = right = NULL;
		}

		KDnode(Vector3f &point) {
			this->point = point;
			splitAxis = -1;
			splitValue = 0.0f;
			tmin = 1.0f; tmax = -1.0f;
			leaf = false;
			left = right = NULL;
		}
	} KDnode;
public:
	KDnode *root;
	int k;
public:
	KDtree(): k(0){
		root = new KDnode();
	}
	~KDtree() {
		delete root;
	}

	inline void ConstructTree(std::vector<Vector3f> &points, int dim = 3) {
		if (points.empty()) {
			return;
		}
		k = dim;
		for (std::vector<Vector3f>::iterator it = points.begin(); it < points.end(); it++) {
			KDnode *n = new KDnode(*it);
			root = Insert(root, n, 0);  // insert from root
		}
	}

	inline bool ConstructTree(std::vector<Vector3f> &points) {
		if (points.empty()) { return false; }
		k = 3;	 // set by default
		root = BuildTree(points, 0);
	}

	inline bool SearchPoint(Vector3f &point) {
		KDnode *n = new KDnode(point);
		return Search(root, n, 0);
	}

// tree operations
private:
	// Node operations
	inline KDnode* NewNode(Vector3f p) {
		KDnode *res = new KDnode();
		res->point = p;
		return res;
	}

	inline KDnode* BuildTree(std::vector<Vector3f> &points, int depth) {
		if (points.size() == 1) {
			KDnode *leaf = new KDnode(points[0]);
			leaf->leaf = true;
			return leaf;
		}

		int axis = depth % k;		
		std::vector<Vector3f> smallP, largeP;
		smallP.reserve(points.size());
		largeP.reserve(points.size());
		float splitValue = 0.0f;
		SplitPlane(points, axis, smallP, largeP, splitValue);		// spilt data based median on current axis to two categories
		// recursive
		KDnode *left = BuildTree(smallP, depth + 1);
		KDnode *right = BuildTree(largeP, depth + 1);
		// link node
		KDnode *n = new KDnode();
		n->splitAxis = axis;
		n->splitValue = splitValue;
		n->left = left;
		n->right = right;
		return n;
	}

	// Tree operations ( less efficiency )
	inline KDnode* Insert(KDnode *root, KDnode *n, int depth) {
		if (root == NULL) {
			root = n; 
			root->splitAxis = 0; // x axis is default split direction for root
			return root;
		}

		int axis = depth % k;
		n->splitAxis = axis;
		if (n->point(axis) < root->point(axis)) {
			root->left = Insert(root->left, n, depth + 1);
		}
		else {
			root->right = Insert(root->right, n, depth + 1);
		}
		return root;
	}

	inline bool Search(KDnode *root, KDnode *n, int depth) {
		if (root == NULL) {	return false; }
		if (root->point == n->point) { return true; }
		int axis = depth % k;
		if (n->point(axis) < root->point(axis)) {
			return Search(root->left, n, depth + 1);
		}
		else {
			return Search(root->right, n, depth + 1);
		}
	}

// utility algorithms
private:
	inline void SplitPlane(std::vector<Vector3f> &points, int axis, std::vector<Vector3f> &smallP, std::vector<Vector3f> &largeP, float &splitValue) {
		// QuickSelect
		splitValue = QuickSelect(points, 0, points.size() - 1, (int)(points.size()*0.5), axis);
		for (std::vector<Vector3f>::iterator it = points.begin(); it < points.end(); it++) {
			if (it->data[axis] <= splitValue) {
				smallP.push_back(*it);
			}
			else {
				largeP.push_back(*it);
			}
		}
	}

	int Partition(std::vector<Vector3f> &points, int left, int right, int pivotID, int axis) {
		float pivotValue = points[pivotID](axis);
		Vector3f tmp = points[right];
		points[right] = points[pivotID];
		points[pivotID] = tmp;
		int resIndex = left;
		for (std::vector<Vector3f>::iterator it = points.begin(); it < points.end(); it++) {
			if (it->data[axis] < points[resIndex](axis)) {
				tmp = *it;
				*it = points[resIndex];
				points[resIndex] = tmp;
				resIndex++;
			}
		}
		tmp = points[resIndex];
		points[resIndex] = points[right];
		points[right] = tmp;
		return resIndex;
	}

	float QuickSelect(std::vector<Vector3f> &points, int left, int right, int kID, int axis) {
		if (left == right) {
			return points[left](axis);
		}
		int pivotID = left + floor(static_cast<float>(rand() % (right - left + 1)));
		pivotID = Partition(points, left, right, pivotID, axis);
		if (kID == pivotID) {
			return points[kID](axis);
		}
		else if (kID < pivotID) {
			return QuickSelect(points, left, pivotID - 1, kID, axis);
		}
		else {
			return QuickSelect(points, pivotID + 1, right, kID, axis);
		}
	}
};

void Sample_KDtree(void) {
	KDtree *tree = new KDtree();
	int samples[8][3] = { { 3, 6, 5 },{ 17, 15, 4 },{ 13, 15, 28 },{ 6, 12, 12 },
	{ 9, 1, -4 },{ 2, 7, 10 },{ 10, 19, 0 } };
	std::vector<Vector3f> points(8);
	for (int i = 0; i < 8; i++) {
		points[i] = Vector3f(samples[i][0], samples[i][1], samples[i][2]);
	}

	tree->ConstructTree(points, 3);
	Vector3f p0 = Vector3f(10, 19, 0);		// found
	(tree->SearchPoint(p0)) ? cout << "Found\n" : cout << "Not Found\n";
	Vector3f p1 = Vector3f(3, 6, 5);		// found
	(tree->SearchPoint(p1)) ? cout << "Found\n" : cout << "Not Found\n";
	Vector3f p2 = Vector3f(1, 19, 10);		// not found
	(tree->SearchPoint(p2)) ? cout << "Found\n" : cout << "Not Found\n";
}
#endif //_KD_TREE_H