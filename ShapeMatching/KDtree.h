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
		int depth;
		int splitAxis;
		Vector3f *location;
		std::vector<Vector3f*> points;

	public:
		KDnode():splitAxis(-1), depth(0), leaf(false){
			left = right = NULL;
		}
		~KDnode() {
			delete left, right;
		}

		KDnode(Vector3f &point) {
			*location = point;
			leaf = false;
			left = right = NULL;
		}

		friend std::ostream& operator<<(std::ostream& out, const KDnode& n) {
			out.width(4);
			out.precision(3);
			if (n.location == NULL) {
				return out;
			}
			out << *n.location << "\n";

			if (n.right != NULL) {
				out << "Depth: " << n.depth + 1 << "right tree: \n";
				out << *(n.right) << "\n";
			}
			if (n.left != NULL) {
				out << "Depth: " << n.depth + 1 << "left tree: \n";
				out << *(n.left) << "\n";
			}
			return out;
		}
	} KDnode;
public:
	KDnode *root;
	int k;
public:
	KDtree(): k(3){
		root = new KDnode();
	}
	~KDtree() {
		delete root;
	}

	inline bool ConstructTree(std::vector<Vector3f> &points) {
		if (points.empty()) { return false; }
		k = 3;	 // set by default
		root = BuildTree(points, 0);
	}

	inline bool SearchPoint(Vector3f &point) {
		return false;
	}

	inline void SearchKnearest(Vector3f *point, float r, std::vector<Vector3f*> &neighbors) {
		if (root == NULL) {
			return;
		}
		// point in sphere

		// left node not null

		// right node not null
	}

// tree operations
private:
	inline KDnode* BuildTree(std::vector<Vector3f*> &points, int depth) {
		if (points.size() == 1) {
			KDnode *leaf = new KDnode(points[0]);
			leaf->leaf = true;
			leaf->depth = depth;
			return leaf;
		}

		int axis = depth % k;		
		std::vector<Vector3f> smallP, largeP;
		smallP.reserve(points.size()*0.5);
		largeP.reserve(points.size()*0.5);
		Vector3f splitPoint(0.0);
		SplitPlane(points, axis, smallP, largeP, splitPoint);		// spilt data based median on current axis to two categories
		// recursive
		KDnode *left = BuildTree(smallP, depth + 1);
		KDnode *right = BuildTree(largeP, depth + 1);
		// link node
		KDnode *n = new KDnode();
		n->splitAxis = axis;
		n->location = &splitPoint;
		n->left = left;
		n->right = right;
		return n;
	}

// utility algorithms
private:
	inline void SplitPlane(std::vector<Vector3f*> &points, int axis, std::vector<Vector3f> &smallP, std::vector<Vector3f> &largeP, Vector3f &splitPoint) {
		// QuickSelect
		splitPoint = QuickSelect(points, 0, points.size() - 1, (int)(points.size()*0.5), axis); // O(nlogn)
		for (std::vector<Vector3f>::iterator it = points.begin(); it < points.end(); it++) {	// O(n)
			if (it->data()[axis] <= splitPoint(axis)) {
				smallP.push_back(*it);
			}
			else {
				largeP.push_back(*it);
			}
		}
	}

	int Partition(std::vector<Vector3f*> &points, int left, int right, int pivotID, int axis) {
		float pivotValue = points[pivotID]->data()[axis];
		Vector3f tmp = *points[right];
		points[right] = points[pivotID];
		points[pivotID] = &tmp;
		int resIndex = left;
		for (std::vector<Vector3f*>::iterator it = points.begin(); it < points.end(); it++) {
			if (it->data()[axis] < points[resIndex](axis)) {
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

	Vector3f QuickSelect(std::vector<Vector3f*> &points, int left, int right, int kID, int axis) {
		if (left == right) {
			return *points[left];
		}
		int pivotID = left + floor(static_cast<float>(rand() % (right - left + 1)));
		pivotID = Partition(points, left, right, pivotID, axis);
		if (kID == pivotID) {
			return points[kID];
		}
		else if (kID < pivotID) {
			return QuickSelect(points, left, pivotID - 1, kID, axis);
		}
		else {
			return QuickSelect(points, pivotID + 1, right, kID, axis);
		}
	}

	bool IntersectSphere(float r, Vector3f &center) {

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

	//tree->ConstructTree(points, 3);
	Vector3f p0 = Vector3f(10, 19, 0);		// found
	(tree->SearchPoint(p0)) ? cout << "Found\n" : cout << "Not Found\n";
	Vector3f p1 = Vector3f(3, 6, 5);		// found
	(tree->SearchPoint(p1)) ? cout << "Found\n" : cout << "Not Found\n";
	Vector3f p2 = Vector3f(1, 19, 10);		// not found
	(tree->SearchPoint(p2)) ? cout << "Found\n" : cout << "Not Found\n";
}
#endif //_KD_TREE_H