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
		int depth;
		Vector3f *location;
		std::vector<Vector3f*> points;

	public:
		KDnode() :depth(0) {
			left = right = NULL;
		}
		~KDnode() {
			delete left, right;
		}

		KDnode(Vector3f *point) {
			*location = *point;
			left = right = NULL;
		}

		bool IsInSphere(Vector3f *p, float r, Vector3f *center) {
			std::cout << *p << std::endl;
			std::cout << *center << std::endl;
			Vector3f tmp = *p - *center;
			std::cout << tmp << std::endl;
			float dist = sqrtf(tmp(0)*tmp(0) + tmp(1)*tmp(1) + tmp(2)*tmp(2));
			return dist <= r;
		}
		// detect whether all points in sphere
		bool IsAllPointsInSphere(float r, Vector3f *center) {
			for (int i = 0; i < points.size(); i++) {
				if (!IsInSphere(points[i], r, center)) {
					return false;
				}
			}
			return true;
		}
		// detect whether there is any intersect sign that needs further process children
		bool IsExistFurtherInsterst(float r, Vector3f *center) {
			for (int i = 0; i < points.size(); i++) {
				if (IsInSphere(points[i], r, center)) {
					return true;
				}
			}
			return location != NULL && IsInSphere(location, r, center);
		}

		friend void PrintTree(const KDnode *n) {
			if (n->location == NULL) {
				return;
			}
			std::cout << *n->location << "\n";
			for (int i = 0; i < n->points.size(); i++) {
				std::cout << "Points: " << (n->points[i])->transpose() << " in current node \n";
			}

			if (n->right != NULL) {
				std::cout << "Depth: " << n->depth + 1 << " right tree: \n";
				std::cout << *(n->right->location) << "\n";

			}
			if (n->left != NULL) {
				std::cout << "Depth: " << n->depth + 1 << " left tree: \n";
				std::cout << *(n->left->location) << "\n";
			}
			return;
		}
	} KDnode;
public:
	KDnode *root;
	int k;
public:
	KDtree() : k(3) {
		root = new KDnode();
	}
	~KDtree() {
		delete root;
	}

	bool ConstructTree(std::vector<Vector3f*> &points) {
		if (points.empty()) { return false; }
		k = 3;	 // set by default
		BuildTree(root, points, 0);
		PrintTree(root);
		return true;
	}

	bool SearchKnearest(Vector3f *query, float r, std::vector<Vector3f*> &neighbors) {
		if (root == NULL) {
			return false;
		}
		return FindKnearest(root, query, r, neighbors);
	}

	// tree operations
private:
	void BuildTree(KDnode *node, std::vector<Vector3f*> &points, int depth) {
		node->depth = depth;
		if (points.empty()) {
			return;
		}
		int size = points.size();
		if (size == 1) {
			node->location = points[0];
			return;
		}

		int axis = depth % 3;
		Mergesort(axis, points, size);
		int mid = (size - 1)*0.5;
		node->location = points[mid];

		std::vector<Vector3f*> left, right;
		left.reserve(size*0.5); 
		right.reserve(size*0.5);
		left = Subset(points, 1, mid);
		right = Subset(points, 0, mid);

		for (int i = 0; i < left.size(); ++i)
			node->points.push_back(left[i]);
		for (int i = 0; i < right.size(); ++i)
			node->points.push_back(right[i]);

		node->left = new KDnode();
		node->right = new KDnode();
		BuildTree(node->left, left, depth + 1);
		BuildTree(node->right, right, depth + 1);
	}

	inline bool FindKnearest(KDnode *node, Vector3f *query, float r, std::vector<Vector3f*> &neighbors) {
		if (node->location == NULL) { return false; }
		if (node->IsInSphere(node->location, r, query)) {
			neighbors.push_back(node->location);
		}

		if (node->left != NULL) {
			if (node->left->IsAllPointsInSphere(r, query)) {
				for (int i = 0; i < node->left->points.size(); i++) {
					neighbors.push_back(node->left->points[i]);
				}
				if (node->left->location != NULL && node->left->IsInSphere(node->left->location, r, query)) {
					neighbors.push_back(node->left->location);
				}
			}
			else if (node->left->IsExistFurtherInsterst(r, query)) {
				FindKnearest(node->left, query, r, neighbors);
			}
		}

		if (node->right != NULL) {
			if (node->right->IsAllPointsInSphere(r, query)) {
				for (int i = 0; i < node->right->points.size(); i++) {
					neighbors.push_back(node->right->points[i]);
				}
				if (node->right->location != NULL && node->right->IsInSphere(node->right->location, r, query)) {
					neighbors.push_back(node->right->location);
				}
			}
			else if (node->right->IsExistFurtherInsterst(r, query)) {
				FindKnearest(node->right, query, r, neighbors);
			}
		}

		if (neighbors.empty()) {
			return false;
		}
		else {
			return true;
		}
	}

// utility algorithms
private:

	inline void Mergesort(int axis, std::vector<Vector3f*> &points, int size) {
		if (size <= 1) {
			return;
		}
		std::vector<Vector3f*> left = Subset(points, true, (int)(size/2));
		std::vector<Vector3f*> right = Subset(points, false, (int)(size/2 - 1));
		Mergesort(axis, left, left.size());
		Mergesort(axis, right, right.size());

		//merge
		int l = 0; int r = 0;
		for (int i = 0; i < size; ++i) {
			if (l < left.size() && (r == right.size() || left[l]->data()[axis] < right[r]->data()[axis] )) {
				points[i] = left[l];
				l++;
			}
			else {
				points[i] = right[r];
				r++;
			}
		}
	}

	inline std::vector<Vector3f*> Subset(std::vector<Vector3f*> &points, bool isFrontPart, int idx) {
		std::vector<Vector3f*>::const_iterator first;
		std::vector<Vector3f*>::const_iterator last;
		if (isFrontPart) {
			first = points.begin();
			last = points.begin() + idx;
		}
		else {
			first = points.begin() + idx + 1;
			last = points.end();
		}
		return std::vector<Vector3f*>(first, last);
	}
};

void Sample_KDtree(void) {
	KDtree *tree = new KDtree();
	int samples[8][3] = { { 3, 6, 5 },{ 17, 15, 4 },{ 13, 15, 28 },{ 6, 12, 12 },
	{ 9, 1, -4 },{ 2, 7, 10 },{ 10, 19, 0 }, {13, 2, 20} };
	std::vector<Vector3f> points(8);
	std::vector<Vector3f*> pointsPtr(8);
	for (int i = 0; i < 8; i++) {
		points[i] = Vector3f(samples[i][0], samples[i][1], samples[i][2]);
		pointsPtr[i] = &points[i];
	}
	

	tree->ConstructTree(pointsPtr);
	vector<Vector3f*> neighbors;
	Vector3f p0 = Vector3f(10, 19, 0);		// found
	(tree->SearchKnearest(&p0, 20.0f, neighbors)) ? cout << "Found\n" : cout << "Not Found\n";
	Vector3f p1 = Vector3f(3, 6, 5);		// found
	//(tree->SearchPoint(p1)) ? cout << "Found\n" : cout << "Not Found\n";
	Vector3f p2 = Vector3f(1, 19, 10);		// not found
	//(tree->SearchPoint(p2)) ? cout << "Found\n" : cout << "Not Found\n";
}
#endif //_KD_TREE_H