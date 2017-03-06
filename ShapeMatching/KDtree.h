#pragma once
#ifndef _KDTREE_H
#define _KDTREE_H
#include <iostream>
#include <vector>
#include <Eigen_op.h>
using namespace std;
using namespace Eigen;

class KDnode {
public:
	int depth;
	std::vector<Vector3f*> candidates;
	Vector3f* location;			// if location == NULL => can be as terminate condition, KDnode == NULL as search or branching terminate condition
	KDnode *left, *right;

public:
	KDnode() : location(), right(), left(), depth(0) {

	}
	~KDnode() {
		delete left, right;
	}

	void ConstructTree(std::vector<Vector3f*> &points) {
		BuildTree(points, 0);
	}

	void SearchKNN(Vector3f* query, float r, std::vector<Vector3f*> &neighbors);
private:
	void BuildTree(std::vector<Vector3f*> &points, int depth);

// utility algorithms
private:

	inline void Mergesort(int axis, std::vector<Vector3f*> &points, int size);

	inline std::vector<Vector3f*> Subset(std::vector<Vector3f*> &points, bool isFrontPart, int idx);

	inline bool IsInSphere(Vector3f *p, float r, Vector3f *center);
	inline bool IsAllInSphere(float r, Vector3f *center);
	inline bool IsNeedFurtherSearch(float r, Vector3f *center);

public:
	friend void PrintNode(const KDnode *n);

	friend void PrintTree(const KDnode *node);
};


void KDnode::SearchKNN(Vector3f* query, float r, std::vector<Vector3f*> &neighbors) {
	if (location == NULL) {
		return;
	}

	if (IsInSphere(location, r, query)) {
		neighbors.push_back(location);
	}

	if (this->left != NULL) {
		if (this->left->IsAllInSphere(r, query)) {
			for (std::vector<Vector3f*>::iterator it = this->left->candidates.begin(); it < this->left->candidates.end(); it++) {
				neighbors.push_back(*it);
			}
			if (this->left->location != NULL && IsInSphere(this->left->location, r, query)) {
				neighbors.push_back(this->left->location);
			}
		}
		else if (this->left->IsNeedFurtherSearch(r, query)) {
			this->left->SearchKNN(query, r, neighbors);
		}
	}

	if (this->right != NULL) {
		if (this->right->IsAllInSphere(r, query)) {
			for (std::vector<Vector3f*>::iterator it = this->right->candidates.begin(); it < this->right->candidates.end(); it++) {
				neighbors.push_back(*it);
			}
			if (this->right->location != NULL && IsInSphere(this->right->location, r, query)) {
				neighbors.push_back(this->right->location);
			}
		}
		else if (this->right->IsNeedFurtherSearch(r, query)) {
			this->right->SearchKNN(query, r, neighbors);
		}
	}
}
void KDnode::BuildTree(std::vector<Vector3f*> &points, int depth) {
	this->depth = depth;
	if (points.empty()) {
		return;
	}
	int size = points.size();
	if (size == 1) {
		location = points[0];
		return;
	}

	int axis = depth % 3;
	Mergesort(axis, points, size);

	int mid = (size - 1)*0.5;
	location = points[mid];

	std::vector<Vector3f*> left, right;
	left = Subset(points, true, mid);
	right = Subset(points, false, mid);

	for (int i = 0; i < left.size(); i++) {
		candidates.push_back(left[i]);
	}
	for (int i = 0; i < right.size(); i++) {
		candidates.push_back(right[i]);
	}
	this->left = new KDnode();
	this->right = new KDnode();
	this->left->BuildTree(left, depth + 1);
	this->right->BuildTree(right, depth + 1);
}

// utility algorithms


inline void KDnode::Mergesort(int axis, std::vector<Vector3f*> &points, int size) {
	if (size <= 1) {
		return;
	}
	std::vector<Vector3f*> left = Subset(points, true, (int)(size / 2));
	std::vector<Vector3f*> right = Subset(points, false, (int)(size / 2 - 1));
	Mergesort(axis, left, left.size());
	Mergesort(axis, right, right.size());

	//merge
	int l = 0; int r = 0;
	for (int i = 0; i < size; ++i) {
		if (l < left.size() && (r == right.size() || left[l]->data()[axis] < right[r]->data()[axis])) {
			points[i] = left[l];
			l++;
		}
		else {
			points[i] = right[r];
			r++;
		}
	}
}

inline std::vector<Vector3f*> KDnode::Subset(std::vector<Vector3f*> &points, bool isFrontPart, int idx) {
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

inline bool KDnode::IsInSphere(Vector3f *p, float r, Vector3f *center) {
	Vector3f tmp = *p - *center;
	float dist = sqrtf(tmp(0)*tmp(0) + tmp(1)*tmp(1) + tmp(2)*tmp(2));
	return dist <= r;
}
inline bool KDnode::IsAllInSphere(float r, Vector3f *center) {
	for (int i = 0; i < this->candidates.size(); i++) {
		if (!IsInSphere(this->candidates[i], r, center)) {
			return false;
		}
	}
	return true;
}
inline bool KDnode::IsNeedFurtherSearch(float r, Vector3f *center) {
	for (int i = 0; i < this->candidates.size(); i++) {
		if (IsInSphere(this->candidates[i], r, center)) {
			return true;
		}
	}
	return location != NULL && IsInSphere(location, r, center);
}


inline void PrintNode(const KDnode *n) {
	if (n->location == NULL) {
		return;
	}
	std::cout << "== Node value: == \n";
	std::cout << n->location->transpose() << "\n";
	std::cout << "Depth: " << n->depth << "\n";
	//for (int i = 0; i < n->candidates.size(); i++) {
	//	std::cout << "Points: " << (n->candidates[i])->transpose() << " in current node \n";
	//}
	if (n->left != NULL) {
		std::cout << "Depth: " << n->depth + 1 << " LEFT tree: \n";
		std::cout << (n->left->location)->transpose() << "\n";
	}
	if (n->right != NULL) {
		std::cout << "Depth: " << n->depth + 1 << " RIGHT tree: \n";
		std::cout << (n->right->location)->transpose() << "\n";

	}
}

inline void PrintTree(const KDnode *node) {
	if (node->location == NULL) { return; }
	std::cout << "== Node value: == \n";
	std::cout << node->location->transpose() << "\n";
	std::cout << "Depth: " << node->depth << "\n\n";

	if (node->left != NULL) {
		std::cout << "Depth: " << node->depth + 1 << " LEFT tree: \n";
		PrintTree(node->left);
	}
	if (node->right != NULL) {
		std::cout << "Depth: " << node->depth + 1 << " RIGHT tree: \n";
		PrintTree(node->right);
	}
}

//*
void Sample_KDtree(void) {
	KDnode *tree = new KDnode();
	int samples[8][3] = { { 3, 6, 5 },{ 17, 15, 4 },{ 13, 15, 28 },{ 6, 12, 12 },
	{ 9, 1, -4 },{ 2, 7, 10 },{ 10, 19, 0 }, {13, 2, 20} };
	std::vector<Vector3f> points(8);
	std::vector<Vector3f*> pointsPtr(8);
	for (int i = 0; i < 8; i++) {
		points[i] = Vector3f(samples[i][0], samples[i][1], samples[i][2]);
		pointsPtr[i] = &points[i];
	}
	

	tree->ConstructTree(pointsPtr);
	PrintTree(tree);
	if (tree->right->right->right == NULL) {
		printf("adhfkldjsafkljasdkjfklajdskf\n");
	}
	vector<Vector3f*> neighbors;
	Vector3f p0 = Vector3f(10, 19, 0);		// found
	(tree->SearchKNN(&p0, 20.0f, neighbors));
	(!neighbors.empty()) ? cout << "Found\n" : cout << "Not Found\n";

	Vector3f p1 = Vector3f(3, 6, 5);		// found
	neighbors.clear();
	(tree->SearchKNN(&p1, 20.0f, neighbors));
	(!neighbors.empty()) ? cout << "Found\n" : cout << "Not Found\n";
	
	Vector3f p2 = Vector3f(1, 19, 10);		// not found(range is small)
	neighbors.clear();
	(tree->SearchKNN(&p2, 1.0f, neighbors));
	(!neighbors.empty()) ? cout << "Found\n" : cout << "Not Found\n";
}
//*/
#endif //_KD_TREE_H