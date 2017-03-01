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
		int splitAxis;
		float tmin, tmax;
		Vector3f point;

	public:
		KDnode():splitAxis(-1), tmin(1.0f), tmax(-1.0f){
			left = right = NULL;
		}
	} KDnode;
public:
	KDnode *root;
public:
	KDtree(){
		root = new KDnode();
	}

	inline void ConstructTree(std::vector<Vector3f> &points) {

	}

private:
	// Node operations
	inline KDnode* NewNode(Vector3f p) {
		KDnode *res = new KDnode();
		res->point = p;
		return res;
	}

	// Tree operations
	inline void Insert(KDnode n) {

	}

	inline void Update(KDnode n) {

	}

	inline void Search(KDnode n) {

	}

	inline void Delete(KDnode n) {

	}
};
#endif //_KD_TREE_H