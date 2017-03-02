#pragma once
#ifndef _AABB_H
#define _AABB_H
#include <vector>
#include <iostream>
#include <Eigen_op.h>
using namespace std;
using namespace Eigen;

class AABB {
public:
	Vector3f min, max;
public:
	AABB(Vector3f &min, Vector3f &max) {
		this->max = max;
		this->min = min;
	}

	void Expand(const AABB &box) {
		if (box.min(0) < min(0)) min(0) = box.min(0);
		if (box.min(1) < min(1)) min(1) = box.min(1);
		if (box.min(2) < min(2)) min(2) = box.min(2);

		if (box.max(0) > max(0)) max(0) = box.max(0);
		if (box.max(1) > max(1)) max(1) = box.max(1);
		if (box.max(2) > max(2)) max(2) = box.max(2);
	}

	int GetDominantAxis() {
		Vector3f l = max - min;
		if (l(0) > l(1) && l(0) > l(2)) return 0;	// x
		if (l(1) > l(0) && l(1) > l(2)) return 1;	// y
		return 2;									// z
	}
};
#endif // !_AABB_H
