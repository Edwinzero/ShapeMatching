#pragma once
#ifndef _POINT_H
#define _POINT_H
#include <iostream>
#include <vector>
#include <Eigen_op.h>
using namespace Eigen;
class Point {
	Vector3f pos;
	Vector3f normal;
public:
	Point() {}

	Point(Vector3f &p, Vector3f &n) {
		this->pos = p;
		this->normal = n;
	}

	Vector3f p() {
		return pos;
	}

	Vector3f n() {
		return normal;
	}

	Point& operator =( Point&);

	bool operator ==( Point&);

	bool operator !=( Point&);

	friend std::ostream& operator<<(std::ostream& out, const Point& p) {
		out.width(4);
		out.precision(3);
		out << p.p()[0] << ", " << p.p()[1] << ", "
			<< p.p()[2];
		return out;
	}
};

Point& Point::operator =(Point& src) {
	pos = src.p();
	normal = src.n();
	return *this;
}

bool Point::operator ==( Point& b) {
	Eigen::Vector3f o_coordinates = b.p();
	if (pos[0] == o_coordinates[0] && pos[1] == o_coordinates[1]
		&& pos[2] == o_coordinates[2]) {
		return true;
	}
	return false;
}

bool Point::operator !=( Point& b) {
	return !(*this == b);
}

#endif // !_POINT_H
