#pragma once
#ifndef _POINT_CLOUD_H
#define _POINT_CLOUD_H
#include <iostream>
#include <vector>
#include <Eigen_op.h>
#include <GLobjects.h>
#include <ModelLoader.h>
using namespace Eigen;

// only support pos and normal
class PointCloud{
public:
	std::vector<Vector4f> points;
	std::vector<Vector4f> normals;
	std::vector<int>	  correspondence;
	Matrix4f  model;
	std::string name;
	int hasPoint, hasNormal;
public:
	PointCloud(){
		points.clear();
		normals.clear();
		correspondence.clear();
		model = Matrix4f::Identity();
		name = "no name";
		hasPoint = 0;
		hasNormal = 0;
	}

	// load from ply
	PointCloud(const char* filepath, std::string modelname = 0) {
		Init(filepath, modelname);
	}

	void Init(const char* filepath, std::string modelname = 0) {
		if (!modelname.empty()) {
			name = modelname;
		}
		PLYModelLoader loader;
		loader.LoadModel(filepath);
		loader.CopyToBuffer(points, normals);
	}

	void ScalePointData(const float scale = 1.0f) {
		ScalePoints(this->points, scale);
	}

	void TransformPointData(const Eigen::Matrix4f &mat) {
		int size = points.size();
		Vector4f tmp(0, 0, 0, 0);
		for (int i = 0; i < size; i++) {
			points[i](3) = 1.0f;
			tmp = mat * points[i];
			points[i] = tmp;
		}
	}
};

void CreateGLmem(GLmem &m, PointCloud &pc) {
	glGenVertexArrays(1, &m.vao);
	glBindVertexArray(m.vao);
	/// position
	int point_size = pc.points.size();
	int normal_size = pc.normals.size();
	if (point_size) {
		pc.hasPoint = 1;
	}
	if (normal_size) {
		pc.hasNormal = 1;
	}
		
	m.m_numVerts = point_size;
	glEnableVertexAttribArray(ATTRIBUTE_LAYOUT_INDEX_POSITION);
	glGenBuffers(1, &m.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*point_size * 4, pc.points.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(ATTRIBUTE_LAYOUT_INDEX_POSITION, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid*)0); // bind vao to vbo

																												  /// normal																											 
	if (normal_size > 0) {
		glEnableVertexAttribArray(ATTRIBUTE_LAYOUT_INDEX_NORMAL);
		glGenBuffers(1, &m.nbo);
		glBindBuffer(GL_ARRAY_BUFFER, m.nbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*normal_size * 4, pc.normals.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(ATTRIBUTE_LAYOUT_INDEX_NORMAL, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid*)0); // bind vao to vbo
	}

	// Reset State
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}


#if 0
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

	Point& operator =(Point&);

	bool operator ==(Point&);

	bool operator !=(Point&);

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

bool Point::operator ==(Point& b) {
	Eigen::Vector3f o_coordinates = b.p();
	if (pos[0] == o_coordinates[0] && pos[1] == o_coordinates[1]
		&& pos[2] == o_coordinates[2]) {
		return true;
	}
	return false;
}

bool Point::operator !=(Point& b) {
	return !(*this == b);
}
#endif
#endif // !_POINT_H
