#pragma once
#ifndef _MODEL_LOADER_H
#define _MODEL_LOADER_H
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <Eigen\Core>
#include <Eigen/Geometry>
#include <Eigen\Dense>
using namespace std;

class PLYModelLoader {
public:
	std::vector<float> points;
	std::vector<float> normals;
	std::vector<int> indices;
	int totalVert;
	int totalFace;
	
public:
	PLYModelLoader():totalFace(0), totalVert(0){}

	void LoadModel(const char* filepath) {
		ifstream file;
		file.open(filepath, ios::in | ios::binary);
		if (!file.is_open()) {
			cerr << "Couldn't open ply file < " << filepath << "> ! \n";
			return;
		}
		std::string line;
		getline(file, line);
		while (line.compare("vertex") != 0) {
			getline(file, line, ' ');
			cout << line << "\n";
		}
		file >> totalVert; // get total vert num
		cout << "model total vert: " << totalVert << "\n";

		// check if it is mesh
		bool hasNormal = false; bool hasColor = false;
		bool isEndHeader = false; bool isMesh = false;
		while (line.compare("face") != 0 && !isEndHeader) {
			getline(file, line);
			cout << line << endl;
			if (line.find("red") != std::string::npos) { hasColor = true; }
			if (line.find("nx") != std::string::npos) { hasNormal = true; }
			if (line.find("end_header") != std::string::npos) { isEndHeader = true; }
		}
		if (!isEndHeader) {
			file >> totalFace; // get total face num
			cout << "total face: " << totalFace << "\n";
			if (totalFace) isMesh = true;
			// read other information until end_header
			while (line.compare("end_header") != 0) {
				getline(file, line);
				cout << line << endl;
			}
		}

		points.reserve(totalVert * 3);
		normals.reserve(totalVert * 3);
		// Read p, n
		if (!hasColor && hasNormal) {
			for (int i = 0; i < totalVert; i++) {
				getline(file, line);
				std::stringstream ss(line);
				float v;
				ss >> v;				points.push_back(v);
				//cout << v << " ";
				ss >> v;				points.push_back(v);
				//cout << v << " ";
				ss >> v;				points.push_back(v);
				//cout << v << " ";

				ss >> v;				normals.push_back(v);
				//cout << v << " ";
				ss >> v;				normals.push_back(v);
				//cout << v << " ";
				ss >> v;				normals.push_back(v);
				//cout << v << " " << "\n";			
			}
		}
		// TODO
		if (isMesh) {

		}

		file.close();
	}

	void LoadModelToGLobject(const char* filepath) {

	}

	void LoadModelToCLmem(const char* filepath) {

	}

	void CopyToBuffer(std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &normals) {
		points.clear(); normals.clear();
		points.resize(this->totalVert);
		normals.resize(this->totalVert);
		for (int i = 0; i < totalVert; i++) {
			Eigen::Vector3f p; p.setZero();
			p(0) = this->points[3 * i + 0];
			p(1) = this->points[3 * i + 1];
			p(2) = this->points[3 * i + 2];
			points[i] = p;

			Eigen::Vector3f n; n.setZero();
			n(0) = this->normals[3 * i + 0];
			n(1) = this->normals[3 * i + 1];
			n(2) = this->normals[3 * i + 2];
			normals[i] = n;
		}
		this->points.clear();
		this->normals.clear();
	}
};

void TransformPoints(std::vector<Eigen::Vector3f> &points, float rx = 0, float ry = 0, float rz = 0, float tx = 0, float ty = 0, float tz = 0) {
	Eigen::Affine3f mat;
	mat.linear() = (Eigen::Matrix3d)Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
	mat.translation() = Eigen::Vector3d(tx, ty, tz);
	Eigen::Matrix4f transMat = mat.matrix().cast<float>();

	for (int i = 0; i < points.size(); i++){
		points[i] = transMat * 
	}
}
#endif