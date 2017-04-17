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
	std::vector<int> colors;
	std::vector<int> indices;
	int totalVert;
	int totalFace;
	
public:
	PLYModelLoader():totalFace(0), totalVert(0){}

	void LoadModel(const char* filepath) {
		ifstream file;
		file.open(filepath, ios::binary);
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
			float v = 0.0f;
			for (int i = 0; i < totalVert; i++) {
				//*
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
				//*/
			}
		}
		if (hasColor && hasNormal) {
			float v = 0.0f;
			for (int i = 0; i < totalVert; i++) {
				//*
				getline(file, line);
				std::stringstream ss(line);
				float v;
				ss >> v;				points.push_back(v);
				cout << v << " ";
				ss >> v;				points.push_back(v);
				cout << v << " ";
				ss >> v;				points.push_back(v);
				cout << v << " ";

				ss >> v;				normals.push_back(v);
				cout << v << " ";
				ss >> v;				normals.push_back(v);
				cout << v << " ";
				ss >> v;				normals.push_back(v);
				cout << v << " " << "\n";		
				//*/
				// rgba
				ss >> v;				colors.push_back(v);
				cout << v << " ";
				ss >> v;				colors.push_back(v);
				cout << v << " ";
				ss >> v;				colors.push_back(v);
				cout << v << " ";
				ss >> v;				colors.push_back(v);
				cout << v << " " << "\n";		
				//*/
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

	void CopyToBuffer(std::vector<Eigen::Vector4f> &points, std::vector<Eigen::Vector4f> &normals) {
		points.clear(); normals.clear();
		points.resize(this->totalVert);
		normals.resize(this->totalVert);
		for (int i = 0; i < totalVert; i++) {
			Eigen::Vector4f p; p.setZero();
			p(0) = this->points[3 * i + 0];
			p(1) = this->points[3 * i + 1];
			p(2) = this->points[3 * i + 2];
			p(3) = 1.0f;
			points[i] = p;

			Eigen::Vector4f n; n.setZero();
			n(0) = this->normals[3 * i + 0];
			n(1) = this->normals[3 * i + 1];
			n(2) = this->normals[3 * i + 2];
			n(3) = 0.0f;
			normals[i] = n;
		}
		this->points.clear();
		this->normals.clear();
	}
};

#endif