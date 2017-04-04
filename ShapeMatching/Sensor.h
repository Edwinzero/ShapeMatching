#pragma once
#ifndef __SENSOR_H
#define __SENSOR_H
#include <string>
#include <vector>

using namespace std;

//================
// CameraIntrinsic
//================
class CameraIntrinsic {
public:
	cv::Mat		cameraMatrix; // 3x3 64F matrix
	cv::Mat		distCoeffs; // 8x1 64F vector
	cv::Mat		undistMap[2];

public:
	CameraIntrinsic() {
		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	}

	// 4x1 64F vector : focal length, optical center
	cv::Mat IntrVec(void) const {
		cv::Mat K = cv::Mat(4, 1, CV_64F);

		K.at<double>(0, 0) = cameraMatrix.at<double>(0, 0);
		K.at<double>(1, 0) = cameraMatrix.at<double>(1, 1);
		K.at<double>(2, 0) = cameraMatrix.at<double>(0, 2);
		K.at<double>(3, 0) = cameraMatrix.at<double>(1, 2);

		return K;
	}

	void InitUndistortMap(cv::Size imageSize) {
		cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);
		//cv::Mat newCameraMatrix = cameraMatrix;
		cv::initUndistortRectifyMap(
			cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix,
			imageSize, CV_16SC2, undistMap[0], undistMap[1]
		);
	}
};

//================
// RigidTransform
//================
class RigidTransform {
public:
	// 3x1 64F vector
	cv::Mat		rvec;
	cv::Mat		tvec;
	cv::Mat		evec;
	cv::Mat		fvec;

public:
	RigidTransform() {
		rvec = cv::Mat::zeros(3, 1, CV_64F);
		tvec = cv::Mat::zeros(3, 1, CV_64F);
	}

	// 4x4 64F matrix
	cv::Mat Mat(void) const {
		cv::Mat m = cv::Mat::eye(4, 4, CV_64F);

		// T
		m.at<double>(0, 3) = tvec.at<double>(0, 0);
		m.at<double>(1, 3) = tvec.at<double>(1, 0);
		m.at<double>(2, 3) = tvec.at<double>(2, 0);

		// R
		cv::Mat rmat;
		cv::Rodrigues(rvec, rmat);
		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 3; i++) {
				m.at<double>(i, j) = rmat.at<double>(i, j);
			}
		}

		return m;
	}
};

//================
// CameraParameter
//================
class CameraParameter {
public:
	CameraIntrinsic						intr;
	cv::Mat								extr; // convert to camera space

public:
	CameraParameter() {
		extr = cv::Mat::eye(4, 4, CV_64F);
	}

	void SaveCali(const char *path) {
		FILE *fp = fopen(path, "w");
		if (!fp) {
			return;
		}

		// camera matrix
		for (int i = 0; i < 9; i++) {
			fprintf(fp, "%f ", ((double*)intr.cameraMatrix.ptr())[i]);
		}
		fprintf(fp, "\n");

		// distortion
		for (int i = 0; i < 5; i++) {
			fprintf(fp, "%f ", ((double*)intr.distCoeffs.ptr())[i]);
		}
		fprintf(fp, "\n");

		// kinect to camera
		for (int i = 0; i < 16; i++) {
			fprintf(fp, "%f ", ((double*)extr.ptr())[i]);
		}
		fprintf(fp, "\n");

		fclose(fp);
		fp = NULL;
	}

	void LoadCali(const char *path) {
		FILE *fp = fopen(path, "r");
		if (!fp) {
			return;
		}

		// camera matrix
		for (int i = 0; i < 9; i++) {
			float v;
			fscanf(fp, "%f ", &v);
			((double*)intr.cameraMatrix.ptr())[i] = v;
		}

		// distortion
		for (int i = 0; i < 5; i++) {
			float v;
			fscanf(fp, "%f ", &v);
			((double*)intr.distCoeffs.ptr())[i] = v;
		}

		// kinect to camera
		for (int i = 0; i < 16; i++) {
			float v;
			fscanf(fp, "%f ", &v);
			((double*)extr.ptr())[i] = v;
		}

		fclose(fp);
		fp = NULL;

		intr.InitUndistortMap(cv::Size(512, 424));

#if 1
		cout << "Load Cali" << endl;
		cout << intr.cameraMatrix << endl;
		cout << intr.distCoeffs << endl;
		cout << extr << endl;
#endif
	}
};


//================
// Sensor
//================
class Sensor {
public:
	// global ID
	int			sensorID;
	std::string	serial;

	// nodeID
	int			serverID;
	int			deviceID; // local device

	// sensor exter
	CameraParameter cali_rgb;
	CameraParameter cali_ir;

	cv::Mat dep_to_rgb;
	cv::Mat dep_to_vicon;
			
	cv::Mat dep_to_gl;	
	cv::Mat rgb_to_gl;
	cv::Mat center_to_rgb;

	cv::Mat undistMap[2];
	// image stream
	bool dirt;
	std::vector<unsigned short> img_depth;
public:
	Sensor() : sensorID(-1), serverID(-1), deviceID(-1) {
		
		dep_to_rgb = cv::Mat::eye(4, 4, CV_64FC1);
		dep_to_vicon = cv::Mat::eye(4, 4, CV_64FC1);
		dep_to_gl = cv::Mat::eye(4, 4, CV_64FC1);
		rgb_to_gl = cv::Mat::eye(4, 4, CV_64FC1);
		center_to_rgb = cv::Mat::eye(4, 4, CV_64FC1);

		dirt = true;
		img_depth.resize(424 * 512);
	}

	void LoadSensorParameters(const char* DepthPath, const char* RGBPath) {
		cali_ir.LoadCali(DepthPath);
		cali_rgb.LoadCali(RGBPath);
	}
};


void LoadGlobalIRMatrix(std::vector<Sensor> &sensors, char * filepath) {
	FILE *fp = fopen(filepath, "rb");
	if (!fp) {
		return;
	}
	for (int i = 0; i < sensors.size(); i++) {
		cv::Mat gl_intr = cv::Mat::eye(4, 1, CV_64FC1);
		fread(gl_intr.ptr(), sizeof(double), 4, fp);
		//fread(sensors[i].dep_intr.ptr(), sensors[i].dep_intr.elemSize(), sensors[i].dep_intr.total(), fp);
		fread(sensors[i].dep_to_gl.ptr(), sensors[i].dep_to_gl.elemSize(), sensors[i].dep_to_gl.total(), fp);
	
		cout << "[DepToWorld_MAT] :: " << endl;
		//cout << gl_intr << endl;
		//cout << sensors[i].dep_intr << endl;
		cout << sensors[i].dep_to_gl << endl;
	}
	fclose(fp);
	fp = NULL;
}
void LoadSingleGlobalIRMatrix(Sensor &sensor, int gid, char * filepath) {
	FILE *fp = fopen(filepath, "rb");
	if (!fp) {
		return;
	}
	
	for (int i = 0; i <= gid; i++) {
		cv::Mat gl_intr = cv::Mat::eye(4, 1, CV_64FC1);
		fread(gl_intr.ptr(), sizeof(double), 4, fp);
		//fread(sensors[i].dep_intr.ptr(), sensors[i].dep_intr.elemSize(), sensors[i].dep_intr.total(), fp);
		fread(sensor.dep_to_gl.ptr(), sensor.dep_to_gl.elemSize(), sensor.dep_to_gl.total(), fp);
	}
	
	//cout << "[DepToWorld_MAT] :: " << endl;
	//cout << gl_intr << endl;
	//cout << sensors[i].dep_intr << endl;
	//cout << sensors[i].dep_to_gl << endl;
	
	fclose(fp);
	fp = NULL;
}
void LoadGlobalRGBMatrix(std::vector<Sensor> &sensors, char * filepath) {
	FILE *fp = fopen(filepath, "rb");
	if (!fp) {
		return;
	}
	for (int i = 0; i < sensors.size(); i++) {
		cv::Mat gl_intr = cv::Mat::eye(4, 1, CV_64FC1);
		fread(gl_intr.ptr(), sizeof(double), 4, fp);
		//fread((void*)sensors[i].cali_rgb.intr.cameraMatrix.ptr()[0], sizeof(double), 1, fp);
		//fread((void*)sensors[i].cali_rgb.intr.cameraMatrix.ptr()[4], sizeof(double), 1, fp);
		//fread((void*)sensors[i].cali_rgb.intr.cameraMatrix.ptr()[2], sizeof(double), 1, fp);
		//fread((void*)sensors[i].cali_rgb.intr.cameraMatrix.ptr()[5], sizeof(double), 1, fp);
		fread(sensors[i].rgb_to_gl.ptr(), sensors[i].rgb_to_gl.elemSize(), sensors[i].rgb_to_gl.total(), fp);
		cout << "[DepToWorld_MAT] :: " << endl;
		cout << gl_intr << endl;
		//cout << sensors[i].rgb_intr << endl;
		cout << sensors[i].rgb_to_gl << endl;
	}
	fclose(fp);
	fp = NULL;
}
#endif /* _SENSOR_H */
/*

void LoadIRCali(const char *path) {
FILE *fp = fopen(path, "r");
if (!fp) {
return;
}

// camera matrix
for (int i = 0; i < 9; i++) {
float v;
fscanf(fp, "%f ", &v);
if (i == 0) {
((double*)dep_intr.ptr())[0] = v;
}
if (i == 2) {
((double*)dep_intr.ptr())[2] = v;
}
if (i == 4) {
((double*)dep_intr.ptr())[1] = v;
}
if (i == 5) {
((double*)dep_intr.ptr())[3] = v;
}

}

// distortion
for (int i = 0; i < 5; i++) {
float v;
fscanf(fp, "%f ", &v);
((double*)dep_distCoeffs.ptr())[i] = v;
}

// kinect to camera
for (int i = 0; i < 16; i++) {
float v;
fscanf(fp, "%f ", &v);
((double*)dep_mat.ptr())[i] = v;
}

fclose(fp);
fp = NULL;

//cout << "Load Cali" << endl;
//cout << dep_intr << endl;
//cout << dep_distCoeffs << endl;
//cout << dep_mat << endl;
}

void LoadRGBCali(const char *path) {
FILE *fp = fopen(path, "r");
if (!fp) {
return;
}

// camera matrix
for (int i = 0; i < 9; i++) {
float v;
fscanf(fp, "%f ", &v);
if (i == 0) {
((double*)rgb_intr.ptr())[0] = v;
}
if (i == 2) {
((double*)rgb_intr.ptr())[2] = v;
}
if (i == 4) {
((double*)rgb_intr.ptr())[1] = v;
}
if (i == 5) {
((double*)rgb_intr.ptr())[3] = v;
}
}

// distortion
for (int i = 0; i < 5; i++) {
float v;
fscanf(fp, "%f ", &v);
((double*)rgb_distCoeffs.ptr())[i] = v;
}

// kinect to camera
for (int i = 0; i < 16; i++) {
float v;
fscanf(fp, "%f ", &v);
((double*)rgb_mat.ptr())[i] = v;
}

fclose(fp);
fp = NULL;

//cout << "Load Cali" << endl;
//cout << rgb_intr << endl;
//cout << rgb_distCoeffs << endl;
//cout << rgb_mat << endl;
}

void LoadCentertoRGBCali(const char *path) {
FILE *fp = fopen(path, "r");
if (!fp) {
return;
}

// camera matrix
for (int i = 0; i < 9; i++) {
float v;
fscanf(fp, "%f ", &v);
if (i == 0) {
((double*)rgb_intr.ptr())[0] = v;
}
if (i == 2) {
((double*)rgb_intr.ptr())[2] = v;
}
if (i == 4) {
((double*)rgb_intr.ptr())[1] = v;
}
if (i == 5) {
((double*)rgb_intr.ptr())[3] = v;
}
}

// distortion
for (int i = 0; i < 5; i++) {
float v;
fscanf(fp, "%f ", &v);
((double*)rgb_distCoeffs.ptr())[i] = v;
}

// kinect to camera
for (int i = 0; i < 16; i++) {
float v;
fscanf(fp, "%f ", &v);
((double*)center_to_rgb.ptr())[i] = v;
}

fclose(fp);
fp = NULL;

//cout << "Load Cali" << endl;
//cout << rgb_intr << endl;
//cout << rgb_distCoeffs << endl;
cout << center_to_rgb << endl;
}

void LoadIRtoRGBCali(const char *path) {
FILE *fp = fopen(path, "r");
if (!fp) {
return;
}

// camera matrix
for (int i = 0; i < 9; i++) {
float v;
fscanf(fp, "%f ", &v);
if (i == 0) {
((double*)dep_intr.ptr())[0] = v;
}
if (i == 2) {
((double*)dep_intr.ptr())[2] = v;
}
if (i == 4) {
((double*)dep_intr.ptr())[1] = v;
}
if (i == 5) {
((double*)dep_intr.ptr())[3] = v;
}
}

// distortion
for (int i = 0; i < 5; i++) {
float v;
fscanf(fp, "%f ", &v);
((double*)dep_distCoeffs.ptr())[i] = v;
}

// depth to rgb
for (int i = 0; i < 16; i++) {
float v;
fscanf(fp, "%f ", &v);
((double*)dep_to_rgb.ptr())[i] = v;
}

fclose(fp);
fp = NULL;

//cout << "Load Cali" << endl;
//cout << rgb_intr << endl;
//cout << rgb_distCoeffs << endl;
//cout << rgb_mat << endl;
}
*/