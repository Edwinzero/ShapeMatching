﻿#pragma once
#ifndef _RENDER_PIPELINE_H
#define _RENDER_PIPELINE_H
#include <iostream>

// Registration
#include <FastGlobalRegistration.h>
#include <ICP.h>
#include <pcl\correspondence.h>


#include <GL\glew.h>
#include <GL\freeglut.h>
#include "imgui.h"
#include "imgui_impl_glut.h"
#include <camera.h>
#include <GLobjects.h>
#include <RenderUtils.h>

#include <2Dcontent.h>

#include <PointCloud.h>
#include <RGBDmapping.h>
#include <plyloader.h>
#include <RGBDmappingCPU.h>
#include <GaussianNoise.h>

// PCL (have to add pcl)
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/pcl_search.h>
#include <pcl_op.h>

// Feature
#include <SIFTmatching.h>
#include <GMSmatching.h>
#include <CorrespondenceFinding.h>

// Debug helper
#include <DebugHelper.h>


#define MOCA	0	// 1
#define BF		2	// 2
#define SHANG   3	// 3
#define PLY_REG 0	// 4
#define TEXTURE 0	// 5


using namespace std;
unsigned int screenWidth = 1280;
unsigned int screenHeight = 780;
unsigned int pre_screenWidth = 0; 
unsigned int pre_screenHeight = 0;
bool show_debug_window = true;
bool show_test_window = false;
bool show_cameraInfo_window = false;
bool show_color_img_window = false;
bool show_depth_img_window = false;
bool show_instruction_window = true;

Camera camera;
// Sensor + Data
std::vector<Sensor> sensors;			// moca
Sensor bfsensors[2];					// bf
Sensor kinect;							// shang(home)

// Registration
PointCloud pc0, pc1;
GLmem object0, object1;
//FastGlobalReg fgr;

// PointCloud Rendering
GLmem moca_model;
// RGBD mapping + backprojection
Mapping clprocess;

const float dataScale = 50.0f;
// moca
PointCloud Kpc0, Kpc1;
GLmem Kobject0, Kobject1;
cv::Mat depth0, depth1;
cv::Mat color0, color1;
// Shang
PointCloud Kpcs0, Kpcs1;
GLmem Kobjs0, Kobjs1;
cv::Mat deps0, deps1;
cv::Mat rgbs0, rgbs1;
std::vector < std::pair < cv::Point2f, cv::Point2f > > scorres0;
// bf
PointCloud Kpc_0, Kpc_1, Kpc_2, Kpc_3;
GLmem Kobject_0, Kobject_1, Kobject_2, Kobject_3;
cv::Mat depth_0, depth_1, depth_2, depth_3;
cv::Mat color_0, color_1, color_2, color_3;

// Feature detection
ImageTex colorTex;
ImageTex depthTex;
GLfbo imageCanvas;
GLmem canvas;

// src dst (register all other frame to initial pose Kpc_0, Kpc_1 here)
std::vector<std::pair<int, int>> corres0, corres1;
bool doFGR = false;				// key: R
bool doICP = false;				// key: E
bool doReset = false;			// key: T
bool doRegTest = false;         // key: F

int modelID = -1;
bool showK0 = true;				// Key: 9		2->0
bool showK1 = true;				// Key: 0		3->1
bool showDemo = true;			// Key: 8
// CL programs Key: V
bool reComplieKernel = false;
// GL programs Key: C
bool reComplieShader = false;
GLuint GLPointRenderProgram;
GLuint GLMocaPointRenderProgram;
GLuint GLGradientProgram;

vector<float> pose0_2t0 = { 0.947342f, 0.010327f, -0.320058f, 18.586237f,
-0.009819f, 0.999947f, 0.003200f, -0.223906f,
0.320074f, 0.000112f, 0.947393f, 4.657801f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };
vector<float> pose0_3t1 = { 0.946434f, 0.017699f, -0.322412f, 18.790529f,
-0.016917f, 0.999843f, 0.005228f, -0.206227f,
0.322454f, 0.000507f, 0.946585f, 4.098423f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };

vector<float> pose1_2t0 = { 0.981160f, 0.003028f, 0.193175f, -8.695971f,
-0.003047f, 0.999995f, -0.000202f, -0.004200f,
-0.193175f, -0.000391f, 0.981164f, 0.623203f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };
vector<float> pose1_3t1 = { 0.981019f, -0.011783f, 0.193551f, -7.953295f,
0.011818f, 0.999930f, 0.000972f, -0.079097f,
-0.193549f, 0.001333f, 0.981090f, 0.924206f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };

vector<float> pose2_2t0 = { 0.933808f, 0.013725f, -0.357512f, 16.788931f,
-0.013109f, 0.999906f, 0.004149f, -0.159492f,
0.357535f, 0.000813f, 0.933900f, 4.308386f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };
vector<float> pose2_3t1 = { 0.926306f, 0.007443f, -0.376698f, 18.173956f,
-0.008313f, 0.999965f, -0.000683f, 0.079684f,
0.376679f, 0.003764f, 0.926336f, 3.903884f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };

/*
vector<float> pose0_2t0 = { 0.947342f, 0.010327f, -0.320058f, 18.586237f,
-0.009819f, 0.999947f, 0.003200f, -0.223906f,
0.320074f, 0.000112f, 0.947393f, 4.657801f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };
vector<float> pose0_3t1 = { 0.946434f, 0.017699f, -0.322412f, 18.790529f,
-0.016917f, 0.999843f, 0.005228f, -0.206227f,
0.322454f, 0.000507f, 0.946585f, 4.098423f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };

vector<float> pose1_2t0 = { 0.981160f, 0.003028f, 0.193175f, -8.695971f,
-0.003047f, 0.999995f, -0.000202f, -0.004200f,
-0.193175f, -0.000391f, 0.981164f, 0.623203f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };
vector<float> pose1_3t1 = { 0.981019f, -0.011783f, 0.193551f, -7.953295f,
0.011818f, 0.999930f, 0.000972f, -0.079097f,
-0.193549f, 0.001333f, 0.981090f, 0.924206f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };

vector<float> pose2_2t0 = { 0.933808f, 0.013725f, -0.357512f, 16.788931f,
-0.013109f, 0.999906f, 0.004149f, -0.159492f,
0.357535f, 0.000813f, 0.933900f, 4.308386f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };
vector<float> pose2_3t1 = {	0.926306f, 0.007443f, -0.376698f, 18.173956f,
-0.008313f, 0.999965f, -0.000683f, 0.079684f,
0.376679f, 0.003764f, 0.926336f, 3.903884f,
0.000000f, 0.000000f, 0.000000f, 1.000000f };
*/

void OpenImageFileToBuffer(std::string &pose) {
	char filepath[64];
	sprintf(filepath, "Data/BF/%s/K0/CPose%d_0.png", pose.c_str(), 1);
	cv::Mat tmp0;
	color_0 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
	//tmp0 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
	//cv::undistort(tmp0, color_0, bfsensors[1].cali_rgb.intr.cameraMatrix, bfsensors[1].cali_rgb.intr.distCoeffs);

	sprintf(filepath, "Data/BF/%s/K1/CPose%d_0.png", pose.c_str(), 1);
	cv::Mat tmp1;
	color_1 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
	//tmp1 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
	//cv::undistort(tmp1, color_1, bfsensors[0].cali_rgb.intr.cameraMatrix, bfsensors[0].cali_rgb.intr.distCoeffs);

	sprintf(filepath, "Data/BF/%s/K0/CPose%d_0.png", pose.c_str(), 2);
	color_2 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
	//tmp0 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
	//cv::undistort(tmp0, color_2, bfsensors[1].cali_rgb.intr.cameraMatrix, bfsensors[1].cali_rgb.intr.distCoeffs);

	sprintf(filepath, "Data/BF/%s/K1/CPose%d_0.png", pose.c_str(), 2);
	color_3 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
	//tmp1 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
	//cv::undistort(tmp1, color_3, bfsensors[0].cali_rgb.intr.cameraMatrix, bfsensors[0].cali_rgb.intr.distCoeffs);

	sprintf(filepath, "Data/BF/%s/K0/Pose%d_0.png", pose.c_str(), 1);
	depth_0 = cv::imread(filepath, CV_LOAD_IMAGE_ANYDEPTH);

	sprintf(filepath, "Data/BF/%s/K1/Pose%d_0.png", pose.c_str(), 1);
	depth_1 = cv::imread(filepath, CV_LOAD_IMAGE_ANYDEPTH);

	sprintf(filepath, "Data/BF/%s/K0/Pose%d_0.png", pose.c_str(), 2);
	depth_2 = cv::imread(filepath, CV_LOAD_IMAGE_ANYDEPTH);

	sprintf(filepath, "Data/BF/%s/K1/Pose%d_0.png", pose.c_str(), 2);
	depth_3 = cv::imread(filepath, CV_LOAD_IMAGE_ANYDEPTH);
}

void SaveToPlyfile(const char* filename, std::vector<Eigen::Vector4f> &points, std::vector<Eigen::Vector4f> &normals) {
	PLYModel model;
	model.isMesh = 0;
	model.ifNormal = 0;
	model.ifColor = 0;
	if (points.empty()) {
		printf("[SAVE PLY] :: points buffer is empty... \n");
		return;
	}

	int pointsize = points.size();
	model.vertexCount = pointsize;
	model.positions.resize(pointsize);
	for (int i = 0; i < pointsize; i++) {
		float x = points[i](0);
		float y = points[i](1);
		float z = points[i](2);
		model.positions[i] = glm::vec3(x, y, z);
	}

	if (!normals.empty()) {
		model.ifNormal = 1;
		int normalsize = normals.size();
		model.normals.resize(normalsize);
		for (int i = 0; i < pointsize; i++) {
			float x = normals[i](0);
			float y = normals[i](1);
			float z = normals[i](2);
			model.normals[i] = glm::vec3(x, y, z);
		}
	}
	model.faceCount = 0;
	model.PLYWrite(filename, model.ifNormal, model.ifColor);
	model.FreeMemory();
}

//================================
// default draw event
//================================
void DrawCoord() {
	int num = 2 * 7;
	int grad = 50;

	glBegin(GL_LINES);

	for (int z = -num; z <= num; z++) {
		if (z == 0) {
			//x axis
			glColor3f(1, 0, 0);
		}
		else {
			glColor3f(0.2, 0.2, 0.3);
		}
		glVertex3f(-num * grad, 0, z * grad);
		glVertex3f(+num * grad, 0, z * grad);
	}

	for (int x = -num; x <= num; x++) {
		if (x == 0) {
			//z axis
			glColor3f(0, 0, 1);
		}
		else {
			glColor3f(0.2, 0.2, 0.3);
		}
		glVertex3f(x * grad, 0, -num * grad);
		glVertex3f(x * grad, 0, +num * grad);
	}

	glEnd();
}
//=========================================================================
//		Draw methods
//=========================================================================
void DrawScene2D() {
	{
		//glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
		glDisable(GL_DEPTH_TEST);
	//	glEnableClientState(GL_VERTEX_ARRAY);
	//	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	//	glEnableClientState(GL_COLOR_ARRAY);
		// Setup viewport, orthographic projection matrix
		int offsetx = screenWidth * 0.5;
		int offsety = screenHeight * 0.5;
		glViewport(offsetx, offsety, (GLsizei)screenWidth/2, (GLsizei)screenHeight/2);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0.0f, screenWidth,  0.0f, screenHeight, -1.0f, +1.0f);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		// Draw a triangle at 0.5 z
		//glBegin(GL_TRIANGLES);
		//glColor3f(1.0f, 0.0f, 0.0f);
		//glVertex3f(50.5, 50.5, 0.5);
		//glVertex3f(550.5, 50.5, 0.5);
		//glVertex3f(550.0, 150.5, 0.5);
		//glEnd();

	//	glDisableClientState(GL_COLOR_ARRAY);
	//	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	//	glDisableClientState(GL_VERTEX_ARRAY);
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		//glPopAttrib();	
	}
}

void DrawGLmem(GLuint &program, PointCloud &pc, GLmem &mem, Eigen::Matrix4f &rep, Eigen::Vector3f color = Eigen::Vector3f(1.0f, 0.0f, 0.0f)) {
	glEnable(GL_PROGRAM_POINT_SIZE);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	if (pc.hasPoint) {
		glUseProgram(program);
		Eigen::Matrix4f proj = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
		glGetFloatv(GL_PROJECTION_MATRIX, proj.data());
		glGetFloatv(GL_MODELVIEW_MATRIX, view.data());
		// render transformation (scale + rep)
		model = pc.model * rep;
		
		glUniformMatrix4fv(glGetUniformLocation(program, "proj"), 1, GL_FALSE, proj.data());
		glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, view.data());
		glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, model.data());
		glUniform3fv(glGetUniformLocation(program, "color"), 1, color.data());
		glBindVertexArray(mem.vao);
		glDrawArrays(GL_POINTS, 0, mem.m_numVerts);
		glBindVertexArray(0);

		glUseProgram(0);
	}
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void DrawScene3D() {
	{
		//glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
		glEnable(GL_DEPTH_TEST);
		// Setup viewport, orthographic projection matrix
		glViewport(0, 0, (GLsizei)screenWidth, (GLsizei)screenHeight);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluPerspective(45.0, (GLdouble)screenWidth / (GLdouble)screenHeight, 1.0, 10000.0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf(&camera.Mat()[0][0]);

		// default coord grids
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			glLineWidth(2.0);
			DrawCoord();
			glColor3f(0, 1, 0);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 500, 0);
			glEnd();
		}
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		// Point cloud data
		if (BF) {  // shang
			glEnable(GL_PROGRAM_POINT_SIZE);
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			Eigen::Matrix4f rep; // camera to world
			rep <<
					0.0f, 1.0f*dataScale, 0.0f, 0.0f,
					-1.0f*dataScale, 0.0f, 0.0f, 0.69f*dataScale,
					0.0f, 0.0f, 1.0f*dataScale, 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f;
			
			if (showK0) {
				if (Kpc_0.hasPoint) {
					DrawGLmem(GLPointRenderProgram, Kpc_0, Kobject_0, rep, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
				}
				if (Kpc_2.hasPoint) {
					DrawGLmem(GLPointRenderProgram, Kpc_2, Kobject_2, rep, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
				}
			}
			
			if (showK1) {
				if (Kpc_1.hasPoint) {
					DrawGLmem(GLPointRenderProgram, Kpc_1, Kobject_1, rep, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
				}
				if (Kpc_3.hasPoint) {
					DrawGLmem(GLPointRenderProgram, Kpc_3, Kobject_3, rep, Eigen::Vector3f(1.0f, 1.0f, 0.0f));
				}
			}			
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
		}
		if (SHANG) {
			glEnable(GL_PROGRAM_POINT_SIZE);
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			Eigen::Matrix4f rep; // camera to world
			rep <<
					1.0f*dataScale, 0.0F, 0.0f, 0.0f,
					0.0F, -1.0f*dataScale, 0.0f, 0.69f*dataScale,
					0.0f, 0.0f, 1.0f*dataScale, 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f;
			
			if (Kpcs0.hasPoint) {
				DrawGLmem(GLPointRenderProgram, Kpcs0, Kobjs0, rep, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
			}
			if (Kpcs1.hasPoint) {
				DrawGLmem(GLPointRenderProgram, Kpcs1, Kobjs1, rep, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
			}
			
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
		}
		if (MOCA) {  // moca
			glEnable(GL_PROGRAM_POINT_SIZE);
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			Eigen::Matrix4f rep; // camera to world
			rep <<
				0.0f, 1.0f*dataScale, 0.0f, 0.0f,
				-1.0f*dataScale, 0.0f, 0.0f,0.0f,
				0.0f, 0.0f, 1.0f*dataScale, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f;
			if (Kpc0.hasPoint) {
				DrawGLmem(GLPointRenderProgram, Kpc0, Kobject0, rep);
			}
			if (Kpc1.hasPoint) {
				DrawGLmem(GLPointRenderProgram, Kpc1, Kobject1, rep);
			}
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
		}
		if (PLY_REG && showDemo) { // ply registration
			glEnable(GL_PROGRAM_POINT_SIZE);
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			Eigen::Matrix4f rep = Eigen::Matrix4f::Identity(); // camera to world
			if (pc0.hasPoint) {
				DrawGLmem(GLPointRenderProgram, pc0, object0, rep);
			}
			if (pc1.hasPoint) {
				DrawGLmem(GLPointRenderProgram, pc1, object1, rep);
				//glEnableClientState(GL_VERTEX_ARRAY);
				//glEnableClientState(GL_COLOR_ARRAY);
				//glColor3f(0.0f, 1.0f, 0.0f);
				//glVertexPointer(3, GL_FLOAT, sizeof(GLfloat) * 3, points1.data());
				//glColorPointer(3, GL_FLOAT, sizeof(GLfloat) * 3, pc1.normals.data());
				//glDrawArrays(GL_POINTS, 0, points1.size());

				//glDisableClientState(GL_COLOR_ARRAY);
				//glDisableClientState(GL_VERTEX_ARRAY);
			}
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
		}
		
		if (0) {
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			if (moca_model.vao) {
				glUseProgram(GLMocaPointRenderProgram);
				Eigen::Matrix4f proj = Eigen::Matrix4f::Identity();
				Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
				Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
				model *= 1000.0f;
				glGetFloatv(GL_PROJECTION_MATRIX, proj.data());
				glGetFloatv(GL_MODELVIEW_MATRIX, view.data());
				glUniformMatrix4fv(glGetUniformLocation(GLMocaPointRenderProgram, "proj"), 1, GL_FALSE, proj.data());
				glUniformMatrix4fv(glGetUniformLocation(GLMocaPointRenderProgram, "view"), 1, GL_FALSE, view.data());
				glUniformMatrix4fv(glGetUniformLocation(GLMocaPointRenderProgram, "model"), 1, GL_FALSE, model.data());
				glUniform3fv(glGetUniformLocation(GLMocaPointRenderProgram, "color"), 1, Eigen::Vector3f(0.0f, 1.0f, 0.0f).data());
				glBindVertexArray(moca_model.vao);
				glDrawArrays(GL_POINTS, 0, moca_model.m_numVerts);
				glBindVertexArray(0);

				glUseProgram(0);
			}
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
			glDisable(GL_PROGRAM_POINT_SIZE);

			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
		}
	
		//glPopAttrib();
	}
}

void drawGUI()
{
	ImGui_ImplGLUT_NewFrame(screenWidth, screenHeight);

	// 1. Show a simple window
	// Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets appears in a window automatically called "Debug"
	if(show_debug_window)
	{	
		ImGui::SetWindowSize(ImVec2(200, 150), ImGuiSetCond_FirstUseEver);
		ImGui::SetWindowPos(ImVec2(0, screenHeight-250));
		static float f = 0.0f;
		ImGui::Text("Debug information");
		ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
		if (ImGui::Button("Test Window")) show_test_window ^= 1;
		if (ImGui::Button("Camera Info")) show_cameraInfo_window ^= 1;
		if (ImGui::Button("Color Image Window")) show_color_img_window ^= 1;
		if (ImGui::Button("Depth Image Window")) show_depth_img_window ^= 1;
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	}

	// 2. Show another simple window, this time using an explicit Begin/End pair
	if (show_cameraInfo_window)
	{
		ImGui::SetNextWindowSize(ImVec2(200, 150), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowPos(ImVec2(0, screenHeight-350), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("Camera Info", &show_cameraInfo_window);
		ImGui::Text("Camera information");
		glm::vec3 campos = camera.GetPos();
		ImGui::Text("Camera position: (%.3f, %.3f, %.3f)", campos.x, campos.y, campos.z);
		glm::vec3 camdir = camera.GetDir();
		ImGui::Text("Camera direction: (%.3f, %.3f, %.3f)", camdir.x, camdir.y, camdir.z);
		glm::vec3 camright = glm::cross(camdir, glm::vec3(0.0f, 1.0f, 0.0f));
		ImGui::Text("Camera right: (%.3f, %.3f, %.3f)", camright.x, camright.y, camright.z);
		ImGui::End();
	}

	// 3. Show the ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	//if (show_test_window)
	//{
	//	ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
	//	ImGui::ShowTestWindow(&show_test_window);
	//}

	if (show_instruction_window) {
		ImGui::SetNextWindowSize(ImVec2(200, 150), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowPos(ImVec2(0, screenHeight - 350), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("User Instruction", &show_color_img_window, ImGuiWindowFlags_AlwaysAutoResize);
		ImGui::Text("Key 9: turn on/off display upper camera data");
		ImGui::Text("Key 0: turn on/off display upper camera data");
		ImGui::Text("Key f: do registration on currently visiable data");
		ImGui::Text("Key t: revert transformation");
		ImGui::Text("Key 1: load board dataset");
		ImGui::Text("Key 2: load chair dataset");
		ImGui::Text("Key 3: load human dataset");
		ImGui::Text("Key Esc: exit program");
		ImGui::End();
	}
	/*
	if (show_color_img_window) {
		ImGui::SetNextWindowSize(ImVec2(colorTex.width, colorTex.height), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowPos(ImVec2(150, 20), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("Color Image Window", &show_color_img_window, ImGuiWindowFlags_AlwaysAutoResize);
		ImGui::Image((void*)colorTex.texID, ImVec2(colorTex.width, colorTex.height));
		ImGui::End();
	}
	if (show_depth_img_window) {
		ImGui::SetNextWindowSize(ImVec2(depthTex.width, depthTex.height), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowPos(ImVec2(150, 20), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("Depth Image Window", &show_depth_img_window, ImGuiWindowFlags_AlwaysAutoResize);
		ImGui::Image((void*)depthTex.texID, ImVec2(depthTex.width, depthTex.height));
		ImGui::End();
	}
	//*/
	ImGui::Render();
}

//=========================================================================
//		Render
//=========================================================================
void Render(void)
{
	// Get Back to the Modelview
	glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_DEPTH_TEST);

	if (1) {
		// draw 3D scene
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		DrawScene3D();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	} 
	if (0) {
		// draw 2D scene
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		DrawScene2D();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}

	if(1){
		// draw gui, seems unnecessary to push/pop matrix and attributes
		//glPushAttrib(GL_ALL_ATTRIB_BITS);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		drawGUI();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		//glPopAttrib();
	}

	glFlush();
	glutSwapBuffers();
	glutPostRedisplay();
}
//=========================================================================
//		Reshape
//=========================================================================
void Reshape(int w, int h)
{
	// update screen width and height for imgui new frames
	screenWidth = w;
	screenHeight = h;

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;
	float ratio = 1.0f* w / h;

	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);
	glutPostRedisplay();
}

//=========================================================================
//		Image Process methods
//=========================================================================
void CLImageProcess() {
	// rgbd mapping
	cv::Mat res0;
	cv::Mat res1, res2, res3;
	if (MOCA) {
		clprocess.DepthToRGBMapping(sensors[0].cali_ir.intr.IntrVec(), sensors[0].cali_ir.extr,
			sensors[0].cali_rgb.intr.IntrVec(), sensors[0].cali_rgb.extr,
			(unsigned short*)depth0.ptr(), color0, res0, Kpc0.points);
		ImgShow("CL RGBDmapping K0", res0, 512, 424);

		clprocess.DepthToRGBMapping(sensors[1].cali_ir.intr.IntrVec(), sensors[1].cali_ir.extr,
			sensors[1].cali_rgb.intr.IntrVec(), sensors[1].cali_rgb.extr,
			(unsigned short*)depth1.ptr(), color1, res1, Kpc1.points);
		ImgShow("CL RGBDmapping K1", res1, 512, 424);

		clprocess.BackProjectPoints(sensors[0].cali_ir.intr.IntrVec(), sensors[0].dep_to_gl, (unsigned short*)depth0.ptr(), Kpc0.points, Kpc0.normals);
		CreateGLmem(Kobject0, Kpc0);

		clprocess.BackProjectPoints(sensors[1].cali_ir.intr.IntrVec(), sensors[1].dep_to_gl, (unsigned short*)depth1.ptr(), Kpc1.points, Kpc1.normals);
		CreateGLmem(Kobject1, Kpc1);
	}
	if (BF) {// shang
			clprocess.DepthToRGBMapping(bfsensors[0].cali_ir.intr.IntrVec(), bfsensors[0].dep_to_rgb,
				bfsensors[0].cali_rgb.intr.IntrVec(),
				(unsigned short*)depth_0.ptr(), color_0, res0, Kpc_0.points);
			//ImgShow("CL RGBDmapping K0", res0, 512, 424);


			clprocess.DepthToRGBMapping(bfsensors[0].cali_ir.intr.IntrVec(), bfsensors[0].dep_to_rgb,
				bfsensors[0].cali_rgb.intr.IntrVec(), 
				(unsigned short*)depth_2.ptr(), color_2, res2, Kpc_2.points);
			//ImgShow("CL RGBDmapping K0", res2, 512, 424);

			clprocess.BackProjectPointsShang(bfsensors[0].cali_ir.intr.IntrVec(), bfsensors[0].cali_ir.extr, (unsigned short*)depth_0.ptr(), Kpc_0.points, Kpc_0.normals);
			CreateGLmem(Kobject_0, Kpc_0);

			//SaveToPlyfile("pose1_0.ply", Kpc_0.points, Kpc_0.normals);

			clprocess.BackProjectPointsShang(bfsensors[0].cali_ir.intr.IntrVec(), bfsensors[0].cali_ir.extr, (unsigned short*)depth_2.ptr(), Kpc_2.points, Kpc_2.normals);
			CreateGLmem(Kobject_2, Kpc_2);

			//SaveToPlyfile("pose1_2.ply", Kpc_2.points, Kpc_2.normals);
		
		
			clprocess.DepthToRGBMapping(bfsensors[1].cali_ir.intr.IntrVec(), bfsensors[1].dep_to_rgb,
				bfsensors[1].cali_rgb.intr.IntrVec(), 
				(unsigned short*)depth_1.ptr(), color_1, res1, Kpc_1.points);
			//ImgShow("CL RGBDmapping K1", res1, 512, 424);


			clprocess.DepthToRGBMapping(bfsensors[1].cali_ir.intr.IntrVec(), bfsensors[1].dep_to_rgb,
				bfsensors[1].cali_rgb.intr.IntrVec(), 
				(unsigned short*)depth_3.ptr(), color_3, res3, Kpc_3.points);
			//ImgShow("CL RGBDmapping K1", res3, 512, 424);



			clprocess.BackProjectPointsShang(bfsensors[1].cali_ir.intr.IntrVec(), bfsensors[1].cali_ir.extr, (unsigned short*)depth_3.ptr(), Kpc_3.points, Kpc_3.normals);
			//DHELPER::PointCloudStatisticalResult(Kpc_3.points);
			DHELPER::PointCloudValidNormal(Kpc_3.normals);
			CreateGLmem(Kobject_3, Kpc_3);

			//SaveToPlyfile("pose1_3.ply", Kpc_3.points, Kpc_3.normals);


			clprocess.BackProjectPointsShang(bfsensors[1].cali_ir.intr.IntrVec(), bfsensors[1].cali_ir.extr, (unsigned short*)depth_1.ptr(), Kpc_1.points, Kpc_1.normals);
			CreateGLmem(Kobject_1, Kpc_1);

			//SaveToPlyfile("pose1_1.ply", Kpc_1.points, Kpc_1.normals);
		
	}
	cv::Mat sres1, sres2;
	if (SHANG) {
		clprocess.DepthToRGBMapping(kinect.cali_ir.intr.IntrVec(), kinect.dep_to_rgb,
			kinect.cali_rgb.intr.IntrVec(),
			(unsigned short*)deps0.ptr(), rgbs0, sres1, Kpcs0.points);
		ImgShow("CL RGBDmapping SHANG pose t", sres1, 512, 424);


		clprocess.DepthToRGBMapping(kinect.cali_ir.intr.IntrVec(), kinect.dep_to_rgb,
			kinect.cali_rgb.intr.IntrVec(),
			(unsigned short*)deps1.ptr(), rgbs1, sres2, Kpcs1.points);
		ImgShow("CL RGBDmapping SHANG pose t+1", sres2, 512, 424);

		clprocess.BackProjectPointsShang(kinect.cali_ir.intr.IntrVec(), kinect.cali_ir.extr, (unsigned short*)deps0.ptr(), Kpcs0.points, Kpcs0.normals);
		CreateGLmem(Kobjs0, Kpcs0);

		//SaveToPlyfile("Spose1_0.ply", Kpcs0.points, Kpcs0.normals);

		clprocess.BackProjectPointsShang(kinect.cali_ir.intr.IntrVec(), kinect.cali_ir.extr, (unsigned short*)deps1.ptr(), Kpcs1.points, Kpcs1.normals);
		CreateGLmem(Kobjs1, Kpcs1);

		//SaveToPlyfile("Spose1_2.ply", Kpcs1.points, Kpcs1.normals);
	}

	// feature matching
	if(1){
		//std::vector<cv::Point2f> corres_src, corres_dst;
		//ExtractSIFTpointsFLANN(res0, res2, corres_src, corres_dst, 400);
		//ExtractSIFTpointsRANSACFLANN(res0, res2, corres_src, corres_dst, 400);
		//ExtractSIFTpointsFLANN(res1, res3, corres_src, corres_dst, 400);
		//ExtractSIFTpointsRANSACFLANN(res1, res3, corres_src, corres_dst, 400);
		//cv::waitKey(0);
		//GridMatch(res0, res2);
		//GridMatch(color_0, color_2);
		
			//GenCorrespondenceFromGridMatch(res2, res0, corres0);
			//printf("[res2 -> res0] :: << GMS filter >> coorespondence set size: %d\n", corres0.size());
	
			//GenCorrespondenceFromGridMatch(res3, res1, corres1);
			//printf("[res3 -> res1] :: << GMS filter >> coorespondence set size: %d\n", corres1.size());
		
			GenCorrespondenceFromGridMatch(sres2, sres1, scorres0, 2);
			printf("[sres2 -> sres1] :: << GMS filter >> coorespondence set size: %d\n", scorres0.size());
			cv::Mat show = VerifyDrawInlier(sres2, sres1, scorres0);
			ImgShow("verify matching result", show, 1024, 424);

			vector<Eigen::Vector4f> gmsPt2, gmsPt1, gmsN2, gmsN1;
			for (int i = 0; i < scorres0.size(); i++) {
				std::pair<cv::Point2f, cv::Point2f> tmp = scorres0[i];
				gmsPt2.push_back(Kpcs1.points[tmp.first.y * 512 + tmp.first.x]);
				gmsN2.push_back(Kpcs1.normals[tmp.first.y * 512 + tmp.first.x]);
				gmsPt1.push_back(Kpcs0.points[tmp.first.y * 512 + tmp.first.x]);
				gmsN1.push_back(Kpcs0.normals[tmp.first.y * 512 + tmp.first.x]);
			}

			// pcl
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features1(new pcl::PointCloud<pcl::FPFHSignature33>);
			PCLfpfhEstimation(pfh_features1, gmsPt1, gmsN1);
			std::cout << "output points.size (): " << pfh_features1->points.size() << std::endl;
			// Display and retrieve the shape context descriptor vector for the 0th point.
			pcl::FPFHSignature33 descriptor1 = pfh_features1->points[0];
			std::cout << descriptor1 << std::endl;

			pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features2(new pcl::PointCloud<pcl::FPFHSignature33>);
			PCLfpfhEstimation(pfh_features2, gmsPt2, gmsN2);
			std::cout << "output points.size (): " << pfh_features2->points.size() << std::endl;
			// Display and retrieve the shape context descriptor vector for the 0th point.
			pcl::FPFHSignature33 descriptor2 = pfh_features2->points[0];
			std::cout << descriptor2 << std::endl;

			int countfinal = 0;
			std::vector<std::pair<cv::Point2f, cv::Point2f>> filteredcandi;
			for (int i = 0; i < pfh_features1->size(); i++) {
				if (PCLcompareFPFHfeature(pfh_features1->at(i), pfh_features2->at(i))) {
					countfinal++;
					//printf("find same feature\n");
					filteredcandi.push_back(scorres0[i]);
				}
			}
			printf("exact same feature number is: %d\n", countfinal);

			// convert match to cv::keypoint and use verify inlier to display result
			cv::Mat checkshow = VerifyDrawInlier(sres2, sres1, filteredcandi);
			ImgShow("filtered matching result", checkshow, 1024, 424);
	}

	// correspondence finding
	{
		// projective data corres
		//CORRES::ProjectiveCorresondence(Kpc_2.points, Kpc_2.normals, Kpc_0.points, Kpc_0.normals, Kpc_2.model, Kpc_0.model, corres0, bfsensors[0]);
		//printf("[res2 -> res0] :: [Projective corres] :: set size is %d\n", corres0.size());
		//CORRES::ProjectiveCorresondence(Kpc_1.points, Kpc_1.normals, Kpc_1.points, Kpc_1.normals, Kpc_1.model, Kpc_1.model, corres1, bfsensors[1]);
		//printf("[res3 -> res1] :: [Projective corres] :: set size is %d\n", corres1.size());
	}
}
//=========================================================================
//		Update
//=========================================================================
// src(current data); dst(target data); transfer src to dst
void DoICP(PointCloud &src, PointCloud &dst, std::vector<std::pair<int, int>> &corres) {
	Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
	Eigen::Vector3f trans = Eigen::Vector3f::Zero();
	if (1) {
		if (corres.empty()) {
			float rme = PointToPoint_ICP(src.points, dst.points, rot, trans);
			std::cout << "[Point to Point SVD ICP] :: " << std::endl;
			std::cout << rot << std::endl;
			std::cout << trans << std::endl;
			std::cout << "rme : " << rme << std::endl;
			std::cout << std::endl;
		}
		else {
			float rme = PointToPoint_ICP(src.points, dst.points, corres, rot, trans);
			std::cout << "[Point to Point SVD ICP] :: " << std::endl;
			std::cout << rot << std::endl;
			std::cout << trans << std::endl;
			std::cout << "rme : " << rme << std::endl;
			std::cout << std::endl;
		}
		
	}
	else {

		// update pose
		std::cout << "[Point to Plane Iterative ICP] :: " << std::endl;
		for (int i = 0; i < 15; i++) {
			float rme = PointToPoint_iterICP(src.points, dst.points, rot, trans);

			if (i < 14) {
				std::cout << ">> Iter " << i << " ======" << std::endl;
				continue;
			}

			std::cout << ">> Iter " << i << " ======" << std::endl;
			std::cout << rot << std::endl;
			std::cout << trans << std::endl;
			std::cout << "rme : " << rme << std::endl;
			std::cout << std::endl;
			//std::cout << out_rot.determinant() << std::endl;
		}

	}
	Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
	ConstructMatEigenToEigen4(mat, rot, trans);
	src.model = mat * src.model;
}
void DoFGR(PointCloud &src, PointCloud &dst, std::vector<std::pair<int, int>> &corres) {
	FastGlobalReg fgr;
	fgr.LoadPoints(src.points, dst.points);
	fgr.LoadCorrespondence(corres);
	fgr.NormalizePoints();
	fgr.OptimizePairwise(false, 2, 0, 1);
	double rme = fgr.rme();
	std::cout << "[Fast Global Reg] " << std::endl;
	std::cout << "rme : " << rme << std::endl;
	std::cout << std::endl;
	Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
	mat = fgr.GetRes().inverse();
	src.model = mat * src.model;
}
void DoFGRTestCorrectness(PointCloud &src, PointCloud &dst) {
	FastGlobalReg fgr;
	fgr.LoadPoints(src.points, dst.points);
	fgr.LoadCorrespondence(src.points, 40);
	fgr.NormalizePoints();
	fgr.OptimizePairwise(false, 2, 0, 1);
	double rme = fgr.rme();
	std::cout << "[Fast Global Reg] " << std::endl;
	std::cout << "rme : " << rme << std::endl;
	std::cout << std::endl;
	Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
	mat = fgr.GetRes().inverse();
	src.model = mat * src.model;
}
void DoReset(PointCloud &pc) {
	Eigen::Matrix4f tmp = pc.model.inverse();
	pc.model = tmp; // eigen mat cant assign to itself
}
void DoReset(PointCloud &pc, Eigen::Matrix4f &rep) {
	pc.model = Eigen::Matrix4f::Identity();
	pc.MatrixTransform(rep);
	pc.MatrixScale(dataScale, dataScale, dataScale);
}

void Update(void) {
	if (doFGR) {
		DoFGR(Kpc_0, Kpc_2, corres0);
		DoFGR(Kpc_1, Kpc_3, corres1);
		DoFGRTestCorrectness(pc0, pc1);
		doFGR = false;
	}

	if (doICP) {
		printf("[P2P] : 2 -> 0: \n");
		DoICP(Kpc_2, Kpc_0, corres0);
		printf("[P2P] : 3 -> 1: \n");
		DoICP(Kpc_3, Kpc_1, corres1);
		std::vector<std::pair<int, int>> em;
		printf("[P2P] : ref \n");
		DoICP(pc1, pc0, em);
		doICP = false;
	}

	if (doRegTest) {
		if (modelID == 0) {
			if (showK0) {
				Kpc_2.model = createMat4(pose0_2t0) * Kpc_2.model;
				float rmse = rme(Kpc_2.points, Kpc_0.points, Kpc_2.model);
				printf("Data1 ==> [Model_2 to Model_0] :: Registration rmse = %f \n", rmse);
			}
			if (showK1) {
				Kpc_3.model = createMat4(pose0_3t1) * Kpc_3.model;
				float rmse = rme(Kpc_3.points, Kpc_1.points, Kpc_3.model);
				printf("Data1 ==> [Model_3 to Model_1] :: Registration rmse = %f \n", rmse);
			}
		}
		if (modelID == 1) {
			if (showK0) {
				Kpc_2.model = createMat4(pose1_2t0) * Kpc_2.model;
				float rmse = rme(Kpc_2.points, Kpc_0.points, Kpc_2.model);
				printf("Data2 ==> [Model_2 to Model_0] :: Registration rmse = %f \n", rmse);
			}
			if (showK1) {
				Kpc_3.model = createMat4(pose1_3t1) * Kpc_3.model;
				float rmse = rme(Kpc_3.points, Kpc_1.points, Kpc_3.model);
				printf("Data2 ==> [Model_3 to Model_1] :: Registration rmse = %f \n", rmse);
			}
		}
		if (modelID == 2) {
			if (showK0) {
				Kpc_2.model = createMat4(pose2_2t0) * Kpc_2.model;
				float rmse = rme(Kpc_2.points, Kpc_0.points, Kpc_2.model);
				printf("Data3 ==> [Model_2 to Model_0] :: Registration rmse = %f \n", rmse);
			}
			if (showK1) {
				Kpc_3.model = createMat4(pose2_3t1) * Kpc_3.model;
				float rmse = rme(Kpc_3.points, Kpc_1.points, Kpc_3.model);
				printf("Data3 ==> [Model_3 to Model_1] :: Registration rmse = %f \n", rmse);
			}
		}
		doRegTest = false;
	}

	if (doReset) {
		Eigen::Matrix4f rep;
		rep <<
			0.0f, 1.0f, 0.0f, 0.0f,
			-1.0f, 0.0f, 0.0f, 0.69f*dataScale,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f;
		DoReset(Kpc_2, rep);
		DoReset(Kpc_3, rep);
		DoReset(pc0);
		doReset = false;
	}

	if (reComplieShader) {
		GLPointRenderProgram = CompileGLShader("PointRender", "Shaders/PointRender.vs", "Shaders/PointRender.fs");
		GLMocaPointRenderProgram = CompileGLShader("MOCA_PointRender", "Shaders/PointCloudRender.vs", "Shaders/PointCloudRender.fs");
		reComplieShader = false;
		printf("[SHADER] :: Compile done !!!\n");
	}

	if (reComplieKernel) {
		{
			std::vector<char> kernelfile;
			LoadKernelText(kernelfile, "Kernels/RGBDmapping.cl");
			clprocess.UpdateShader(kernelfile.data());
		}
		CLImageProcess();
		reComplieKernel = false;
		printf("[KERNEL] :: Compile done !!!\n");
	}
}

//=========================================================================
//		keyboard & mouse callback
//=========================================================================
bool keyboardEvent(unsigned char nChar, int nX, int nY)
{
	ImGuiIO& io = ImGui::GetIO();
	io.AddInputCharacter(nChar);

	if (nChar == 27) { //Esc-key
		glutLeaveMainLoop();
	}
	if (nChar == '+') {
		camera.ChangeAperture(0.02f);
	}
	if (nChar == '-') {
		camera.ChangeAperture(-0.02f);
	}
	if (nChar == ']') {
		camera.ChangeFocalDistance(10.0f);
	}
	if (nChar == '[') {
		camera.ChangeFocalDistance(-10.0f);
	}
	if (nChar == 'D' || nChar == 'd') {
		show_debug_window = !show_debug_window;
	}
	if (nChar == 'I' || nChar == 'i') {
		show_instruction_window = !show_instruction_window;
	}
	if (nChar == 'r') {
		doFGR = !doFGR;
	}
	if (nChar == 'e') {
		doICP = !doICP;
	}
	if (nChar == 't') {
		doReset = !doReset;
	}
	if (nChar == 'f') {
		doRegTest = !doRegTest;
	}
	if (nChar == 'c') {
		reComplieShader = !reComplieShader;
	}
	if (nChar == 'v') {
		reComplieKernel = !reComplieKernel;
	}
	if (nChar == '1') {
		std::string model = "people0";
		OpenImageFileToBuffer(model);
		modelID = 0;
		CLImageProcess();
	}
	if (nChar == '2') {
		std::string model = "people1";
		OpenImageFileToBuffer(model);
		modelID = 1;
		CLImageProcess();
	}
	if (nChar == '3') {
		std::string model = "people2";
		OpenImageFileToBuffer(model);
		modelID = 2;
		CLImageProcess();
	}
	if (nChar == '9') {
		showK0 = !showK0;
	}
	if (nChar == '0') {
		showK1 = !showK1;
	}
	if (nChar == '8') {
		//showDemo = !showDemo;
	}

	return true;
}

void KeyboardSpecial(int key, int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.AddInputCharacter(key);
}

void keyboardCallback(unsigned char nChar, int x, int y)
{
	if (keyboardEvent(nChar, x, y))
	{
		glutPostRedisplay();
	}
}

bool mouseEvent(int button, int state, int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);

	if (state == GLUT_DOWN && (button == GLUT_LEFT_BUTTON))
		io.MouseDown[0] = true;
	else
		io.MouseDown[0] = false;

	if (state == GLUT_DOWN && (button == GLUT_RIGHT_BUTTON))
		io.MouseDown[1] = true;
	else
		io.MouseDown[1] = false;

	if (state == GLUT_DOWN && (button == GLUT_MIDDLE_BUTTON))
		io.MouseDown[2] = true;
	else
		io.MouseDown[2] = false;

	return true;
}

void MouseWheel(int button, int dir, int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);
	if (dir > 0)
	{
		// Zoom in
		io.MouseWheel = 1.0;
		camera.Zoom(dir * 2.0f);
	}
	else if (dir < 0)
	{
		// Zoom out
		io.MouseWheel = -1.0;
		camera.Zoom(dir * 2.0f);
	}
}

void MouseEventCallback(int button, int state, int x, int y)
{
	if (mouseEvent(button, state, x, y))
	{

	}
}

void MouseDragCallback(int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);
	float dy = (io.MousePos.y - io.MousePosPrev.y);
	float dx = (io.MousePos.x - io.MousePosPrev.x);
	if (io.MouseDown[0]) {
		camera.Rot(-dy * 0.20f, -dx * 0.20f);
	}
	if (io.MouseDown[2]) {
		camera.Move(-dx * 1.0f, dy * 1.0f);
	}
	glutPostRedisplay();
}

void MouseMoveCallback(int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);

	glutPostRedisplay();
}
//=========================================================================
//		Initialize platforms
//=========================================================================
void Init_GLshader(void) {
	GLPointRenderProgram = CompileGLShader("PointRender", "Shaders/PointRender.vs", "Shaders/PointRender.fs");
	GLGradientProgram = CompileGLShader("ImageGradient", "Shaders/2Dcanvas.vs", "Shaders/2Dcanvas.fs");
	GLMocaPointRenderProgram = CompileGLShader("MOCA_PointRender", "Shaders/PointCloudRender.vs", "Shaders/PointCloudRender.fs");
}
// initialize ogl and imgui
void Init_OpenGL(int argc, char **argv, const char* title)
{
	glutInit(&argc, argv);
	glutInitContextVersion(3, 0);
	//glutInitContextFlags(GLUT_FORWARD_COMPATIBLE); // cause error
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_MULTISAMPLE);
	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition(500, 300);
	glutCreateWindow(title);
	fprintf(stdout, "INFO: OpenGL Version: %s\n", glGetString(GL_VERSION));

	// glew
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		getchar();
		return;
	}

	// shaders
	Init_GLshader();

	// callback
	glutDisplayFunc(Render);
	glutReshapeFunc(Reshape);
	glutIdleFunc(Update);
	glutKeyboardFunc(keyboardCallback);
	glutSpecialFunc(KeyboardSpecial);
	glutMouseFunc(MouseEventCallback);
	glutMouseWheelFunc(MouseWheel);
	glutMotionFunc(MouseDragCallback);
	glutPassiveMotionFunc(MouseMoveCallback);
}

void Init_RenderScene(void) {

	// virtual camera
	{
		camera.focus = glm::vec3(0.0f, 0.0f, 0.0);
		camera.angle = glm::vec2(-30.0f, 0.0f);
		camera.fov = glm::vec2(45.0f, 45.0f);
		camera.apertureRadius = 0.01f;
		camera.focalDistance = 100.0f;
		camera.radius = 100.0f;
	}

	if(PLY_REG){
		pc0.Init("Depth_0000.ply", "child0");
		pc1.Init("Depth_0000.ply", "child1");
		pc0.MatrixScale(dataScale);
		pc1.MatrixScale(dataScale);
		AffineTransformPointsFromAngle(pc0.points, Eigen::Vector3f(20.0f, -50.0f, 70.0f), Eigen::Vector3f(-1.01f, +1.02f, -1.12f));

		CreateGLmem(object0, pc0);
		CreateGLmem(object1, pc1);
	}

	if (TEXTURE) {
		PLYModel m_model("Data/textured_model.ply", 1, 1);
		CreateGLmem(moca_model, m_model.positions, m_model.normals, m_model.colors);
	}

	// Load images
	if (0) {
		char filepath[64];
		//sprintf(filepath, "Data/K1/Pose_%d.jpeg", 666);
		//sprintf(filepath, "Data/BF/People0/K1/CPose%d_0.png", 1);
		sprintf(filepath, "Data/SHANG/Seq2/Pose_%d.jpeg", 823);
		cv::Mat testRGB = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
		//cv::flip(testRGB, testRGB, 1);
		ImgShow("rgb", testRGB, 960, 540);

		//sprintf(filepath, "Data/K1/Pose_%d.png", 666);
		//sprintf(filepath, "Data/BF/People0/K1/Pose%d_0.png", 1);
		sprintf(filepath, "Data/SHANG/Seq2/Pose_%d.png", 822);
		cv::Mat testDepth;
		//LoadFrame(testDepth, filepath);
		testDepth = cv::imread(filepath, CV_LOAD_IMAGE_ANYDEPTH);
		ImgShow("depth", testDepth, 512, 424);
	}

	// images for RGBD process
	if (MOCA) {
		char filepath[64];
		sprintf(filepath, "Data/K0/Pose_%d.jpeg", 666);
		color0 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
		cv::flip(color0, color0, 1);

		sprintf(filepath, "Data/K1/Pose_%d.jpeg", 666);
		color1 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
		cv::flip(color1, color1, 1);

		sprintf(filepath, "Data/K0/Pose_%d.png", 666);
		LoadFrame(depth0, filepath);

		sprintf(filepath, "Data/K1/Pose_%d.png", 666);
		LoadFrame(depth1, filepath);

		Kpc0.MatrixScale(dataScale, dataScale, dataScale);
		Kpc1.MatrixScale(dataScale, dataScale, dataScale);
	}
	if (BF) { // sHANG
		std::string model = "people2";
		modelID = 2;
		OpenImageFileToBuffer(model);
	}
	if (SHANG) { // home capture
		char filepath[64];
		//sprintf(filepath, "Data/SHANG/Seq2/Pose_%d.jpeg", 822);
		sprintf(filepath, "Data/SHANG/Seq1/Pose_%d.jpeg", 705);
		cv::Mat tmp0;
		tmp0 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
		cv::undistort(tmp0, rgbs0, bfsensors[0].cali_rgb.intr.cameraMatrix, bfsensors[0].cali_rgb.intr.distCoeffs);

		//sprintf(filepath, "Data/SHANG/Seq2/Pose_%d.jpeg", 836);
		sprintf(filepath, "Data/SHANG/Seq1/Pose_%d.jpeg", 697);
		cv::Mat tmp1;
		tmp1 = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
		cv::undistort(tmp1, rgbs1, bfsensors[1].cali_rgb.intr.cameraMatrix, bfsensors[1].cali_rgb.intr.distCoeffs);

		//sprintf(filepath, "Data/SHANG/Seq2/Pose_%d.png", 822);
		sprintf(filepath, "Data/SHANG/Seq1/Pose_%d.png", 705);
		deps0 = cv::imread(filepath, CV_LOAD_IMAGE_ANYDEPTH);

		//sprintf(filepath, "Data/SHANG/Seq2/Pose_%d.png", 836);
		sprintf(filepath, "Data/SHANG/Seq1/Pose_%d.png", 697);
		deps1 = cv::imread(filepath, CV_LOAD_IMAGE_ANYDEPTH);
	}
	// bilaterial filter for depth image
	if (0) {
		//cv::Mat res = FILTER::BilateralFilter(depth1, 9, 1.5,4.5);
		//ImgShow("bf depth1", depth1, 512, 424);
		//cv::imwrite("kjsdk1.png", depth1);
		//cv::imwrite("kjsdk.png", res);
	}
	// CPU test
	if (0) {
		//cv::Mat res;

		//RGBD::UndistortColorImage(color1, res, sensors[1]);
		//RGBD::UndistortDepthImage(depth1, res, sensors[1]);

		//RGBD::DepthToRGBMapping(sensors[1], color1, depth1, res);
		//ImgShow("rgbd mapping", res, res.cols, res.rows);
	}
}

void Init_Sensors(void) {
	// Sensor initialization
	if(MOCA){
		const int num_sensor = 2;
		// init sensors
		sensors.resize(num_sensor);
		// load RGB DEP in ex params
		char depPath[64], rgbPath[64];
		for (int i = 0; i < num_sensor; i++) {
			sprintf(depPath, "CamParams/KINECT%d_ir_cali.txt", i);
			sprintf(rgbPath, "CamParams/KINECT%d_rgb_cali.txt", i);
			sensors[i].LoadSensorParameters(depPath, rgbPath);
		}
		// load dep to global params
		LoadGlobalIRMatrix(sensors, "newSegData930/sensor-ir.mat");
		LoadGlobalRGBMatrix(sensors, "newSegData930/sensor-rgb.mat");
	}
	if (BF) {		
		// upper kinect (K1)
		bfsensors[0].LoadSensorParameters("Data/BF/IRIR_extr_cali_1.txt", "Data/BF/RGBRGB_extr_cali_1.txt");
		LoadIRtoRGBMatrix(bfsensors[0], "Data/BF/IRRGB_extr_caliIR_K1.txt");
		// lower kinect (B0)
		bfsensors[1].LoadSensorParameters("Data/BF/IRIR_extr_cali_0.txt", "Data/BF/RGBRGB_extr_cali_0.txt");
		LoadIRtoRGBMatrix(bfsensors[1], "Data/BF/IRRGB_extr_caliIR_B0.txt");
		//printf("BF [0]: \n");
		//std::cout << bfsensors[0].cali_ir.extr << std::endl;
		//printf("BF [1]: \n");
		//std::cout << bfsensors[1].cali_ir.extr << std::endl;
	}
	if (SHANG) {
		kinect.LoadSensorParameters("Data/SHANG/KINECT_ir_cali.txt", "Data/SHANG/KINECT_rgb_cali.txt");
		kinect.dep_to_rgb = kinect.cali_rgb.extr.inv() * kinect.cali_ir.extr;
		kinect.cali_ir.extr = cv::Mat::eye(4, 4, CV_64FC1);
		kinect.cali_rgb.extr = cv::Mat::eye(4, 4, CV_64FC1);
		printf("KINECT SHANG: \n");
		std::cout << kinect.cali_ir.extr << std::endl;
		std::cout << kinect.cali_rgb.extr << std::endl;
		std::cout << kinect.dep_to_rgb << std::endl;
	}
}


void Init_2DContents() {
	// for gui display, use cv process then bind gui to display
	//InitColorImage(colorTex, "rgb_color.jpeg");
	//InitDepthImage(depthTex, "BGdepth.png");
	//CreateFBO(imageCanvas, colorTex.width, colorTex.height);
}

void Init_OpenCL(void) {
	// init fusion
	clprocess.depth_img_size = cl::int2(512, 424);
	clprocess.color_img_size = cl::int2(1920, 1080);
	clprocess.bound[0] = cl::float3(-1.28, 0.00 - 0.5, -1.28);
	clprocess.bound[1] = cl::float3(+1.28, 2.56 - 0.5, +1.28);
	clprocess.global_size2 = 512;
	clprocess.local_size2 = 4;
	clprocess.global_size3 = 256;
	clprocess.local_size3 = 8;
	clprocess.Create();
	{
		std::vector<char> kernelfile;
		LoadKernelText(kernelfile, "Kernels/RGBDmapping.cl");
		clprocess.UpdateShader(kernelfile.data());
	}

}

void Init_Imgui(void) {
	//glClearColor(0.447f, 0.565f, 0.604f, 1.0f);
	//glClear(GL_COLOR_BUFFER_BIT);
	ImGui_ImplGLUT_Init();
}


//=========================================================================
//		Run Mainloop
//=========================================================================
void Run_Render(int argc, char **argv, const char* title){ 
	Init_OpenGL(argc, argv, title);

	Init_RenderScene();
	Init_2DContents();

	Init_Sensors();
	Init_OpenCL();
	CLImageProcess();

	Init_Imgui();

	glutMainLoop();
	ImGui_ImplGLUT_Shutdown();
}


#endif // !_RENDER_PIPELINE_H
