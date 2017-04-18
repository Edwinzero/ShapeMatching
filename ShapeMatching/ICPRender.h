#pragma once
#ifndef __ICP_RENDER_H
#define __ICP_RENDER_H
#include <GL\glut.h>
#include <mswindow.h>

#include <camera.h>
#include <Sensor.h>
#include <LearnFusion.h>
#include <DatabufConvert.h>
#include "SimpleObjloader.h"
#include "ICP.h"
#include <glm/gtc/type_ptr.hpp>


//*
class ICPRender : public MSWindow {
public:
	Camera camera;

	// ICP
	std::vector<Eigen::Vector3f> data[2];
	std::vector<glm::vec3> renderbuf[2];
	std::vector<Eigen::Vector3f> normal[2];
	std::vector<Eigen::Vector2f> uvs[2];
	std::vector<unsigned short> indices[2];
	Eigen::Matrix4f data0TransMat;
	Eigen::Matrix4f data1Transformation;
	Eigen::Matrix3f rotation;

	// Control
	int P2PT_ICP, P2PL_ICP;
	float err;

public:
	ICPRender(){}
	//================================
	// init
	//================================
	void init(void) {
		// RIGID data
		LoadOBJfile("../Model/monkey.obj", data[0], normal[0], uvs[0], indices[0]);
		LoadOBJfile("../Model/monkey.obj", data[1], normal[1], uvs[1], indices[1]);
		Convert::ConstructMatEigenToEigen4(data1Transformation, euler_to_mat(Eigen::Vector3f(-15, 30, 20)), Eigen::Vector3f(2.0, 3.0, -1.0));
		Convert::CalGlobalCoordEigen(data1Transformation, data[1]);
		
		data0TransMat = Eigen::Matrix4f::Identity();

		// Control 
		P2PT_ICP = 0;
		P2PL_ICP = 0;

		// Test
		Eigen::Matrix3f test0 = Eigen::Matrix3f::Identity();
		test0(0, 1) = 20; test0(0, 2) = 30; test0(1, 2) = 50;
		glm::mat4 ass; Eigen::Matrix4f ass1l;
		//TestMatrix(ass, test0, Eigen::Vector3f(8, 6, 8));
		Convert::TestConstructMatrixEigen(ass1l, test0, Eigen::Vector3f(8, 6, 8));
	}

	//================================
	// update
	//================================
	void update(void) {

	}

	//================================
	// render
	//================================
	void render(void) {
		// clear buffer
		glClearColor(0.0, 0.0, 0.0, 0.0);
		glClearDepth(1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// matrix
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf(&camera.Mat()[0][0]);

		glEnable(GL_DEPTH_TEST);


		DrawCoord();
		//DrawMesh();
		//DrawBox();

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glScalef(100, 100, 100);

		float color_table[][3] = {
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 1 },
		{ 1, 1, 1 },
		};

		// ICP
		if(P2PT_ICP)
		{
			Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
			Eigen::Vector3f trans = Eigen::Vector3f::Zero();
			PointToPoint_ICP(data[0], data[1], rot, trans, err);

			// Transformate data
			Convert::ConstructMatEigenToEigen4(data0TransMat, rot, trans);
			Convert::CalGlobalCoordEigen(data0TransMat, data[0]);
			P2PT_ICP = 0;
			std::cout << "[Point to Point SVD] :: ICP error = " << err << std::endl;
			std::cout << "rotation: " << std::endl;
			std::cout << rot.transpose() << std::endl;
			std::cout << "translation: " << std::endl;
			std::cout << trans.transpose() << std::endl;
		}
		if (P2PL_ICP)
		{
			Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
			Eigen::Vector3f trans = Eigen::Vector3f::Zero();
			PointToPlane_ICP(data[0], data[1], normal[1], rot, trans, err);

			// Transformate data
			Convert::ConstructMatEigenToEigen4(data0TransMat, rot, trans);
			Convert::CalGlobalCoordEigen(data0TransMat, data[0]);
			P2PL_ICP = 0;
			std::cout << "[Point to Point SVD] :: ICP error = " << err << std::endl;
			std::cout << "rotation: " << std::endl;
			std::cout << rot.transpose() << std::endl;
			std::cout << "translation: " << std::endl;
			std::cout << trans.transpose() << std::endl;
		}

		// Draw source
		{
			glEnableClientState(GL_VERTEX_ARRAY);
			//glEnableClientState(GL_COLOR_ARRAY);
			//glColor3fv(color_table[0]);
			
			Convert::ConvertEigenToGlm3(renderbuf[0], data[0]);
			glVertexPointer(3, GL_FLOAT, sizeof(GLfloat)*3, renderbuf[0].data());
			//glColorPointer(3, GL_FLOAT, sizeof(GLfloat)*3, normal[0].data());
			glDrawArrays(GL_POINTS, 0, renderbuf[0].size());
			//glDisableClientState(GL_COLOR_ARRAY);
			glDisableClientState(GL_VERTEX_ARRAY);
		}
		// Draw target
		{
			glEnableClientState(GL_VERTEX_ARRAY);
			//glEnableClientState(GL_COLOR_ARRAY);
			glColor3fv(color_table[0]);

			Convert::ConvertEigenToGlm3(renderbuf[1], data[1]);
			glVertexPointer(3, GL_FLOAT, sizeof(GLfloat) * 3, renderbuf[1].data());
			//glColorPointer(3, GL_FLOAT, sizeof(GLfloat)*3, normal[1].data());
			glDrawArrays(GL_POINTS, 0, renderbuf[1].size());

			//glDisableClientState(GL_COLOR_ARRAY);
			glDisableClientState(GL_VERTEX_ARRAY);
		}

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}

	//================================
	// reshape
	//================================
	void reshape(int w, int h) {
		// viewport
		glViewport(0, 0, (GLsizei)w, (GLsizei)h);

		// projection matrix
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(45.0, (GLfloat)w / (GLfloat)h, 1.0f, 20000.0f);
	}

	//================================
	// mouse event
	//================================
	void mouseWheel(float dz) {
		camera.Zoom(dz * 50.0f);
	}

	void mouseDrag(float dx, float dy) {
		camera.Rot(-dy * 0.2f, -dx * 0.2f);
	}

	void middleMouseDrag(float dx, float dy) {
		camera.Move(-dx * 1.0f, dy * 1.0f);
	}

	virtual void keyDown(int key) {
		if (key == 'P') {
			P2PT_ICP = 1;
			return;
		}
		if (key == 'O') {
			P2PL_ICP = 1;
			return;
		}

		if (key >= '0' && key <= '9') {

			return;
		}
	}

	//================================
	// default drawing
	//================================
	void DrawCoord(void) {
		//glLineWidth(2.0);
		glBegin(GL_LINES);

		for (int z = -100; z <= 100; z++) {
			if (z == 0) {
				//x axis
				glColor3f(1, 0, 0);
			}
			else {
				glColor3f(0.2, 0.2, 0.3);
			}
			glVertex3f(-1000, 0, z * 10);
			glVertex3f(+1000, 0, z * 10);
		}

		for (int x = -100; x <= 100; x++) {
			if (x == 0) {
				//z axis
				glColor3f(0, 0, 1);
			}
			else {
				glColor3f(0.2, 0.2, 0.3);
			}
			glVertex3f(x * 10, 0, -1000);
			glVertex3f(x * 10, 0, 1000);
		}

		glColor3f(0, 1, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 500, 0);

		glEnd();
	}
/*
	void DrawBox(void) {
		glBegin(GL_LINES);
		glVertex3f(fusion.bound[0].x, fusion.bound[0].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[0].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[0].x, fusion.bound[1].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[1].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[0].x, fusion.bound[0].y, fusion.bound[1].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[0].y, fusion.bound[1].z);
		glVertex3f(fusion.bound[0].x, fusion.bound[1].y, fusion.bound[1].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[1].y, fusion.bound[1].z);

		glVertex3f(fusion.bound[0].x, fusion.bound[0].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[0].x, fusion.bound[0].y, fusion.bound[1].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[0].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[0].y, fusion.bound[1].z);
		glVertex3f(fusion.bound[0].x, fusion.bound[1].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[0].x, fusion.bound[1].y, fusion.bound[1].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[1].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[1].y, fusion.bound[1].z);

		glVertex3f(fusion.bound[0].x, fusion.bound[0].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[0].x, fusion.bound[1].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[0].x, fusion.bound[0].y, fusion.bound[1].z);
		glVertex3f(fusion.bound[0].x, fusion.bound[1].y, fusion.bound[1].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[0].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[1].y, fusion.bound[0].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[0].y, fusion.bound[1].z);
		glVertex3f(fusion.bound[1].x, fusion.bound[1].y, fusion.bound[1].z);
		glEnd();
	}
	//*/
};
#endif /* _RENDER_H */
//*/
/*
// BACK PROJ

Lfusion fusion;

cv::Mat intr;
cv::Mat extr;

std::vector<Sensor> sensors;
cv::Mat depth;
std::vector<cl::float4> points, normals;
fusion.bound[0] = cl::float4(-256, 0, -256, 0);
fusion.bound[1] = cl::float4(256, 512, 256, 0);
fusion.image_size = cl::int2(512, 424);
fusion.global_size2 = cl::size2(512, 512);
fusion.local_size2 = cl::size2(16, 16);
fusion.global_size3 = cl::size3(512, 512, 512);
fusion.local_size3 = cl::size3(8, 8, 8);
fusion.Create();
{
std::vector<char> kernelfile;
LoadText(kernelfile, "../Backproject.cl");
fusion.UpdateShader(kernelfile.data());
}


LoadFrame(depth, "../20169217252/K0/Pose_0.png");
cout << depth.cols << endl;


cv::imshow("test", depth);
cout << depth.at<unsigned short>(8, 208) << endl;
//cv::imwrite("test.png", depth);
cv::waitKey(1);

LoadOpenGLMat(intr, extr, "../CapturedData/warm/sensor.mat");

//fusion.BackProjectPoints(intr, extr, (unsigned short*)depth.ptr(), points, normals);
*/