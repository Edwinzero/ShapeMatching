#pragma once
#ifndef _2D_CONTENT_H
#define _2D_CONTENT_H
#include <cv_op.h>
#include <opencv2\core\core.hpp>

typedef struct ImageTex {
	GLuint texID;
	int width, height;

	ImageTex():texID(-1), width(1), height(1){}
}ImageTex;

typedef struct ImageCanvas {

}ImageCanvas;

void InitColorImage(GLuint &tex, int &width, int &height, char* filepath) {
	cv::Mat pic = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);
	//cv::imshow("test", pic);
	//cv::waitKey(1);

	if (&pic == NULL) {
		return;
	}
	//cv::flip(pic, pic, 0);
	width = pic.cols;
	height = pic.rows;

	glEnable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, pic.cols, pic.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, pic.data);
	glGenerateMipmap(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
}

void InitDepthImage(GLuint &tex, int &width, int &height, char* filepath) {
	cv::Mat pic = cv::imread(filepath, CV_LOAD_IMAGE_ANYDEPTH);
	//cv::imshow("test", pic);
	//cv::waitKey(1);

	if (&pic == NULL) {
		return;
	}
	//cv::flip(pic, pic, 0);
	width = pic.cols;
	height = pic.rows;

	glEnable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, pic.cols, pic.rows, 0, GL_LUMINANCE, GL_UNSIGNED_SHORT, pic.data);
	glGenerateMipmap(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
}
void InitColorImage(ImageTex &img, char* filepath) {
	InitColorImage(img.texID, img.width, img.height, filepath);
}
void InitDepthImage(ImageTex &img, char* filepath) {
	InitDepthImage(img.texID, img.width, img.height, filepath);
}

// all the opencv related 2d draw write here...
void Draw2DContent(GLuint &tex, int width, int height) {

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, tex);

	glBegin(GL_QUADS);
		glTexCoord2f(0, 0);
		glVertex3f(0.0f, 0.0f, 0.5f);
		glTexCoord2f(0, 1);
		glVertex3f(0.0f, height, 0.5f);
		glTexCoord2f(1, 1);
		glVertex3f( width, height, 0.5f);
		glTexCoord2f(1, 0);
		glVertex3f( width, 0.0f, 0.5f);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

}


void Draw2DContent(GLuint &tex) {
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, tex);
	
	
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
}

#endif // !_2D_CONTENT_H
