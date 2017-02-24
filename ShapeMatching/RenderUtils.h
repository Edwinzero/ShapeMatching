#pragma once
//#define GL_VERSION_4_1
#include <GL\glew.h>
#include <vector>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

#ifndef __RENDER_UTILS_H
#define __RENDER_UTILS_H
// Method to load the shader contents from a string
void LoadGLShaderFromFile(std::vector<char> &str, const char *path) {
	FILE *fp = fopen(path, "rb");
	if (!fp) {
		return;
	}

	fseek(fp, 0, SEEK_END);
	size_t size = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	str.resize(size + 1);
	fread(str.data(), 1, size, fp);
	str[size] = '\0';

	fclose(fp);
}

//-----------------------------------------------------------------------------
// Purpose: Compiles a GL shader program and returns the handle. Returns 0 if
//			the shader couldn't be compiled for some reason.
//-----------------------------------------------------------------------------
GLuint CompileGLShaderFromSource(const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader)
{
	GLuint unProgramID = glCreateProgram();

	GLuint nSceneVertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(nSceneVertexShader, 1, &pchVertexShader, NULL);
	glCompileShader(nSceneVertexShader);

	GLint vShaderCompiled = 0;
	glGetShaderiv(nSceneVertexShader, GL_INFO_LOG_LENGTH, &vShaderCompiled);
	if (vShaderCompiled > 1)
	{
		printf("%s - Unable to compile vertex shader %d!\n", pchShaderName, nSceneVertexShader);
		GLchar* log = new char[vShaderCompiled + 1];
		glGetShaderInfoLog(nSceneVertexShader, vShaderCompiled, 0, log);
		printf("Log:: \n%s\n", log);
		delete log;
		glDeleteProgram(unProgramID);
		glDeleteShader(nSceneVertexShader);
		return 0;
	}
	glAttachShader(unProgramID, nSceneVertexShader);
	glDeleteShader(nSceneVertexShader); // the program hangs onto this once it's attached

	GLuint  nSceneFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(nSceneFragmentShader, 1, &pchFragmentShader, NULL);
	glCompileShader(nSceneFragmentShader);

	GLint fShaderCompiled = 0;
	glGetShaderiv(nSceneFragmentShader, GL_INFO_LOG_LENGTH, &fShaderCompiled);
	if (fShaderCompiled > 1)
	{
		printf("%s - Unable to compile fragment shader %d!\n", pchShaderName, nSceneFragmentShader);
		GLchar* log = new char[fShaderCompiled + 1];
		glGetShaderInfoLog(nSceneFragmentShader, fShaderCompiled, 0, log);
		printf("Log:: \n%s\n", log);
		delete log;
		glDeleteProgram(unProgramID);
		glDeleteShader(nSceneFragmentShader);
		return 0;
	}
	glAttachShader(unProgramID, nSceneFragmentShader);
	glDeleteShader(nSceneFragmentShader); // the program hangs onto this once it's attached

	glLinkProgram(unProgramID);

	GLint programSuccess = GL_TRUE;
	glGetProgramiv(unProgramID, GL_LINK_STATUS, &programSuccess);
	if (programSuccess != GL_TRUE)
	{
		printf("%s - Error linking program %d!\n", pchShaderName, unProgramID);
		glDeleteProgram(unProgramID);
		return 0;
	}

	glUseProgram(unProgramID);
	glUseProgram(0);

	return unProgramID;
}

GLuint CompileGLShaderFromSource(const char *pchShaderName, const char *pchVertexShader, const char *pchGeometryShader, const char *pchFragmentShader)
{
	GLuint unProgramID = glCreateProgram();
	//  vertex
	GLuint nSceneVertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(nSceneVertexShader, 1, &pchVertexShader, NULL);
	glCompileShader(nSceneVertexShader);

	GLint vShaderCompiled = 0;
	glGetShaderiv(nSceneVertexShader, GL_INFO_LOG_LENGTH, &vShaderCompiled);
	if (vShaderCompiled > 1)
	{
		printf("%s - Unable to compile vertex shader %d!\n", pchShaderName, nSceneVertexShader);
		GLchar* log = new char[vShaderCompiled + 1];
		glGetShaderInfoLog(nSceneVertexShader, vShaderCompiled, 0, log);
		printf("Log:: \n%s\n", log);
		delete log;
		glDeleteProgram(unProgramID);
		glDeleteShader(nSceneVertexShader);
		return 0;
	}
	glAttachShader(unProgramID, nSceneVertexShader);
	glDeleteShader(nSceneVertexShader); // the program hangs onto this once it's attached

	//  geometry
	GLuint nSceneGeometryShader = glCreateShader(GL_GEOMETRY_SHADER);
	glShaderSource(nSceneGeometryShader, 1, &pchGeometryShader, NULL);
	glCompileShader(nSceneGeometryShader);

	GLint gShaderCompiled = 0;
	glGetShaderiv(nSceneGeometryShader, GL_INFO_LOG_LENGTH, &gShaderCompiled);
	if (gShaderCompiled > 1)
	{
		printf("%s - Unable to compile geometry shader %d!\n", pchShaderName, nSceneGeometryShader);
		GLchar* log = new char[gShaderCompiled + 1];
		glGetShaderInfoLog(nSceneGeometryShader, gShaderCompiled, 0, log);
		printf("Log:: \n%s\n", log);
		delete log;
		glDeleteProgram(unProgramID);
		glDeleteShader(nSceneGeometryShader);
		return 0;
	}
	glAttachShader(unProgramID, nSceneVertexShader);
	glDeleteShader(nSceneVertexShader); // the program hangs onto this once it's attached

	// fragment
	GLuint  nSceneFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(nSceneFragmentShader, 1, &pchFragmentShader, NULL);
	glCompileShader(nSceneFragmentShader);

	GLint fShaderCompiled = 0;
	glGetShaderiv(nSceneFragmentShader, GL_INFO_LOG_LENGTH, &fShaderCompiled);
	if (fShaderCompiled > 1)
	{
		printf("%s - Unable to compile fragment shader %d!\n", pchShaderName, nSceneFragmentShader);
		GLchar* log = new char[fShaderCompiled + 1];
		glGetShaderInfoLog(nSceneFragmentShader, fShaderCompiled, 0, log);
		printf("Log:: \n%s\n", log);
		delete log;
		glDeleteProgram(unProgramID);
		glDeleteShader(nSceneFragmentShader);
		return 0;
	}
	glAttachShader(unProgramID, nSceneFragmentShader);
	glDeleteShader(nSceneFragmentShader); // the program hangs onto this once it's attached

	glLinkProgram(unProgramID);

	GLint programSuccess = GL_TRUE;
	glGetProgramiv(unProgramID, GL_LINK_STATUS, &programSuccess);
	if (programSuccess != GL_TRUE)
	{
		printf("%s - Error linking program %d!\n", pchShaderName, unProgramID);
		glDeleteProgram(unProgramID);
		return 0;
	}

	glUseProgram(unProgramID);
	glUseProgram(0);

	return unProgramID;
}


GLuint CompileGLShader(const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader) {
	std::vector<char> vert, frag;
	LoadGLShaderFromFile(vert, pchVertexShader);
	LoadGLShaderFromFile(frag, pchFragmentShader);
	return CompileGLShaderFromSource(pchShaderName, vert.data(), frag.data());
}

// ======================
//		Text File I/O
// ======================
char *TextFileRead(char *fn) {


	FILE *fp;
	char *content = NULL;

	int count = 0;

	if (fn != NULL) {
		fp = fopen(fn, "rt");

		if (fp != NULL) {

			fseek(fp, 0, SEEK_END);
			count = ftell(fp);
			rewind(fp);

			if (count > 0) {
				content = (char *)malloc(sizeof(char) * (count + 1));
				count = fread(content, sizeof(char), count, fp);
				content[count] = '\0';
			}
			fclose(fp);
		}
	}
	return content;
}

int TextFileWrite(char *fn, char *s) {

	FILE *fp;
	int status = 0;

	if (fn != NULL) {
		fp = fopen(fn, "w");

		if (fp != NULL) {

			if (fwrite(s, sizeof(char), strlen(s), fp) == strlen(s))
				status = 1;
			fclose(fp);
		}
	}
	return(status);
}
#endif 