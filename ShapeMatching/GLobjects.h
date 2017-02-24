#pragma once
#ifndef _GL_OBJECTS_H
#define _GL_OBJECTS_H
#include <vector>
#include <iostream>
#include <GL\glew.h>
#include <GL\freeglut.h>
#include <Eigen\Core>

typedef struct FBO {
	GLuint fbo_;
	GLuint tex_;
	GLuint dep_;
} FBO;

void CreateFBO(FBO &o, int width, int height) {
	glGenFramebuffers(0, &o.fbo_);
	glBindFramebuffer(GL_FRAMEBUFFER, o.fbo_);

	glGenTextures(1, &o.tex_);
	glBindTexture(GL_TEXTURE_2D, o.tex_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, 0);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, o.tex_, 0);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}



#define ATTRIBUTE_LAYOUT_INDEX_POSITION 0
#define ATTRIBUTE_LAYOUT_INDEX_NORMAL   1
#define ATTRIBUTE_LAYOUT_INDEX_UV		2
class GLmem {
public:
	GLuint vao;
	GLuint vbo; // v
	GLuint nbo;	// n
	GLuint ubo;	// uv
	GLuint ebo;	// idx

	GLuint texID;
	GLuint depID;

	size_t m_numIndices;
	size_t m_numVerts;
public:
	GLmem() : vao(-1), vbo(-1), ebo(-1), texID(-1), depID(-1) {}
	~GLmem() {
		if (vao >= 0) glDeleteBuffers(1, &vao);
		if (vbo >= 0) glDeleteBuffers(1, &vbo);
		if (ebo >= 0) glDeleteBuffers(1, &ebo);
		if (texID >= 0) glDeleteBuffers(1, &texID);
		if (depID >= 0) glDeleteBuffers(1, &depID);
	}
};

void CreateGLmem(GLmem &m, std::vector<float> vertices, std::vector<float> normals, std::vector<float> uvs, std::vector<unsigned int> indices) {
	size_t numVertices = vertices.size() / 3;
	size_t numNormals = normals.size() / 3;
	size_t numUVs = uvs.size() / 2;
	size_t numIndices = indices.size();

	glGenVertexArrays(1, &m.vao);
	glBindVertexArray(m.vao);
	/// position
	m.m_numVerts = numVertices;
	glEnableVertexAttribArray(ATTRIBUTE_LAYOUT_INDEX_POSITION);
	glGenBuffers(1, &m.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*numVertices * 3, vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(ATTRIBUTE_LAYOUT_INDEX_POSITION, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (GLvoid*)0); // bind vao to vbo
	/// normal																											 
	if (numNormals > 0) {
		glEnableVertexAttribArray(ATTRIBUTE_LAYOUT_INDEX_NORMAL);
		glGenBuffers(1, &m.nbo);
		glBindBuffer(GL_ARRAY_BUFFER, m.nbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*numNormals * 3, normals.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(ATTRIBUTE_LAYOUT_INDEX_NORMAL, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (GLvoid*)0); // bind vao to vbo
	}
	/// uvs
	if (numUVs > 0) {
		glEnableVertexAttribArray(ATTRIBUTE_LAYOUT_INDEX_UV);
		glGenBuffers(1, &m.ubo);
		glBindBuffer(GL_ARRAY_BUFFER, m.ubo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*numUVs * 2, uvs.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(ATTRIBUTE_LAYOUT_INDEX_UV, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (GLvoid*)0); // bind vao to vbo
	}
	/// idx
	if (numIndices > 0){
		// IBO
		glGenBuffers(1, &m.ebo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*numIndices, indices.data(), GL_STATIC_DRAW);
		m.m_numIndices = numIndices;
	}
	
	// Reset State
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

template <class T>
void CreateGLmem(GLmem &m, std::vector<T> vertices, std::vector<T> normals) {
	size_t numVertices = vertices.size();
	size_t numNormals = normals.size();
	if (sizeof(T) == 4) {
		numVertices /= 3;
		numNormals /= 3;
	}

	glGenVertexArrays(1, &m.vao);
	glBindVertexArray(m.vao);
	/// position
	m.m_numVerts = numVertices;
	glEnableVertexAttribArray(ATTRIBUTE_LAYOUT_INDEX_POSITION);
	glGenBuffers(1, &m.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*numVertices * 3, vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(ATTRIBUTE_LAYOUT_INDEX_POSITION, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (GLvoid*)0); // bind vao to vbo

	/// normal																											 
	if (numNormals > 0) {
		glEnableVertexAttribArray(ATTRIBUTE_LAYOUT_INDEX_NORMAL);
		glGenBuffers(1, &m.nbo);
		glBindBuffer(GL_ARRAY_BUFFER, m.nbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*numNormals * 3, normals.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(ATTRIBUTE_LAYOUT_INDEX_NORMAL, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (GLvoid*)0); // bind vao to vbo
	}

	// Reset State
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

}

#endif // !_GL_OBJECTS_H