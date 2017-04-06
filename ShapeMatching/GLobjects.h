#pragma once
#ifndef _GL_OBJECTS_H
#define _GL_OBJECTS_H
#include <vector>
#include <iostream>
#include <GL\glew.h>
#include <GL\freeglut.h>
#include <Eigen\Core>

typedef struct GLfbo {
	GLuint fbo_;
	GLuint tex_;
	GLuint dep_;

	GLfbo():fbo_(-1), tex_(-1), dep_(-1){}
} FBO;

bool CreateFBO(GLfbo &o, int width, int height) {
	glGenFramebuffers(0, &o.fbo_);
	glBindFramebuffer(GL_FRAMEBUFFER, o.fbo_);

	glGenTextures(1, &o.tex_);
	glBindTexture(GL_TEXTURE_2D, o.tex_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, o.tex_, 0);

	glGenRenderbuffers(1, &o.dep_);	// bind depth buffer
	glBindRenderbuffer(GL_RENDERBUFFER, o.dep_);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, o.dep_);

	// check FBO status
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		return false;
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	return true;
}

void CleanFBO(GLfbo &obj) {
	glDeleteRenderbuffers(1, &obj.dep_);
	glDeleteTextures(1, &obj.tex_);
	glDeleteFramebuffers(1, &obj.fbo_);
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

template <class T>
void CreateGLmem(GLmem &m, std::vector<T> vertices, std::vector<T> normals, std::vector<T> colors) {
	size_t numVertices = vertices.size();
	size_t numNormals = normals.size();
	size_t numColors = colors.size();
	if (sizeof(T) == 4) {
		numVertices /= 3;
		numNormals /= 3;
		numColors /= 4;
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

	if (numColors > 0) {
		glEnableVertexAttribArray(2);
		glGenBuffers(1, &m.ubo);
		glBindBuffer(GL_ARRAY_BUFFER, m.ubo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(int)*numColors * 3, colors.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(int), (GLvoid*)0); // bind vao to vbo
	}

	// Reset State
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

}

void CreateCanvas(GLmem &m) {
	GLfloat quadVertices[] = {   // Vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
								 // Positions   // TexCoords
		-1.0f,  1.0f,  0.0f, 1.0f,
		-1.0f, -1.0f,  0.0f, 0.0f,
		1.0f, -1.0f,  1.0f, 0.0f,

		-1.0f,  1.0f,  0.0f, 1.0f,
		1.0f, -1.0f,  1.0f, 0.0f,
		1.0f,  1.0f,  1.0f, 1.0f
	};
	glGenVertexArrays(1, &m.vao);
	glBindVertexArray(m.vao);
	glGenBuffers(1, &m.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 5, (GLvoid*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 5, (GLvoid*)(sizeof(float)*3));
	glBindVertexArray(0);
}
#endif // !_GL_OBJECTS_H
