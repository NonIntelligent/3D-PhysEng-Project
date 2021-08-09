#pragma once
#include <vector>

#include "GL/glew.h"

#include "Maths/WorldMaths.h"
#include "ErrorLogger.h"

class VertexBuffer {

private:
	unsigned int m_RendererID = 0;
	unsigned int drawHint = 0;
	unsigned int m_Count = 0;

public:
	VertexBuffer() {};
	// Size in bytes
	VertexBuffer(const void* data, unsigned int size, unsigned int count, unsigned int drawHint);
	~VertexBuffer();

	void changeBufferData(const void* data, unsigned int size, unsigned int count);

	void bind() const;
	void unBind() const;

	// This method must be called before changing the buffer data to use the new hint.
	void setHint(unsigned int drawHint);

	inline unsigned int getCount() const {
		return m_Count;
	}
};

// Data contained in the buffer layout
struct VertexBufferElement {
	GLuint type;
	GLuint count;
	GLubyte normalised;

	static GLuint getSizeOfType(GLuint type) {
		switch(type) {
		case GL_FLOAT:			return 4;
		case GL_UNSIGNED_INT:	return 4;
		case GL_UNSIGNED_BYTE:	return 1;
		}
		ASSERT(false);
		return 0;
	}
};

// Easily setup the layout of the buffer for rendering
class VertexBufferLayout {
private:
	std::vector<VertexBufferElement> m_Elements;
	GLuint m_Stride;
public:
	VertexBufferLayout()
		: m_Stride(0) {
	};

	template<typename T>
	void push(unsigned int count) {
		static_assert(false);
	}

	template<>
	void push<float>(unsigned int count) {
		m_Elements.push_back({GL_FLOAT, count, GL_FALSE});
		m_Stride += count * VertexBufferElement::getSizeOfType(GL_FLOAT);
	}

	template<>
	void push<GLuint>(unsigned int count) {
		m_Elements.push_back({GL_UNSIGNED_INT, count, GL_FALSE});
		m_Stride += count * VertexBufferElement::getSizeOfType(GL_UNSIGNED_INT);
	}

	template<>
	void push<GLubyte>(unsigned int count) {
		m_Elements.push_back({GL_UNSIGNED_BYTE, count, GL_TRUE});
		m_Stride += count * VertexBufferElement::getSizeOfType(GL_UNSIGNED_BYTE);
	}

	inline const std::vector<VertexBufferElement>& getElements() const {
		return m_Elements;
	}
	inline unsigned int getStride() const {
		return m_Stride;
	}

};

class VertexArray {
private:
	GLuint m_RendererID;
public:
	VertexArray();
	~VertexArray();

	void addBuffer(const VertexBuffer& vb, const VertexBufferLayout& layout);

	void bind() const;
	void unBind() const;
};

// Allows vertices to be reused when drawing
class IndexBuffer {

private:
	unsigned int m_RendererID = 0;
	unsigned int m_Count = 0;
	unsigned int drawHint = 0;

public:
	IndexBuffer() {
	};
	IndexBuffer(const unsigned int * data, unsigned int count, unsigned int drawHint);
	~IndexBuffer();

	void bind() const;
	void unBind() const;

	inline unsigned int getCount() const {
		return m_Count;
	}
};

struct Vertex {
	Vec3f position;
	Vec3f normal;
	Vec3f texCoord;
};
