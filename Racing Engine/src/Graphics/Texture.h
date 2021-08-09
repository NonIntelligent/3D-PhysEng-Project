#pragma once

#include "GL/glew.h"
#include <string>

#include "ErrorLogger.h"

class Texture {
private:
	unsigned int textureID = 0;
	std::string m_Filepath;
	std::string name;
	unsigned char* m_LocalBuffer = nullptr;
	int width = 0, height = 0, bytesPP = 0;

	bool active = false;

public:
	Texture();
	~Texture();

	// Bind texture to a binding point.
	void bind(unsigned int slot = 0);
	void unbind() const;

	bool loadTexture(const std::string &path, const std::string &glslVariableName);
	bool loadCubeMap(const std::string paths[], const std::string &glslVariableName);

	void deleteTexture();

	inline int getWidth() const { return width; }
	inline int getHeight() const { return height; }
	inline std::string getName() const { return name; }
	inline unsigned int getTextureID() const { return textureID; }

};

