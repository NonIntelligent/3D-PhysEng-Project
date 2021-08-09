#include "Texture.h"

#include <iostream>
#include "Vendor/stb_image.h"

Texture::Texture() 
: name("error") {
}

Texture::~Texture() {

}

void Texture::bind(unsigned int slot) {
	GLCall(glActiveTexture(GL_TEXTURE0 + slot));
	GLCall(glBindTexture(GL_TEXTURE_2D, textureID));
}

void Texture::unbind() const {
	GLCall(glActiveTexture(0));
	GLCall(glBindTexture(GL_TEXTURE_2D, 0));
}

bool Texture::loadTexture(const std::string & path, const std::string &glslVariableName) {
	// TODO check if a texture is already loaded with this texture object and clear the old one.
	if(active) {
		// Clear old texture.
	}


	this->m_Filepath = path;
	this->name = glslVariableName;

	stbi_set_flip_vertically_on_load(1);
	m_LocalBuffer = stbi_load(path.c_str(), &width, &height, &bytesPP, 4);

	GLCall(glGenTextures(1, &textureID));
	GLCall(glBindTexture(GL_TEXTURE_2D, textureID));

	// Linearlly choose and scale the texture pixel
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

	// Wrap horizontally and vertically by clamp
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT));

	GLCall(glGenerateMipmap(GL_TEXTURE_2D));

	if(m_LocalBuffer) {
		// Load texture data into GPU Vram.
		GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_LocalBuffer));
		active = true;
	}
	else {
		std::cout << "No data was loaded. Path: " + path << std::endl;
		return false;
	}

	GLCall(glBindTexture(GL_TEXTURE_2D, 0));
	// Free data from system memory.
	stbi_image_free(m_LocalBuffer);
	return true;
}

bool Texture::loadCubeMap(const std::string paths[], const std::string &glslVariableName) {
	// TODO check if a texture is already loaded with this texture object and clear the old one.
	if(active) {
		// Clear old texture.
	}

	this->m_Filepath = paths[0];
	this->name = glslVariableName;

	stbi_set_flip_vertically_on_load(1);

	GLCall(glGenTextures(1, &textureID));
	GLCall(glBindTexture(GL_TEXTURE_CUBE_MAP, textureID));

	for(unsigned int i = 0; i < 6; i++) {
		m_LocalBuffer = stbi_load(paths[i].c_str(), &width, &height, &bytesPP, 4);

		if(! m_LocalBuffer) {
			std::cout << "Cubemap failed to load path" << std::endl;
			stbi_image_free(m_LocalBuffer);
			return false;
		}

		GLCall(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_LocalBuffer));
		stbi_image_free(m_LocalBuffer);

	}

	// Linearlly choose and scale the texture pixel
	GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
	GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

	// Wrap horizontally and vertically by clamp
	GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
	GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE));

	//GLCall(glGenerateMipmap(GL_TEXTURE_CUBE_MAP));
	GLCall(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));

	return true;
}

void Texture::deleteTexture() {
	active = false;
	GLCall(glDeleteTextures(1, &textureID));
}
