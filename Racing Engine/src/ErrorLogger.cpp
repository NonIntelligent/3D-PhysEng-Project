#include "ErrorLogger.h"

#include <iostream>
#include "GL/glew.h"

void glClearErrors() {
	while(glGetError() != GL_NO_ERROR);
}

bool glCheckError(const char* function, const char* file, int line) {
	while(GLenum error = glGetError()) {
		std::cout << "OpenGL Error: [" << error << "]" << function <<
			" " << file << ": " << line << std::endl;
		return false;
	}
	return true;
}