#include "Shader.h"
#include <iostream>
#include <fstream>
#include <sstream>

#include "ErrorLogger.h"

Shader::Shader() 
	: m_FilePath("error"), m_RendererID(0) {
}

Shader::Shader(const std::string & filepath)
	: m_FilePath(filepath), m_RendererID(0) {
	ShaderProgramSource source = parseShader(filepath);
	if(source.fragmentSource == "error") {
		std::cout << "Problem in loading file, skipping" << std::endl;
		return;
	}
	m_RendererID = createShader(source.vertexSource, source.fragmentSource);
}

Shader::~Shader() {
	
}

void Shader::bind() const {
	GLCall(glUseProgram(m_RendererID));
}

void Shader::unBind() const {
	GLCall(glUseProgram(0));
}

void Shader::deleteProgram() {
	GLCall(glDeleteProgram(m_RendererID));
}

GLuint Shader::getID() const {
	return m_RendererID;
}

ShaderProgramSource Shader::parseShader(const std::string& filePath) {
	std::ifstream stream(filePath);

	// Stream failed to open file
	if(!stream.is_open()) {
		return {"error", "error"};
	}

	std::string line;
	std::stringstream ss[2];
	ShaderType type = ShaderType::NONE;

	while(getline(stream, line)) {
		if(line.find("#shader") != std::string::npos) {
			if(line.find("vertex") != std::string::npos)
				type = ShaderType::VERTEX;
			else if(line.find("fragment") != std::string::npos)
				type = ShaderType::FRAGMENT;

		}
		else {
			ss[(int)type] << line << '\n';
		}
	}

	return {ss[0].str(), ss[1].str()};
}

GLuint Shader::compileShader(GLuint type, const std::string& sourceCode) {
	GLuint id = glCreateShader(type);
	const char* src = sourceCode.c_str();
	GLCall(glShaderSource(id, 1, &src, nullptr));
	GLCall(glCompileShader(id));


	// get compile status
	int result;
	glGetShaderiv(id, GL_COMPILE_STATUS, &result);

	// If compile failed the print out error
	if(result == GL_FALSE) {
		int length;
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
		char* message = (char*)alloca(length * sizeof(char));
		glGetShaderInfoLog(id, length, &length, message);
		std::cout << "Failed to compile " <<
			(type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader" << std::endl;
		std::cout << message << std::endl;
		glDeleteShader(id);
		return 0;
	}

	return id;
}

GLuint Shader::createShader(const std::string & vertexShader, const std::string & fragShader) {
	unsigned int program = glCreateProgram();
	GLuint vs = compileShader(GL_VERTEX_SHADER, vertexShader);
	GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragShader);

	GLCall(glAttachShader(program, vs));
	GLCall(glAttachShader(program, fs));
	GLCall(glLinkProgram(program));
	GLCall(glValidateProgram(program));

	glDeleteShader(vs);
	glDeleteShader(fs);

	return program;
}


void Shader::setUniform1i(const std::string & name, int value) {
	GLCall(glUniform1i(getUniformLocation(name), value));
}

void Shader::setUniform1f(const std::string & name, float value) {
	GLCall(glUniform1f(getUniformLocation(name), value));
}

void Shader::setUniform3f(const std::string & name, Vec3f floats){
	GLCall(glUniform3f(getUniformLocation(name), floats.x, floats.y, floats.z));
}

void Shader::setUniform4f(const std::string & name, Vec4f floats) {
	GLCall(glUniform4f(getUniformLocation(name), floats.x, floats.y, floats.z, floats.w));
}

void Shader::setUniformMatrix4fv(const std::string & name, GLsizei count, GLboolean transpose, Mat4f matrix) {
	float data[16];
	matrix.data(data);
	GLCall(glUniformMatrix4fv(getUniformLocation(name), count, transpose, data));
}

void Shader::setUniformBlock(const std::string &blockName, GLuint blockBindingPt) {
	GLuint uniformBlock;
	GLCall(uniformBlock = glGetUniformBlockIndex(m_RendererID, blockName.c_str()));
	GLCall(glUniformBlockBinding(m_RendererID, uniformBlock, blockBindingPt));
}


GLint Shader::getUniformLocation(const std::string & name) {
	// Caching disabled for now as multiple objects using the same shader will have different cache values.

	/*if(m_UniformLocationCache.find(name) != m_UniformLocationCache.end()) {

		return m_UniformLocationCache[name];
	}*/

	GLint location = -1;

	GLCall(location = glGetUniformLocation(m_RendererID, name.c_str()));
	if(location == -1 && missingUniformCounter++ < 10) {
		std::cout << "Warning: uniform doesn't exist or is not in use '" << name << "'" << " RendererID: " << m_RendererID << std::endl;
	}
	m_UniformLocationCache[name] = location;
	return location;
}
