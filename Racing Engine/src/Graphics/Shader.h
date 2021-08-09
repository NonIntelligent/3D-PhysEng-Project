#pragma once
#include <string>
#include <unordered_map>

#include "GL/glew.h"
#include "Maths/WorldMaths.h"

enum class UniformType {
	NONE = -99,
	INT1 = 40,
	FLOAT1 = 45,
	VEC3 = 52,
	VEC4 = 53,
	MAT4 = 54,
	TEXTURE = 60,
	CUBEMAP = 61
};

// All data required for different types of uniforms can be stored in the matrix.
// The first row is the vector and individual values starting from top left [0][0]
struct ShaderUniform {
	std::string name = "setup";
	std::string resourceName = "setup";
	Mat4f dataMatrix;

	UniformType type = UniformType::NONE;
};

enum class ShaderType {
	NONE = -1,
	VERTEX = 0,
	FRAGMENT = 1,
	// Compute requires opengl 4.3 to work
	COMPUTE = 2
};

struct ShaderProgramSource {
	std::string vertexSource;
	std::string fragmentSource;
};

class Shader {
private:
	std::string m_FilePath;
	GLuint m_RendererID;
	std::unordered_map<std::string, GLint> m_UniformLocationCache;
	unsigned int missingUniformCounter = 0;

public:
	//std::vector<ShaderUniform> uniforms;

public:

	Shader();
	Shader(const std::string& filepath);
	~Shader();

	void bind() const;
	void unBind() const;

	void deleteProgram();

	GLuint getID() const;

	inline std::string getFilePath() const { return m_FilePath; };

	// set uniforms
	void setUniform1i(const std::string& name, int value);
	void setUniform1f(const std::string & name, float value);
	void setUniform3f(const std::string & name, Vec3f floats);
	void setUniform4f(const std::string& name, Vec4f floats);
	void setUniformMatrix4fv(const std::string& name, GLsizei count, GLboolean transpose, Mat4f matrix);

	void setUniformBlock(const std::string &blockName, GLuint blockBindingPt);

private:
	// Reads the GLSL code in the file so both vertex and fragment shaders can be in one file
	ShaderProgramSource parseShader(const std::string& filePath);
	// Compiles shader into program
	GLuint compileShader(GLuint type, const std::string& sourceCode);
	// Compiles and attaches the shaders to the program 
	GLuint createShader(const std::string& vertexShader, const std::string& fragShader);

	GLint getUniformLocation(const std::string& name);
};
