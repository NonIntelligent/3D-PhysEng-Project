#pragma once
#include <vector>
#include <unordered_map>

#include "Shader.h"
#include "VertexObjects.h"
#include "Texture.h"
#include "Maths/WorldMaths.h"
#include "Objects/Objects.h"
#include "Objects/Primitive.h"

#include "GL/glew.h"
#include "GLFW/glfw3.h"

constexpr int MAX_DIR_LIGHTS = 2;
constexpr int MAX_POINT_LIGHTS = 16;
constexpr int MAX_SPOT_LIGHTS = 16;

// Rendering components were created following the guide (https://www.youtube.com/playlist?list=PLlrATfBNZ98foTJPJ_Ev03o2oq3-GGOS2)
class Renderer {
private:
	GLuint u_CamRendererID = 0;
	Mat4f camLookAt;
	Mat4f camPerspective;


	Shader defaultShader = Shader("res/Shaders/default.shader");

public:
	Vec3f camPos = {0.f, 0.f, 3.f};
	Vec3f camTargetDir = {0.f, 0.f, -1.f};
	Vec3f camUp = {0.f, 1.f, 0.f};

	// horizontal angle, toward -Z in rads.
	float horizontalAngle = PI_f;
	// vertical angle, look at the horizon in rads.
	float verticalAngle = 0.f;

	Vec3f xaxis;
	Vec3f yaxis;
	Vec3f zaxis;

	float camSpeedX = 0.0;
	float camSpeedY = 0.0;
	float camSpeedZ = 0.0;

	double fov_y = 45.0;
	float viewport[2] = {1280, 720};
	float near = 0.1f;
	float far = 100.f;

	std::vector<Texture> textures;
	std::vector<DirectionalLight> dirLights;
	std::vector<PointLight> pointLights;
	std::vector<SpotLight> spotLights;
	std::unordered_map<std::string, Shader> shaders;


public:
	void init();

	Renderer() { };
	~Renderer();

	void clear() const;

	void draw(const Model *model, const std::string &name);

	void drawSkybox(const Cube *skybox, const std::string &name);
	void drawLine(const Vec3f &start, const Vec3f &end, const Vec3f &colour) const;
	void drawLights();

	void perspective();
	void lookAt();
	void updateCameraUniform();

	void update(const double &dt);

	// Updates camera target location
	void updateCameraState(double deltaX, double deltaY, float multiplier);

	void updateShaderUniform(Shader *shader, const std::vector<ShaderUniform> &uniforms);
	void updateLightUniforms(Shader *shader);
	Texture findTexture(const std::string name);

private:
	void updateLights(const double &dt);

	void deleteAllTextures();
	void deleteAllShaders();
	void deleteAllLights();

	void setupGlobalUniforms();
	void setupOpenGLOptions();
	void setupShaders();
	void setupTextures();
	void setupLights();
};