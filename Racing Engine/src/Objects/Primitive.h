#pragma once

#include "Objects.h"

class Cube : public Model {
protected:
	void setupRendering();

public:
	bool setup = false;

	Vec3f colour;

	float cubeVertices[192] = {
		// Front vertices
		-0.5f, -0.5f, 0.5f,		0.f, 0.f, 1.f,		0.f, 0.f, // 0
		0.5f, -0.5f, 0.5f,		0.f, 0.f, 1.f,		1.f, 0.f,
		0.5f, 0.5f, 0.5f,		0.f, 0.f, 1.f,		1.f, 1.f,
		-0.5f, 0.5f, 0.5f,		0.f, 0.f, 1.f,		0.f, 1.f, // 3

		// Back vertices
		-0.5f, -0.5f, -0.5f,	0.f, 0.f, -1.f,		1.f, 0.f, // 4
		0.5f, -0.5f, -0.5f,		0.f, 0.f, -1.f,		0.f, 0.f,
		0.5f, 0.5f, -0.5f,		0.f, 0.f, -1.f,		0.f, 1.f,
		-0.5f, 0.5f, -0.5f,		0.f, 0.f, -1.f,		1.f, 1.f, // 7

		// Left vertices
		-0.5f, -0.5f, -0.5f,	-1.f, 0.f, 0.f,		0.f, 0.f, // 8
		-0.5f, -0.5f, 0.5f,		-1.f, 0.f, 0.f,		1.f, 0.f,
		-0.5f, 0.5f, 0.5f,		-1.f, 0.f, 0.f,		1.f, 1.f,
		-0.5f, 0.5f, -0.5f,		-1.f, 0.f, 0.f,		0.f, 1.f, // 11

		// Right vertices
		0.5f, -0.5f, 0.5f,		1.f, 0.f, 0.f,		0.f, 0.f, // 12
		0.5f, -0.5f, -0.5f,		1.f, 0.f, 0.f,		1.f, 0.f,
		0.5f, 0.5f, -0.5f,		1.f, 0.f, 0.f,		1.f, 1.f,
		0.5f, 0.5f, 0.5f,		1.f, 0.f, 0.f,		0.f, 1.f, // 15

		// Top vertices
		-0.5f, 0.5f, 0.5f,		0.f, 1.f, 0.f,		0.f, 0.f, // 16
		0.5f, 0.5f, 0.5f,		0.f, 1.f, 0.f,		1.f, 0.f,
		0.5f, 0.5f, -0.5f,		0.f, 1.f, 0.f,		1.f, 1.f,
		-0.5f, 0.5f, -0.5f,		0.f, 1.f, 0.f,		0.f, 1.f, // 19

		// Bottom vertices
		-0.5f, -0.5f, 0.5f,		0.f, -1.f, 0.f,		1.f, 0.f, // 20
		0.5f, -0.5f, 0.5f,		0.f, -1.f, 0.f,		0.f, 0.f,
		0.5f, -0.5f, -0.5f,		0.f, -1.f, 0.f,		0.f, 1.f,
		-0.5f, -0.5f, -0.5f,	0.f, -1.f, 0.f,		1.f, 1.f // 23
	};

	GLuint indices[36] = {
		// Front
		0, 1, 2,
		2, 3, 0,
		// Back side
		5, 4, 7,
		7, 6, 5,
		// Left side
		8, 9, 10,
		10, 11, 8,
		// Right side
		12, 13, 14,
		14, 15, 12,
		// Top side
		16, 17, 18,
		18, 19, 16,
		// Bottom side
		21, 20, 23,
		23, 22, 21,
	};

public:
	Cube();
	Cube(Vec3f position);
	~Cube();

	void init();
	void update(double dt);

	void setTextureCoordRange(float min, float max);
};

class Sphere : public Model{
protected:
	void setupRendering();
public:
	static std::vector<objl::Vertex> modelVertices;

	bool setup = false;

	Vec3f colour;
public:
	Sphere();
	Sphere(Vec3f position);
	~Sphere();

	void init();
	void update(double dt);
};

class ModelObject : public Model {
protected:
	void setupRendering();
public:

	bool setup = false;

	Vec3f colour;
public:
	ModelObject();
	ModelObject(Vec3f position);
	~ModelObject();

	void init();
	void update(double dt);

	void loadModel(const std::string &filepath);
};

// Light classes were made following an openGL guide explaining how they work.
// https://learnopengl.com/Lighting/Light-casters
// https://learnopengl.com/Lighting/Multiple-lights

// Global light source to illuminate an entire scene.
class DirectionalLight : public Cube {
private:
	Vec3f lightColour;

public:
	Vec3f direction;

	Vec3f ambient;
	Vec3f diffuse;
	Vec3f specular;

public:
	DirectionalLight(Vec3f position);
	~DirectionalLight();

	void init();
	void update(double dt);

	void setLightColour(Vec3f &colour);
	Vec3f getLightColour();

private:
	void calculateLight();
};

// A light source that illuminates in all directions at a position.
// With light rays fading out over distance.
class PointLight : public Cube {
private:
	Vec3f lightColour;

public:
	Vec3f ambient;
	Vec3f diffuse;
	Vec3f specular;

	// Reduces intensity of light over distance.

	float constant;
	float linear;
	float quadratic;

public:
	PointLight(Vec3f position);
	~PointLight();

	void init();
	void update(double dt);

	void setLightColour(Vec3f &colour);
	Vec3f getLightColour();

private:
	void calculateLight();
};

class SpotLight : public Cube {
private:
	Vec3f lightColour;

public:
	Vec3f direction;

	Vec3f ambient;
	Vec3f diffuse;
	Vec3f specular;

	// Reduces intensity of light over distance.

	float constant;
	float linear;
	float quadratic;

	// Determines the radius of spotLight as value of cosine.

	float cutOff;
	float outerCutOff;

public:
	SpotLight(Vec3f position, Vec3f direction);
	~SpotLight();

	void init();
	void update(double dt);

	void setLightColour(Vec3f &colour);
	Vec3f getLightColour();

private:
	void calculateLight();
};