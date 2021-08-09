#include "Primitive.h"

#include "GLFW/glfw3.h"

void Cube::setupRendering() {
	va = new VertexArray();

	// Use this buffer
	// Specify the data to use and its layout (x = vertecies * no. of components xyz)
	vb = new VertexBuffer(cubeVertices, 24 * 8 * sizeof(float), 24, GL_STATIC_DRAW);

	// Bind attributes to vertex buffer and array
	VertexBufferLayout layout;
	// Vertex has 8 (3 coordinate 3 normal 2 texture) components and are all floats
	layout.push<float>(3); // position
	layout.push<float>(3); // normal
	layout.push<float>(2); // texCoord
	va->addBuffer((*vb), layout);

	// Using index buffer saves 21% of bytes (912 / 1152).
	// 912 bytes at 24 vertices and 36 indices. 1152 bytes at 36 vertices and 0 indices.

	ib = new IndexBuffer(indices, 36, GL_STATIC_DRAW);
	hasIndexBuffer = true;

	shaderName = "Lighting";

	Vec3f temp = Vec3f(1.0f, 1.0f, 1.0f);

	ShaderUniform u_Colour;
	u_Colour.name = "u_Colour";
	u_Colour.dataMatrix.setRowData(0, &colour);
	u_Colour.type = UniformType::VEC3;
	ShaderUniform textureSlot;
	textureSlot.name = "u_Texture";
	textureSlot.resourceName = "box";
	textureSlot.dataMatrix.matrix[0][0] = 1.f;
	textureSlot.type = UniformType::TEXTURE;
	ShaderUniform model;
	model.name = "model";
	model.dataMatrix = modelMatrix;
	model.type = UniformType::MAT4;

	ShaderUniform materialDiffuse;
	materialDiffuse.name = "material.diffuse";
	materialDiffuse.dataMatrix.setRowData(0, &temp);
	materialDiffuse.type = UniformType::VEC3;
	ShaderUniform materialSpecular;
	temp.set(0.1f, 0.1f, 0.1f);
	materialSpecular.name = "material.specular";
	materialSpecular.dataMatrix.setRowData(0, &temp);
	materialSpecular.type = UniformType::VEC3;
	ShaderUniform materialShininess;
	materialShininess.name = "material.shininess";
	materialShininess.dataMatrix.matrix[0][0] = 64.f;
	materialShininess.type = UniformType::FLOAT1;


	uniforms.push_back(u_Colour);
	uniforms.push_back(textureSlot);
	uniforms.push_back(model);
	uniforms.push_back(materialDiffuse);
	uniforms.push_back(materialSpecular);
	uniforms.push_back(materialShininess);

}

Cube::Cube() {
}

Cube::Cube(Vec3f position) {
	this->position = position;

	phys = Physics(&this->position, &this->rotationAngles);
	phys.sizes = Vec3f(0.5f * scale.x, 0.5f * scale.y, 0.5f * scale.z);
	usePhysics = true;

	collider = new BoxCollider(this);
	BoxCollider* c = (BoxCollider*) collider;
	c->setAABB(false);
	c->init();
	collider->setScale(scale);
	useCollision = true;
}

Cube::~Cube() {

}

void Cube::init() {
	setupRendering();

	setup = true;
}

void Cube::update(double dt) {
	if(usePhysics) {
		phys.update(dt);
		// Updates centre of mass.
		phys.centreOfMass = position;

		phys.sizes = collider->sizes;
	}

	if(useCollision) collider->update(dt);

	// Scale, then rotate and finally translate
	Mat4f::scaleVec(&modelMatrix, scale);
	Mat4f::rotate(&modelMatrix, Vec3f{1.f, 1.f, 1.f}, rotationAngles);
	Mat4f::translate(&modelMatrix, position);

	// updates uniforms
	uniforms.at(0).dataMatrix.setRowData(0, &colour);
	uniforms.at(2).dataMatrix = modelMatrix;

	// Reset values.
	modelMatrix.setAsIdentity(); // Remove this to apply transform every update
}

void Cube::setTextureCoordRange(float min, float max) {
	min = (min > max) ? 0.f : min;

	max = (max < min) ? 1.f : max;

	cubeVertices[6] = min; cubeVertices[7] = min;
	cubeVertices[14] = max; cubeVertices[15] = min;
	cubeVertices[22] = max; cubeVertices[23] = max;
	cubeVertices[30] = min; cubeVertices[31] = max;

	cubeVertices[38] = max; cubeVertices[39] = min;
	cubeVertices[46] = min; cubeVertices[47] = min;
	cubeVertices[54] = min; cubeVertices[55] = max;
	cubeVertices[62] = max; cubeVertices[63] = max;

	cubeVertices[70] = min; cubeVertices[71] = min;
	cubeVertices[78] = max; cubeVertices[79] = min;
	cubeVertices[86] = max; cubeVertices[87] = max;
	cubeVertices[94] = min; cubeVertices[95] = max;

	cubeVertices[102] = min; cubeVertices[103] = min;
	cubeVertices[110] = max; cubeVertices[111] = min;
	cubeVertices[118] = max; cubeVertices[119] = max;
	cubeVertices[126] = min; cubeVertices[127] = max;

	cubeVertices[134] = min; cubeVertices[135] = min;
	cubeVertices[142] = max; cubeVertices[143] = min;
	cubeVertices[150] = max; cubeVertices[151] = max;
	cubeVertices[158] = min; cubeVertices[159] = max;

	cubeVertices[166] = max; cubeVertices[167] = min;
	cubeVertices[174] = min; cubeVertices[175] = min;
	cubeVertices[182] = min; cubeVertices[183] = max;
	cubeVertices[190] = max; cubeVertices[191] = max;
}

DirectionalLight::DirectionalLight(Vec3f position) {
	this->position = position;
	direction.set(-0.2, -1.0f, -0.3f);

	ambient.set(0.4f, 0.4f, 0.4f);
	diffuse.set(0.5f, 0.5f, 0.5f);
	specular.set(0.7f, 0.7f, 0.7f);
}

DirectionalLight::~DirectionalLight() {
}

void DirectionalLight::init() {
	// Cube setup rendering
	setupRendering();
	uniforms.clear();

	scale.set(0.2f, 0.2f, 0.2f);

	shaderName = "CubeColoured";

	ShaderUniform u_Colour;
	u_Colour.name = "u_Colour";
	u_Colour.dataMatrix.setRowData(0, &colour);
	u_Colour.type = UniformType::VEC3;
	ShaderUniform model;
	model.name = "model";
	model.dataMatrix = modelMatrix;
	model.type = UniformType::MAT4;

	uniforms.push_back(u_Colour);
	uniforms.push_back(model);

	setup = true;
}

void DirectionalLight::update(double dt) {
	// Scale, then rotate and finally translate
	Mat4f::scaleVec(&modelMatrix, scale);
	Mat4f::rotate(&modelMatrix, Vec3f{1.f, 0.f, 0.f}, 0.0);
	Mat4f::translate(&modelMatrix, position);

	// updates uniforms
	uniforms.at(0).dataMatrix.setRowData(0, &colour);
	uniforms.at(1).dataMatrix = modelMatrix;

	// Reset values.
	modelMatrix.setAsIdentity(); // Remove this to apply transform every update
}

void DirectionalLight::setLightColour(Vec3f & colour) {
	lightColour = colour;
	this->colour = colour;
}

Vec3f DirectionalLight::getLightColour() {
	return lightColour;
}

void DirectionalLight::calculateLight() {

}

PointLight::PointLight(Vec3f position) : Cube(position){

	ambient.set(0.05f, 0.05f, 0.05f);
	diffuse.set(0.8f, 0.8f, 0.8f);
	specular.set(1.f, 1.f, 1.f);

	// Intensity will reach 1% at ~36 units

	constant = 1.f;
	linear = 0.14f;
	quadratic = 0.07f;
}

PointLight::~PointLight() {
}

void PointLight::init() {
	// Cube setup rendering
	setupRendering();
	uniforms.clear();

	scale.set(0.2f, 0.2f, 0.2f);

	shaderName = "CubeColoured";

	ShaderUniform u_Colour;
	u_Colour.name = "u_Colour";
	u_Colour.dataMatrix.setRowData(0, &colour);
	u_Colour.type = UniformType::VEC3;
	ShaderUniform model;
	model.name = "model";
	model.dataMatrix = modelMatrix;
	model.type = UniformType::MAT4;

	uniforms.push_back(u_Colour);
	uniforms.push_back(model);

	setup = true;
}

void PointLight::update(double dt) {
	if (usePhysics) phys.update(dt);

	// Scale, then rotate and finally translate
	Mat4f::scaleVec(&modelMatrix, scale);
	Mat4f::rotate(&modelMatrix, Vec3f{1.f, 0.f, 0.f}, 0.0);
	Mat4f::translate(&modelMatrix, position);

	// updates uniforms
	uniforms.at(0).dataMatrix.setRowData(0, &colour);
	uniforms.at(1).dataMatrix = modelMatrix;

	// Reset values.
	modelMatrix.setAsIdentity(); // Remove this to apply transform every update
}

void PointLight::setLightColour(Vec3f & colour) {
	lightColour = colour;
	this->colour = colour;
}

Vec3f PointLight::getLightColour() {
	return lightColour;
}

void PointLight::calculateLight() {
}

SpotLight::SpotLight(Vec3f position, Vec3f direction) : Cube(position){
	this->direction = direction;

	ambient.set(0.f, 0.f, 0.f);
	diffuse.set(1.f, 1.f, 1.f);
	specular.set(1.f, 1.f, 1.f);

	// Intensity will reach 1% at ~36 units

	constant = 1.f;
	linear = 0.14f;
	quadratic = 0.07f;

	// Cone radius as a cosine value.

	cutOff = std::cosf(toRadf(12.5f));
	outerCutOff = std::cosf(toRadf(15.f));
}

SpotLight::~SpotLight() {
}

void SpotLight::init() {
	// Cube setup rendering
	setupRendering();
	uniforms.clear();

	scale.set(0.2f, 0.2f, 0.2f);

	shaderName = "CubeColoured";

	ShaderUniform u_Colour;
	u_Colour.name = "u_Colour";
	u_Colour.dataMatrix.setRowData(0, &colour);
	u_Colour.type = UniformType::VEC3;
	ShaderUniform model;
	model.name = "model";
	model.dataMatrix = modelMatrix;
	model.type = UniformType::MAT4;

	uniforms.push_back(u_Colour);
	uniforms.push_back(model);

	setup = true;
}

void SpotLight::update(double dt) {
	if(usePhysics) phys.update(dt);

	// Scale, then rotate and finally translate
	Mat4f::scaleVec(&modelMatrix, scale);
	Mat4f::rotate(&modelMatrix, Vec3f{1.f, 0.f, 0.f}, 0.0);
	Mat4f::translate(&modelMatrix, position);

	// updates uniforms
	uniforms.at(0).dataMatrix.setRowData(0, &colour);
	uniforms.at(1).dataMatrix = modelMatrix;

	// Reset values.
	modelMatrix.setAsIdentity(); // Remove this to apply transform every update
}

void SpotLight::setLightColour(Vec3f & colour) {
	lightColour = colour;
	this->colour = colour;
}

Vec3f SpotLight::getLightColour() {
	return lightColour;
}

void SpotLight::calculateLight() {
}

std::vector<objl::Vertex> Sphere::modelVertices = std::vector<objl::Vertex>();

void ModelObject::setupRendering() {
	setupModelWithoutIndex();

	shaderName = "Lighting";

	ShaderUniform u_Colour;
	u_Colour.name = "u_Colour";
	u_Colour.dataMatrix.setRowData(0, &colour);
	u_Colour.type = UniformType::VEC4;
	ShaderUniform textureSlot;
	textureSlot.name = "u_Texture";
	textureSlot.resourceName = "grey-asphalt";
	textureSlot.dataMatrix.matrix[0][0] = 1.f;
	textureSlot.type = UniformType::TEXTURE;
	ShaderUniform model;
	model.name = "model";
	model.dataMatrix = modelMatrix;
	model.type = UniformType::MAT4;


	uniforms.push_back(u_Colour);
	uniforms.push_back(textureSlot);
	uniforms.push_back(model);
}

ModelObject::ModelObject() {
}

ModelObject::ModelObject(Vec3f position) {
	this->position = position;

	phys = Physics(&this->position, &this->rotationAngles);
	usePhysics = false;

	collider = new BoxCollider(this);
	BoxCollider* c = (BoxCollider*)collider;
	c->setAABB(false);
	collider->setScale(scale, {0.5f, 0.5f, 0.5f});
	c->init();
	useCollision = true;
}

ModelObject::~ModelObject() {
}

void ModelObject::init() {
	setupRendering();

	setup = true;
}

void ModelObject::update(double dt) {
	if(usePhysics) {
		phys.update(dt);
	}

	if(useCollision) collider->update(dt);

	// Scale, then rotate and finally translate
	Mat4f::scaleVec(&modelMatrix, scale);
	Mat4f::rotate(&modelMatrix, Vec3f{1.f, 1.f, 1.f}, phys.angularPos);
	Mat4f::translate(&modelMatrix, position);

	// updates uniforms
	for(auto it = uniforms.begin(); it != uniforms.end(); it++) {
		if((*it).name == "u_Colour") {
			(*it).dataMatrix.setRowData(0, &colour);
		}
		else if((*it).name == "model") {
			(*it).dataMatrix = modelMatrix; // Copy values
		}
	}

	// Reset values and unbind
	modelMatrix.setAsIdentity(); // Remove this to apply transform every update
}

void ModelObject::loadModel(const std::string & filepath) {
	objl::Loader loader;
	if(!loader.LoadFile(filepath)) {
		std::cout << "Model file did not load correctly" << std::endl;
		return;
	}
	this->vertices = loader.LoadedVertices;
	this->indices = loader.LoadedIndices;
}

void Sphere::setupRendering() {
	setupModelWithoutIndex();

	shaderName = "Lighting";

	ShaderUniform u_Colour;
	u_Colour.name = "u_Colour";
	u_Colour.dataMatrix.setRowData(0, &colour);
	u_Colour.type = UniformType::VEC4;
	ShaderUniform textureSlot;
	textureSlot.name = "u_Texture";
	textureSlot.resourceName = "gray-asphalt";
	textureSlot.dataMatrix.matrix[0][0] = 1.f;
	textureSlot.type = UniformType::TEXTURE;
	ShaderUniform model;
	model.name = "model";
	model.dataMatrix = modelMatrix;
	model.type = UniformType::MAT4;


	uniforms.push_back(u_Colour);
	uniforms.push_back(textureSlot);
	uniforms.push_back(model);
}

Sphere::Sphere() {
}

Sphere::Sphere(Vec3f position) {
	this->position = position;

	phys = Physics(&this->position, &this->rotationAngles);
	usePhysics = true;

	collider = new SphereCollider(this);
	SphereCollider* s = (SphereCollider*)collider;
	s->setRadius(1.f);
	s->init();
	useCollision = true;
}

Sphere::~Sphere() {
}

void Sphere::init() {
	objl::Loader loader;
	if(Sphere::modelVertices.size() == 0) {
		if(!loader.LoadFile("res/Models/Sphere.obj")) {
			std::cout << "Car file did not load correctly" << std::endl;
			return;
		}
		Sphere::modelVertices = loader.LoadedVertices;
		this->vertices = Sphere::modelVertices;
		this->indices = loader.LoadedIndices;
	}
	else {
		this->vertices = Sphere::modelVertices;
	}
	setupRendering();

	setup = true;
}

void Sphere::update(double dt) {
	if(usePhysics) {
		phys.update(dt);
	}

	if(useCollision) collider->update(dt);

	// Scale, then rotate and finally translate
	Mat4f::scaleVec(&modelMatrix, scale);
	Mat4f::rotate(&modelMatrix, Vec3f{1.f, 1.f, 1.f}, phys.angularPos);
	Mat4f::translate(&modelMatrix, position);

	// updates uniforms
	for(auto it = uniforms.begin(); it != uniforms.end(); it++) {
		if((*it).name == "u_Colour") {
			(*it).dataMatrix.setRowData(0, &colour);
		}
		else if((*it).name == "model") {
			(*it).dataMatrix = modelMatrix; // Copy values
		}
	}

	// Reset values and unbind
	modelMatrix.setAsIdentity(); // Remove this to apply transform every update
}
