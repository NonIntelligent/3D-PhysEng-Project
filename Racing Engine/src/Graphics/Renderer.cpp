#include "Renderer.h"
#include <string>
#include <iostream>

#include "GLFW/glfw3.h"
#include "ErrorLogger.h"

void Renderer::init() {
	perspective();
	lookAt();
	setupOpenGLOptions();
	setupShaders();
	setupTextures();
	setupGlobalUniforms();
	setupLights();
}

Renderer::~Renderer() {
	deleteAllTextures();
	deleteAllShaders();
	deleteAllLights();
}

void Renderer::clear() const {
	glClearColor(0.2, 0.2, 0.2, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::draw(const Model *model, const std::string & name) {
	Shader* current = &defaultShader;

	auto result = shaders.find(name);
	// Shader has not been found
	if(result == shaders.end()) {
		// Create shader and add to hashmap
		Shader shader = Shader("res/Shaders/" + name + ".shader");
		if(shader.getID() != 0) {
			shaders[name] = shader;
			current = &shaders[name];
		}
	}
	else {
		current = &(result->second);
	}

	current->bind();
	model->va->bind();

	// Check shaders to see if lights uniform data needs to be applied.
	// PROBLEM: Uniform types are stored per model and are specific to that model but
	// light data is situational.
	// SOLUTION: Uniform types stay the same but create new method to set uniform data to these shaders.
	// Setting uniform data with incorrect names does not cause error. So in event of missing uniforms, program will continue.
	// Light objects will need to be stored in Renderer class.

	// Does name match with any shaders with lighting uniforms.
	if(name == "Lighting") {
		updateLightUniforms(current);
	}

	// Updates shader uniform values based on model
	updateShaderUniform(current, model->uniforms);

	if(model->isIndexBuffer()) {
		// Assumed index buffer to be unsigned int
		model->ib->bind();
		GLCall(glDrawElements(GL_TRIANGLES, model->ib->getCount(), GL_UNSIGNED_INT, nullptr));
		model->ib->unBind(); // Next iteration may not use index buffer so previous ib should not be bound.
		return;
	}

	GLCall(glDrawArrays(GL_TRIANGLES, 0, model->vb->getCount()));
}

void Renderer::drawSkybox(const Cube * skybox, const std::string & name) {
	// Draw even when depth values are equal.
	glDepthFunc(GL_LEQUAL);

	Shader* current = &defaultShader;

	auto result = shaders.find(name);
	// Shader has not been found
	if(result == shaders.end()) {
		// Create shader and add to hashmap
		Shader shader = Shader("res/Shaders/" + name + ".shader");
		if(shader.getID() != 0) {
			shaders[name] = shader;
			current = &shaders[name];
		}
	}
	else {
		current = &(result->second);
	}

	current->bind();
	skybox->va->bind();

	// Updates shader uniform values based on model
	updateShaderUniform(current, skybox->uniforms);

	GLCall(glDrawArrays(GL_TRIANGLES, 0, skybox->vb->getCount()));

	glDepthFunc(GL_LESS); // Reset depth func back to normal
}

void Renderer::drawLine(const Vec3f &start, const Vec3f &end, const Vec3f &colour) const {
	float line[] = {
		start.x, start.y, start.z,
		end.x, end.y, end.z
	};

	// Fill buffer wth data in line array (2 vertices with 3 components)
	VertexArray va = VertexArray();
	VertexBuffer vb = VertexBuffer(line, 2 * 3 * sizeof(float), 2, GL_STATIC_DRAW);

	// Bind attributes to vertex buffer and array
	VertexBufferLayout layout;
	layout.push<float>(3);
	// Add all attributes to the buffer and array
	va.addBuffer((vb), layout);

	auto result = shaders.find("Line");

	Shader sh = result->second;

	sh.bind();
	va.bind();

	sh.setUniform3f("u_Colour", colour);

	GLCall(glDrawArrays(GL_LINES, 0, 2));

	result->second.unBind();
	va.unBind();
}

void Renderer::drawLights() {
	for(int i = 0; i < dirLights.size(); i++) {
		draw(&dirLights[i], dirLights[i].shaderName);
	}

	for(int i = 0; i < pointLights.size(); i++) {
		draw(&pointLights[i], pointLights[i].shaderName);
	}

	for(int i = 0; i < spotLights.size(); i++) {
		draw(&spotLights[i], spotLights[i].shaderName);
	}
}

void Renderer::perspective() {
	float fn = far + near;
	float deltaPlane = far - near;
	float aspectRatio = viewport[0] / viewport[1];
	float t = tan(toRad(fov_y) / 2.0);

	float calcHeight = near * t * 2;
	float calcWidth = calcHeight * aspectRatio;

	// https://solarianprogrammer.com/2013/05/22/opengl-101-matrices-projection-view-model/#:~:text=The%20matrix%20M%2C%20that%20contains,a%20single%20matrix%20for%20efficiency.
	
	camPerspective.setAsZero();
	camPerspective.matrix[0][0] = 1.f / (aspectRatio * t);
	camPerspective.matrix[1][1] = 1.f / t;
	camPerspective.matrix[2][2] = -fn / deltaPlane;
	camPerspective.matrix[3][2] = (-2.f * far * near) / deltaPlane;
	camPerspective.matrix[2][3] = -1.f;

}

void Renderer::lookAt() {
	Vec3f xAxis;
	Vec3f zAxis;
	Vec3f yAxis;
	Vec3f dotPos;

	zAxis = camTargetDir;
	zAxis.negate();
	zAxis.normalise();
	Vec3f::cross(&xAxis, &camUp, &zAxis);
	xAxis.normalise();
	Vec3f::cross(&yAxis, &zAxis, &xAxis);

	xaxis = xAxis;
	yaxis = yAxis;
	zaxis = zAxis;

	dotPos.x = -Vec3f::dot(&xAxis, &camPos);
	dotPos.y = -Vec3f::dot(&yAxis, &camPos);
	dotPos.z = -Vec3f::dot(&zAxis, &camPos);

	camLookAt.setColData(0, &xAxis);
	camLookAt.setColData(1, &yAxis);
	camLookAt.setColData(2, &zAxis);
	camLookAt.setRowData(3, &dotPos);

}

void Renderer::update(const double & dt) {
	// Moves camera based on direction.
	Vec3f distance;
	Vec3f::scale(&distance, &xaxis, camSpeedX * dt);
	Vec3f::add(&camPos, &camPos, &distance);

	Vec3f::scale(&distance, &Vec3f(0.f, 1.f, 0.f), camSpeedY * dt);
	Vec3f::add(&camPos, &camPos, &distance);

	Vec3f::scale(&distance, &zaxis, camSpeedZ * dt);
	Vec3f::add(&camPos, &camPos, &distance);

	// Update lighting.
	updateLights(dt);
}

void Renderer::updateCameraState(double deltaX, double deltaY, float multiplier) {

	Vec3f direction;

	// Convert screen space offset to clip space.
	double clipX = deltaX / viewport[0];
	double clipY = deltaY / viewport[1];

	horizontalAngle += toRadf(180.f * multiplier * clipX);
	verticalAngle += toRadf(180.f * multiplier * clipY);

	// Keeps angles within range of -2PI -> +2PI
	// Absolutely no idea if this is better than 2 conditional branches/statements.
	if(fabsf(horizontalAngle) >= 2 * PI_f) {
		horizontalAngle = fmodf(horizontalAngle, PI_f);
	}

	// Limit how far you can look up/down to +85 and -75 degrees.
	if(verticalAngle > 1.483f) {
		verticalAngle = 1.483f;
	}
	else if(verticalAngle < -1.309f) {
		verticalAngle = -1.309f;
	}

	// Direction vector in world coordinates.
	direction.x = cosf(verticalAngle) * -sinf(horizontalAngle);
	direction.y = sinf(verticalAngle);
	direction.z = cosf(verticalAngle) * cosf(horizontalAngle);

	// Set new target position from origin.
	camTargetDir = direction;
}

void Renderer::updateCameraUniform() {
	// Fill the buffer data
	float data[16];
	camPerspective.data(data);
	GLCall(glBindBuffer(GL_UNIFORM_BUFFER, u_CamRendererID));

	GLCall(glBufferSubData(GL_UNIFORM_BUFFER, 0, 16 * sizeof(float), data));
	camLookAt.data(data);
	GLCall(glBufferSubData(GL_UNIFORM_BUFFER, 16 * sizeof(float), 16 * sizeof(float), data));

	float pos[] {camPos.x, camPos.y, camPos.z};

	GLCall(glBufferSubData(GL_UNIFORM_BUFFER, 32 * sizeof(float), 3 * sizeof(float), pos));

	GLCall(glBindBuffer(GL_UNIFORM_BUFFER, 0));
}

void Renderer::updateLights(const double &dt) {
	for(int i = 0; i < dirLights.size(); i++) {
		dirLights[i].update(dt);
	}

	for(int i = 0; i < pointLights.size(); i++) {
		pointLights[i].update(dt);
	}

	for(int i = 0; i < spotLights.size(); i++) {
		spotLights[i].update(dt);
	}
}

void Renderer::updateShaderUniform(Shader *shader, const std::vector<ShaderUniform> &uniforms) {
	for(auto it = uniforms.begin(); it != uniforms.end(); it++) {
		if((*it).name == "setup" || (*it).type == UniformType::NONE) {
			std::cout << "Uniform not setup, skipping." << std::endl;
			continue;
		}

		switch((*it).type) {
		case UniformType::VEC3:
			shader->setUniform3f((*it).name, (*it).dataMatrix.getRowAsVec3(0));
			break;
		case UniformType::VEC4:
			shader->setUniform4f((*it).name, (*it).dataMatrix.getRowAsVec4(0));
			break;
		case UniformType::MAT4:
			shader->setUniformMatrix4fv((*it).name, 1, GL_TRUE, (*it).dataMatrix);
			break;
		case UniformType::INT1:
			shader->setUniform1i((*it).name, (*it).dataMatrix.matrix[0][0]);
			break;
		case UniformType::FLOAT1:
			shader->setUniform1f((*it).name, (*it).dataMatrix.matrix[0][0]);
			break;
		case UniformType::TEXTURE:
		{
			Texture texture = findTexture((*it).resourceName);
			// Texture could not be found
			if(texture.getName() == "error") {
				textures.at(0).bind(0);
				shader->setUniform1i((*it).name, 0);
				break;
			}

			unsigned int bindingPoint = (*it).dataMatrix.matrix[0][0];


			if(bindingPoint > 16) {
				textures.at(0).bind(0);
				shader->setUniform1i((*it).name, (int)bindingPoint);
				break;
			}

			texture.bind(bindingPoint);

			shader->setUniform1i((*it).name, (int)bindingPoint);

			break;
		}
		case UniformType::CUBEMAP:
		{
			Texture texture = findTexture((*it).resourceName);
			// Texture could not be found
			if(texture.getName() == "error") {
				break;
			}

			unsigned int bindingPoint = (*it).dataMatrix.matrix[0][0];
			GLCall(glActiveTexture(GL_TEXTURE0 + bindingPoint));
			GLCall(glBindTexture(GL_TEXTURE_CUBE_MAP, texture.getTextureID()));
			shader->setUniform1i((*it).name, (int)bindingPoint);

			break;
		}
		default:
			printf("Uniform type not supported");
			break;
		}
	}
}

void Renderer::updateLightUniforms(Shader * shader) {
	// Set the number of lights in the scene
	shader->setUniform1i("num_dirLights", dirLights.size());
	shader->setUniform1i("num_pointLights", pointLights.size());
	shader->setUniform1i("num_spotLights", spotLights.size());

	// Set directionalLight uniforms
	for(int i = 0; i < dirLights.size() && i < MAX_DIR_LIGHTS; i++) {
		std::string number = std::to_string(i);
		shader->setUniform3f("dirLights[" + number + "].direction", dirLights[i].direction);
		shader->setUniform3f("dirLights[" + number + "].lightColour", dirLights[i].getLightColour());
		shader->setUniform3f("dirLights[" + number + "].ambient", dirLights[i].ambient);
		shader->setUniform3f("dirLights[" + number + "].diffuse", dirLights[i].diffuse);
		shader->setUniform3f("dirLights[" + number + "].specular", dirLights[i].specular);
	}

	// Set pointLight uniforms
	for(int i = 0; i < pointLights.size() && i < MAX_POINT_LIGHTS; i++) {
		std::string number = std::to_string(i);
		shader->setUniform3f("pointLights[" + number + "].position", pointLights[i].position);
		shader->setUniform3f("pointLights[" + number + "].lightColour", pointLights[i].getLightColour());
		shader->setUniform3f("pointLights[" + number + "].ambient", pointLights[i].ambient);
		shader->setUniform3f("pointLights[" + number + "].diffuse", pointLights[i].diffuse);
		shader->setUniform3f("pointLights[" + number + "].specular", pointLights[i].specular);

		shader->setUniform1f("pointLights[" + number + "].constant", pointLights[i].constant);
		shader->setUniform1f("pointLights[" + number + "].linear", pointLights[i].linear);
		shader->setUniform1f("pointLights[" + number + "].quadratic", pointLights[i].quadratic);
	}

	// Set spotLight uniforms
	for(int i = 0; i < spotLights.size() && i < MAX_SPOT_LIGHTS; i++) {
		std::string number = std::to_string(i);
		shader->setUniform3f("spotLights[" + number + "].position", spotLights[i].position);
		shader->setUniform3f("spotLights[" + number + "].direction", spotLights[i].direction);
		shader->setUniform3f("spotLights[" + number + "].lightColour", spotLights[i].getLightColour());
		shader->setUniform3f("spotLights[" + number + "].ambient", spotLights[i].ambient);
		shader->setUniform3f("spotLights[" + number + "].diffuse", spotLights[i].diffuse);
		shader->setUniform3f("spotLights[" + number + "].specular", spotLights[i].specular);

		shader->setUniform1f("spotLights[" + number + "].constant", spotLights[i].constant);
		shader->setUniform1f("spotLights[" + number + "].linear", spotLights[i].linear);
		shader->setUniform1f("spotLights[" + number + "].quadratic", spotLights[i].quadratic);

		shader->setUniform1f("spotLights[" + number + "].cutOff", spotLights[i].cutOff);
		shader->setUniform1f("spotLights[" + number + "].outerCutOff", spotLights[i].outerCutOff);
	}
}

Texture Renderer::findTexture(const std::string name) {

	Texture texture;

	for(auto it = textures.begin(); it != textures.end(); it++) {
		if(name == (*it).getName()) {
			texture = (*it);
			break;
		}
	}

	return texture;
}

void Renderer::deleteAllTextures() {
	for(auto it = textures.begin(); it != textures.end(); it++) {
		(*it).deleteTexture();
	}
	textures.clear();
}

void Renderer::deleteAllShaders() {
	for(auto &elem: shaders) {
		elem.second.deleteProgram();
	}
	shaders.clear();
}

void Renderer::deleteAllLights() {
	for(auto &elem : dirLights) {
		elem.deleteHeapAllocation();
	}

	for(auto &elem : pointLights) {
		elem.deleteHeapAllocation();
	}

	for(auto &elem : spotLights) {
		elem.deleteHeapAllocation();
	}

	dirLights.clear();
	pointLights.clear();
	spotLights.clear();
}

void Renderer::setupGlobalUniforms() {
	GLCall(glGenBuffers(1, &u_CamRendererID));
	GLCall(glBindBuffer(GL_UNIFORM_BUFFER, u_CamRendererID));
	// 2 matrices with 16 floats in each + 3 floats for position.
	GLCall(glBufferData(GL_UNIFORM_BUFFER, (2 * 16 + 3) * sizeof(float), NULL, GL_STATIC_DRAW));
	GLCall(glBindBuffer(GL_UNIFORM_BUFFER, 0));

	// Bind uniform buffer to a binding point 0, with the buffer object at offset 0 of size.
	GLCall(glBindBufferRange(GL_UNIFORM_BUFFER, 0, u_CamRendererID, 0, (2 * 16 + 3) * sizeof(float)));

	updateCameraUniform();
}

void Renderer::setupOpenGLOptions() {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glEnable(GL_DEBUG_OUTPUT);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Renderer::setupShaders() {
	Shader shader = Shader("res/Shaders/default.shader");
	shader.setUniformBlock("camera", 0);
	shaders["default"] = shader;

	shader = Shader("res/Shaders/Lighting.shader");
	shader.setUniformBlock("camera", 0);
	shaders["Lighting"] = shader;

	shader = Shader("res/Shaders/Line.shader");
	shader.setUniformBlock("camera", 0);
	shaders["Line"] = shader;

	shader = Shader("res/Shaders/CubeColoured.shader");
	shader.setUniformBlock("camera", 0);
	shaders["CubeColoured"] = shader;

	shader = Shader("res/Shaders/Skybox.shader");
	shader.setUniformBlock("camera", 0);
	shaders["Skybox"] = shader;
}

void Renderer::setupTextures() {
	// default texture is always bound to 0.
	Texture texture;
	texture.loadTexture("res/Textures/missing.png", "error");
	texture.bind(0);
	textures.push_back(texture);

	// Load all standard textures.
	std::vector<std::string> texturePaths;
	texturePaths.push_back("res/Textures/box.png");
	texturePaths.push_back("res/Textures/grass.png");
	texturePaths.push_back("res/Textures/car-texture.png");
	texturePaths.push_back("res/Textures/grey-asphalt.png");

	for(int i = 0; i < texturePaths.size(); i++) {
		texture = Texture();
		texture.loadTexture(texturePaths[i], texturePaths[i].substr(13, texturePaths[i].length() - 17));
		textures.push_back(texture);
	}

	// Load skybox.
	texturePaths.clear();
	texturePaths.push_back("res/Textures/skyboxNightRIGHT.png");
	texturePaths.push_back("res/Textures/skyboxNightLEFT.png");
	texturePaths.push_back("res/Textures/skyboxNightBOTTOM.png");
	texturePaths.push_back("res/Textures/skyboxNightTOP.png");
	texturePaths.push_back("res/Textures/skyboxNightFRONT.png");
	texturePaths.push_back("res/Textures/skyboxNightBACK.png");

	texture = Texture();
	if(! texture.loadCubeMap(texturePaths.data(), "skybox")) {
		std::cout << "Cubemap failed to load entirely." << std::endl;
	}
	textures.push_back(texture);
}

void Renderer::setupLights() {
	Vec3f colour = Vec3f(0.09f, 0.153f, 0.239f);

	DirectionalLight light1 = DirectionalLight(Vec3f{1.f, 100000.f, 1.f});
	light1.setLightColour(colour);
	light1.init();
	light1.specular.set(0.2f, 0.2f, 0.2f);

	PointLight light2 = PointLight(Vec3f{-35.f, 0.5f, -100.f});
	colour.set(1.f, 0.f, 0.f);
	light2.setLightColour(colour);
	light2.init();

	PointLight light3 = PointLight(Vec3f{-35.f, 0.5f, -102.f});
	colour.set(0.f, 1.f, 0.f);
	light3.setLightColour(colour);
	light3.init();

	PointLight light4 = PointLight(Vec3f{-35.f, 0.5f, -104.f});
	colour.set(0.f, 0.f, 1.f);
	light4.setLightColour(colour);
	light4.init();

	SpotLight light5 = SpotLight(Vec3f{-35.f, -0.5f, -105.f}, Vec3f{0.f, 0.f, -1.f});
	colour.set(1.f, 1.f, 1.f);
	light5.setLightColour(colour);
	light5.init();

	DirectionalLight light6 = DirectionalLight(Vec3f{35000.f, 25000.f, 90000.f});
	colour.set(0.745f, 0.745f, 0.745f);
	light6.setLightColour(colour);
	light6.init();
	light6.direction.set(-0.35f, -0.25f, -0.9f);
	light6.ambient.set(0.05f, 0.05f, 0.05f);
	light6.diffuse.set(0.5f, 0.5f, 0.5f);

	PointLight light7 = PointLight(Vec3f{-35.f, -0.5f, -110.8f});
	colour.set(0.8f, 0.f, 0.f);
	light7.setLightColour(colour);
	light7.diffuse.set(1.f, 1.f, 1.f);
	light7.linear = 0.3;
	light7.quadratic = 0.4;
	light7.init();

	PointLight light8 = PointLight(Vec3f{-35.f, -0.5f, -111.f});
	colour.set(0.f, 0.f, 0.4f);
	light8.setLightColour(colour);
	light8.diffuse.set(1.f, 1.f, 1.f);
	light8.linear = 0.3;
	light8.quadratic = 0.4;
	light8.init();

	dirLights.push_back(light1);
	pointLights.push_back(light2);
	pointLights.push_back(light3);
	pointLights.push_back(light4);
	spotLights.push_back(light5);
	dirLights.push_back(light6);
	pointLights.push_back(light7);
	pointLights.push_back(light8);


}
