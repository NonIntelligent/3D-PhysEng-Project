#include <iostream>
#include <vector>

// Used to find memory leaks
//#include "vld.h"

#include "GL/glew.h"
#include "GLFW/glfw3.h"

#include "Graphics/Renderer.h"
#include "IO/Keyboard.h"
#include "IO/Mouse.h"
#include "Maths/Structures.h"
#include "Maths/WorldMaths.h"
#include "Objects/Vehicle.h"
#include "Objects/Objects.h"
#include "Objects/Primitive.h"


const int VSYNC_OFF = 0;
const int VSYNC_ON = 1;
const int VSYNC_HALF = 2;

GLFWwindow* window;
GLFWmonitor* monitor;
Renderer* renderer;
Mouse mouse;
bool running = false;
bool noClipToggle = false;
bool debug = false;

std::vector<GameObject*> objects;

Car* car;
Cube* skybox;
Cube* ground;

// Methods decleration
void window_size_callback(GLFWwindow* window, int width, int height);
std::vector<Model*> broadPhaseCollision();
bool isColliding(const Model* a, const Model* b);
void resolveCollisions(std::vector<Model*> &collisions);

bool initGLFW() {
	if(!glfwInit())
		return false;

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	window = glfwCreateWindow(1280, 720, "Racing Engine", NULL, NULL);
	if(!window) {
		glfwTerminate();
		return false;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(VSYNC_ON);

	glfwSetWindowSizeCallback(window, window_size_callback);
	glfwSetKeyCallback(window, Keyboard::key_callback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	monitor = glfwGetPrimaryMonitor();

	return true;
}

bool initGLEW() {
	if(glewInit() != GLEW_OK) {
		GLenum err = 0;
		glewGetErrorString(err);
		std::cout << err << std::endl;
		return false;
	}

	return true;
}

bool initALL() {

	if(!initGLFW()) {
		std::cout << "GLFW initialisation failed" << std::endl;
		return false;
	}

	if(!initGLEW()) {
		std::cout << "GLEW initialisation failed" << std::endl;
		return false;
	}

	std::cout << glGetString(GL_VERSION) << std::endl;
	std::cout << "Primary monitor refresh rate: " << glfwGetVideoMode(monitor)->refreshRate << "hz" << std::endl;

	// Initialise objects here
	renderer = new Renderer();
	renderer->init();

	car = new Car("generic", Vec3f{14.f, 0.f, -2.f});
	Cube* cube1 = new Cube(Vec3f{-35.f, 0.f, -100.f});
	Cube* cube2 = new Cube(Vec3f{-35.f, 0.f, -102.f});
	Cube* cube3 = new Cube(Vec3f{-35.f, 0.f, -104.f});
	Cube* cube4 = new Cube(Vec3f{-35.f, 10.f, -106.f});
	Cube* cube5 = new Cube(Vec3f{-35.f, 10.f, -108.f});
	Cube* cube6 = new Cube(Vec3f{-35.f, 10.f, -110.f});
	ModelObject* track = new ModelObject(Vec3f{2.f, -1.f, -50.f});
	track->scale.set(1000.f, 1000.f, 1000.f);
	track->collider->setScale({1.f, 1.f, 1.f}, {50.16f, 20.f, 60.8f});
	track->phys.frictionCoeff_static = 10.f;
	track->phys.frictionCoeff_dynamic = 50.f;

	track->loadModel("res/Models/Track2.obj");
	track->collider->solid = false;
	track->collider->ground = true;

	skybox = new Cube(Vec3f{0.f, 0.f, 0.f});

	ground = new Cube(Vec3f{0.f, -6.f, 0.f});
	ground->scale.set(2000.f, 10.f, 2000.f);
	ground->collider->setScale(ground->scale);
	ground->collider->ground = true;
	BoxCollider* c = (BoxCollider*)ground->collider;
	c->setAABB(false);
	ground->setTextureCoordRange(0.f, 120.f);
	ground->phys.mass = 1.f;
	ground->phys.restitutionCoeff = 1.0f;
	ground->phys.frictionCoeff_static = 1.f;
	ground->phys.frictionCoeff_dynamic = 0.9f;

	objects.push_back(car);
	objects.push_back(cube1);
	objects.push_back(cube2);
	objects.push_back(cube3);
	objects.push_back(cube4);
	objects.push_back(cube5);
	objects.push_back(cube6);
	objects.push_back(track);
	objects.push_back(ground);

	for(int i = 0; i < objects.size(); i++) {
		objects.at(i)->init();
	}

	cube4->phys.velocity.set(0.f, -10.f, 0.f);
	cube4->phys.dragCoefficient = 0.1f;

	cube5->phys.velocity.set(0.f, -10.f, 0.f);
	cube5->phys.dragCoefficient = 0.1f;

	ShaderUniform grassTexture;
	grassTexture.name = "u_Texture";
	grassTexture.resourceName = "grass";
	grassTexture.dataMatrix.matrix[0][0] = 1.f;
	grassTexture.type = UniformType::TEXTURE;
	ground->uniforms.at(1) = grassTexture;
	ground->usePhysics = false;


	// Alter cube to support skybox
	skybox->init();

	float cubeVertices[108] = {
		// positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f
	};

	delete(skybox->va);
	delete(skybox->vb);
	skybox->va = new VertexArray();

	skybox->vb = new VertexBuffer(cubeVertices, 36 * 3 * sizeof(float), 36, GL_STATIC_DRAW);

	// Bind attributes to vertex buffer and array
	VertexBufferLayout layout;
	// Vertex has 8 (3 coordinate 3 normal 2 texture) components and are all floats
	layout.push<float>(3); // position
	skybox->va->addBuffer((*skybox->vb), layout);

	// Alter cube to support skybox
	skybox->shaderName = "Skybox";
	skybox->uniforms.clear();
	ShaderUniform skyboxTexture;
	skyboxTexture.name = "u_Texture";
	skyboxTexture.resourceName = "skybox";
	skyboxTexture.dataMatrix.matrix[0][0] = 1.f;
	skyboxTexture.type = UniformType::CUBEMAP;
	skybox->uniforms.push_back(skyboxTexture);
	skybox->usePhysics = false;
	skybox->useCollision = false;


	return true;
}

// Physics updates here at 60 ticks per second
void physicsUpdate(double timeStep) {
	const float radius = 20.f;
	float camX = sin(glfwGetTime()) * radius;
	float camZ = cos(glfwGetTime()) * radius;

	// Update all objects
	for(auto it = objects.begin(); it != objects.end(); it++) {
		(*it)->update(timeStep);
	}


	renderer->update(timeStep);

	/*
	NOTE: Collision detection currently does not account for velocity. So high-speed object may phase through each other. 
	*/

	std::vector<Model*> actualCollision;
	std::vector<Model*> broadCollision = broadPhaseCollision();

	for(int i = 0; i < broadCollision.size() / 2; i++) {
		if(isColliding(broadCollision.at(i * 2), broadCollision.at(i * 2 + 1))) {
			actualCollision.push_back(broadCollision.at(i * 2));
			actualCollision.push_back(broadCollision.at(i * 2 + 1));
		}
	}

	// Resolve collisions here by adding forces to objects.
	resolveCollisions(actualCollision);
}

// Render all objects within view of the camera
// Use interpolation to draw between updates
void render(double interpolate) {
	renderer->clear();

	// Decide which camera to use.
	if(!noClipToggle) {
		car->currentCamera == 1 ? renderer->camPos = car->camPos1 : renderer->camPos = car->camPos2;
		Vec3f camToCar;
		Vec3f::sub(&camToCar, &car->position, &car->camPos1);
		camToCar.normalise();
		renderer->camTargetDir = camToCar;
	}

	renderer->perspective();
	renderer->lookAt();
	renderer->updateCameraUniform();

	renderer->drawLights();

	for(auto it = objects.begin(); it != objects.end(); it++) {

		Model* model = dynamic_cast<Model*>((*it));

		// Skip if object is not a model
		if(model == nullptr) {
			continue;
		}

		renderer->draw(model, model->shaderName);
	}

	// Draw camera axis.
	if(debug) {
		Vec3f start = {0.f, 0.f, 0.f};
		Vec3f colour = {1.f, 0.f, 0.f};
		renderer->drawLine(start, renderer->xaxis, colour);
		colour.set(0.f, 1.f, 0.f);
		renderer->drawLine(start, renderer->yaxis, colour);
		colour.set(0.f, 0.f, 1.f);
		renderer->drawLine(start, renderer->zaxis, colour);
	}

	// Draw skybox last
	renderer->drawSkybox(skybox, skybox->shaderName);

	glfwSwapBuffers(window);
}

void updateKeyboardInput(double dt) {
	if(Keyboard::getKeyState(GLFW_KEY_C) == KeyState::JUST_PRESSED) {
		// Change camera angle on car
		if(!noClipToggle) {
			if(car->currentCamera < 2) {
				car->currentCamera++;
				Vec3f camToCar;
				Vec3f::sub(&camToCar, &car->position, &car->camPos2);
				camToCar.normalise();
				renderer->camTargetDir = camToCar;
			}
			else {
				car->currentCamera = 1;
				Vec3f camToCar;
				Vec3f::sub(&camToCar, &car->position, &car->camPos1);
				camToCar.normalise();
				renderer->camTargetDir = camToCar;
			}
		}
	}

	if(Keyboard::getKeyState(GLFW_KEY_F) == KeyState::JUST_PRESSED) {
		// XOR operation to toggle a bit (toggle true and false).
		noClipToggle ^= true;
		if(!noClipToggle) {
			renderer->camSpeedX = 0.f;
			renderer->camSpeedY = 0.f;
			renderer->camSpeedZ = 0.f;
		}
		else {
			car->accelerating = false;
			car->decelerating = false;
			car->right = 0;
			renderer->camTargetDir = car->headDir;
		}
	}

	// Cycle integration method for movement physics.
	if(Keyboard::getKeyState(GLFW_KEY_L) == KeyState::JUST_PRESSED) {
		Physics::integration == 4 ? Physics::integration = 1 : Physics::integration++;
		switch(Physics::integration) {
		case 1:
			std::cout << "Semi-Implicit Euler integration method in use" << std::endl;
			break;
		case 2:
			std::cout << "Velocity Verlet integration method in use" << std::endl;
			break;
		case 3:
			std::cout << "Runge-Kutta-Second-Order integration method in use" << std::endl;
			break;
		case 4:
			std::cout << "Runge-Kutta-Fourth-Order integration method in use" << std::endl;
			break;
		default:
			std::cout << "Unknown integration method in use, most likly broken" << std::endl;
			break;
		}
	}

	if(Keyboard::getKeyState(GLFW_KEY_H) == KeyState::JUST_PRESSED) {
		// XOR operation to toggle a bit (toggle true and false).
		debug ^= true;
	}

	if(Keyboard::getKeyState(GLFW_KEY_LEFT_ALT) == KeyState::JUST_PRESSED) {
		// Enables free mouse.
		mouse.freeMouse = true;
		mouse.mouseOffsetX = 0;
		mouse.mouseOffsetY = 0;
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	}
	if(Keyboard::getKeyState(GLFW_KEY_LEFT_ALT) == KeyState::JUST_RELEASED) {
		// Disables free mouse.
		glfwGetCursorPos(window, &mouse.mouseX, &mouse.mouseY);
		mouse.lastMouseX = mouse.mouseX;
		mouse.lastMouseY = mouse.mouseY;
		mouse.freeMouse = false;
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	}


	if(Keyboard::getKeyState(GLFW_KEY_W) == KeyState::JUST_PRESSED) {
		if(noClipToggle) {
			renderer->camSpeedZ = -4.f;
		}
		else {
			car->throttle = 1.f;
			car->accelerating = true;
		}
	}
	if(Keyboard::getKeyState(GLFW_KEY_S) == KeyState::JUST_PRESSED) {
		if(noClipToggle) {
			renderer->camSpeedZ = 4.f;
		}
		else {
			car->throttle = 1.f;
			car->decelerating = true;
		}
	}
	if(Keyboard::getKeyState(GLFW_KEY_A) == KeyState::JUST_PRESSED) {
		if(noClipToggle) {
			renderer->camSpeedX = -4.f;
		}
		else
			car->right = -1;
	}
	if(Keyboard::getKeyState(GLFW_KEY_D) == KeyState::JUST_PRESSED) {
		if(noClipToggle) {
			renderer->camSpeedX = 4.f;
		}
		else
			car->right = 1;
	}

	if(Keyboard::getKeyState(GLFW_KEY_SPACE) == KeyState::JUST_PRESSED) {
		if(noClipToggle) {
			renderer->camSpeedY = 4.f;
		}
	}
	if(Keyboard::getKeyState(GLFW_KEY_LEFT_CONTROL) == KeyState::JUST_PRESSED) {
		if(noClipToggle) {
			renderer->camSpeedY = -4.f;
		}
	}

	if(Keyboard::getKeyState(GLFW_KEY_W) == KeyState::JUST_RELEASED) {
		if(noClipToggle) {
			Keyboard::getKeyState(GLFW_KEY_S) == KeyState::JUST_PRESSED ? renderer->camSpeedZ = 1.f : renderer->camSpeedZ = 0.f;
		}
		else {
			car->accelerating = false;
			if (Keyboard::getKeyState(GLFW_KEY_S) == KeyState::NOT_PRESSED || Keyboard::getKeyState(GLFW_KEY_S) == KeyState::JUST_RELEASED){
				car->throttle = 0.f;
			}
		}
	}
	if(Keyboard::getKeyState(GLFW_KEY_S) == KeyState::JUST_RELEASED) {
		if(noClipToggle) {
			Keyboard::getKeyState(GLFW_KEY_W) == KeyState::JUST_PRESSED ? renderer->camSpeedZ = -1.f : renderer->camSpeedZ = 0.f;
		}
		else {
			car->decelerating = false;
			if(Keyboard::getKeyState(GLFW_KEY_W) == KeyState::NOT_PRESSED || Keyboard::getKeyState(GLFW_KEY_W) == KeyState::JUST_RELEASED) {
				car->throttle = 0.f;
			}
		}
	}
	if(Keyboard::getKeyState(GLFW_KEY_A) == KeyState::JUST_RELEASED) {
		if(noClipToggle) {
			Keyboard::getKeyState(GLFW_KEY_D) == KeyState::JUST_PRESSED ? renderer->camSpeedX = 1.f : renderer->camSpeedX = 0.f;
		}
		else {
			if(Keyboard::getKeyState(GLFW_KEY_D) == KeyState::NOT_PRESSED || Keyboard::getKeyState(GLFW_KEY_D) == KeyState::JUST_RELEASED) {
				car->right = 0;
			}
		}
	}
	if(Keyboard::getKeyState(GLFW_KEY_D) == KeyState::JUST_RELEASED) {
		if(noClipToggle) {
			Keyboard::getKeyState(GLFW_KEY_A) == KeyState::JUST_PRESSED ? renderer->camSpeedX = -1.f : renderer->camSpeedX = 0.f;
		}
		else {
			if(Keyboard::getKeyState(GLFW_KEY_A) == KeyState::NOT_PRESSED || Keyboard::getKeyState(GLFW_KEY_A) == KeyState::JUST_RELEASED) {
				car->right = 0;
			}
		}
	}
	if(Keyboard::getKeyState(GLFW_KEY_SPACE) == KeyState::JUST_RELEASED) {
		if(noClipToggle) {
			Keyboard::getKeyState(GLFW_KEY_LEFT_CONTROL) == KeyState::JUST_PRESSED ? renderer->camSpeedY = -1.f : renderer->camSpeedY = 0.f;
		}
	}
	if(Keyboard::getKeyState(GLFW_KEY_LEFT_CONTROL) == KeyState::JUST_RELEASED) {
		if(noClipToggle) {
			Keyboard::getKeyState(GLFW_KEY_SPACE) == KeyState::JUST_PRESSED ? renderer->camSpeedY = 1.f : renderer->camSpeedY = 0.f;
		}
	}

	Keyboard::updateKeyStates();
}

void updateMouseInput(double dt) {
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if(noClipToggle && state == GLFW_PRESS && mouse.currentDelay <= 0) {
		Sphere* sphere = new Sphere(renderer->camPos);
		sphere->init();
		Vec3f::scale(&sphere->phys.velocity, &renderer->camTargetDir, 24.f);
		objects.push_back(sphere);
		mouse.currentDelay = mouse.clickTickDelay;
	}

	if(mouse.freeMouse) return;

	if(!noClipToggle) return;

	glfwGetCursorPos(window, &mouse.mouseX, &mouse.mouseY);

	if (mouse.firstMouse){
		mouse.lastMouseX = mouse.mouseX;
		mouse.lastMouseY = mouse.mouseY;
		mouse.firstMouse = false;
	}

	// Calc offsets. Y is inverted due to different coordinate spaces.
	mouse.mouseOffsetX = mouse.mouseX - mouse.lastMouseX;
	mouse.mouseOffsetY = mouse.lastMouseY - mouse.mouseY;

	//glfwSetCursorPos(window, renderer->viewport[0] / 2, renderer->viewport[1] / 2);

	mouse.lastMouseX = mouse.mouseX;
	mouse.lastMouseY = mouse.mouseY;

	mouse.currentDelay -= 1;
}

void processInput(double frameTime) {
	// Process key and mouse inputs here
	glfwPollEvents();

	updateKeyboardInput(frameTime);
	updateMouseInput(frameTime);
	renderer->updateCameraState(mouse.mouseOffsetX, mouse.mouseOffsetY, mouse.mouseSensitivity);
}

// Free memory
void deleteHeapObjects() {
	delete(renderer);

	for(GameObject* elem : objects) {
		elem->deleteHeapAllocation();
		delete(elem);
	}

	objects.clear();
}


// Game loop was created following this guide https://gafferongames.com/post/fix_your_timestep/
int main() {

	if(initALL()) {
		running = true;
		Vec3f camToCar;
		Vec3f::sub(&camToCar, &car->position, &car->camPos1);
		camToCar.normalise();
		renderer->camTargetDir = camToCar;
	}

	double t = 0.0;
	double SEC_PER_UPDATE = 1.0 / 60.0;

	int ups = 0;
	int fps = 0;
	int ticks = 0;
	int frames = 0;

	// returns time in seconds after glfw initialisation
	double previousTime = glfwGetTime();
	double accumulator = 0.0;

	while(!glfwWindowShouldClose(window) && running) {
		double currentTime = glfwGetTime();
		double frameTime = currentTime - previousTime;
		if(frameTime > 0.33) // If behind by 20+ updates out of 60
			frameTime = 0.33;
		previousTime = currentTime;

		accumulator += frameTime;

		processInput(frameTime);

		while(accumulator >= SEC_PER_UPDATE) {
			physicsUpdate(SEC_PER_UPDATE);
			ticks++;
			t += SEC_PER_UPDATE;
			accumulator -= SEC_PER_UPDATE;
		}

		// render where the object should be
		render(accumulator / SEC_PER_UPDATE);
		frames++;

		if(t > 1) {
			ups = ticks;
			fps = frames;
			std::cout << "ups: " << ups << " fps: " << fps << std::endl;
			ticks = 0;
			frames = 0;
			t -= 1;
		}

	}


	deleteHeapObjects();

	glfwTerminate();
	return 0;
}

void window_size_callback(GLFWwindow * window, int width, int height) {
	renderer->viewport[0] = width;
	renderer->viewport[1] = height;
	glViewport(0, 0, width, height);
}

std::vector<Model*> broadPhaseCollision() {
	std::vector<Model*> models;
	std::vector<Model*> collisionPairs;

	for(auto it = objects.begin(); it != objects.end(); it++) {
		Model* model = dynamic_cast<Model*>((*it));
		// Skip if object is not a model
		if(model == nullptr) {
			continue;
		}
		if(!model->useCollision) {
			continue;
		}
		models.push_back(model);
	}

	// Creates a collision area based on the player's location with a set size of detection around the player.
	// If there are more than 7 objects in a node then it shall be split into 8 equally sized nodes.

	OcTree<Model*> tree = OcTree<Model*>(Vec3f{0.f, 0.f, 0.f}, Vec3f{10000.f, 500.f, 10000.f}, 6, 70);

	for(int i = 0; i < models.size(); i++) {
		tree.insert(models.at(i), models.at(i)->collider->position, models.at(i)->collider->sizes);
	}

	// Tree leaf nodes now have models stored in its contents.
	// If there are at least 2 objects then create a combination of pairs. [n! / (r! * (n-r)!) ].

	std::vector<OcTreeNode<Model*>*> nodes = tree.getAllLeafNodes();
	Model* singlePair[2];

	for(OcTreeNode<Model*>* elem : nodes) {
		if(elem->contents.size() > 1) {
			tree.getCollisionPairs(elem->contents, collisionPairs, singlePair);
		}
	}

	return collisionPairs;
}

bool isColliding(const Model* a, const Model* b) {
	BoxCollider* colliderA = dynamic_cast<BoxCollider*>(a->collider);
	BoxCollider* colliderB = dynamic_cast<BoxCollider*>(b->collider);

	if(colliderA != nullptr) {
		if(colliderB != nullptr) {
			return CollisionDetect::boxBox(colliderA, colliderB);
		}
		else if (SphereCollider* s = dynamic_cast<SphereCollider*>(b->collider)) {
			return CollisionDetect::boxSphere(colliderA, s);
		}
		else if (PlaneCollider* p = dynamic_cast<PlaneCollider*>(b->collider)) {
			return CollisionDetect::boxPlane(colliderA, p);
		}
	}

	SphereCollider* colliderSphereA = dynamic_cast<SphereCollider*>(a->collider);
	if(colliderSphereA != nullptr) {
		if(colliderB != nullptr) {
			return CollisionDetect::boxSphere(colliderB, colliderSphereA);
		}
		else if(SphereCollider* s = dynamic_cast<SphereCollider*>(b->collider)) {
			return CollisionDetect::sphereSphere(colliderSphereA, s);
		}
		else if(PlaneCollider* p = dynamic_cast<PlaneCollider*>(b->collider)) {
			return CollisionDetect::spherePlane(colliderSphereA, p);
		}
	}

	PlaneCollider* colliderPlaneA = dynamic_cast<PlaneCollider*>(a->collider);
	if(colliderPlaneA != nullptr) {
		if(colliderB != nullptr) {
			return CollisionDetect::boxPlane(colliderB, colliderPlaneA);
		}
		else if(SphereCollider* s = dynamic_cast<SphereCollider*>(b->collider)) {
			return CollisionDetect::spherePlane(s, colliderPlaneA);
		}
		else if(PlaneCollider* p = dynamic_cast<PlaneCollider*>(b->collider)) {
			return CollisionDetect::planePlane(colliderPlaneA, p);
		}
	}

	return false;
}

void resolveCollisions(std::vector<Model*> &collisions) {
	for(int i = 0; i < collisions.size(); i+= 2) {
		Model* a = collisions.at(i);
		Model* b = collisions.at(i + 1);

		ContactManifold manifold;
		for(ContactManifold &elem : a->collider->contacts) {
			if(! elem.resolved) {
				elem.resolved = true;
				manifold = elem;
				break;
			}

		}

		// Skip resolving physics collision.
		if(! a->usePhysics && ! b->usePhysics || manifold.pointCount == 0) {
			continue;
		}

		// If the collider is type Ground then calculate friction.
		if(a->collider->ground && !b->collider->ground) {
			b->phys.onGround = true;
			b->phys.calcFriction(a->phys, manifold.normal);
		}
		else if(b->collider->ground && !a->collider->ground) {
			a->phys.onGround = true;
			a->phys.calcFriction(b->phys, manifold.normal);
		}

		// Skip if the collision is not solid.
		if(!a->collider->solid || !b->collider->solid) continue;

		// When a is not allowed to move, displace b instead. Normal from B->A must also be reversed.
		if(a->usePhysics) {
			a->phys.displace(manifold);
		}
		else {
			manifold.normal.negate();
			b->phys.displace(manifold);
		}

		// Apply result of impulse to one or two objects. Apply impulse to B in direction of B if A does not use physics.
		// Normal has already been reversed in case of B applying impulse.
		a->usePhysics ? a->phys.applyImpulse(b->phys, b->usePhysics, manifold) : b->phys.applyImpulse(a->phys, false, manifold);
	}

	// Clear manifolds for next collsion detection.
	for(Model* elem: collisions) {
		elem->collider->contacts.clear();
	}
}