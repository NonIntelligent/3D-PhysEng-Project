#include "Vehicle.h"

#include <iostream>
#include <string>

// Accelerate in the direction of the head of the car
void Vehicle::drive() {
	Vec3f::scale(&phys.inputForce, &headDir, drivingForce * throttle);
}

void Vehicle::brake() {
	Vec3f brakeDir = headDir;
	brakeDir.negate();
	Vec3f::increment(&phys.inputForce, &brakeDir, brakingForce * throttle);
}

// Accelerate in the opposite direction of the head of the car
void Vehicle::reverse() {
	Vec3f forceDir = headDir;
	forceDir.negate();
	Vec3f::scale(&phys.inputForce, &forceDir, drivingForce * throttle / 2);
}

void Vehicle::turn(double dt) {
	// No longer turning
	if(right == 0.f) {
		turningDir = headDir;
		Vec3f::scale(&phys.inputAngularForce, &turningDir, 0.f);
		turningAngle = 0.f;
		return;
	}

	// Add turning force to rotate physiscs.
	// Turn right or left
	turningDir = rightDir;
	if(right < 0) {
		turningDir.negate();
	}

	turningAngle = -60 * turningSpeed * dt * right;

	//Vec3f::scale(&phys.inputAngularForce, &turningDir, turningForce);
}

bool Vehicle::isGround() {
	return position.y < 0.1 && position.y >= 0.0;
}

Car::Car(std::string name, Vec3f position) {
	this->position = position;
	this->name = name;

	phys = Physics(&this->position, &this->rotationAngles);
	phys.mass = 4.f;
	usePhysics = true;

	headDir.set(0.f, 0.f, -1.f);
	rightDir.set(1.f, 0.f, 0.f);
	upDir.set(0.f, 1.f, 0.f);

	collider = new BoxCollider(this);
	BoxCollider* c = (BoxCollider*)collider;
	c->setAABB(true);
	collider->setScale(scale, {1.2f, 0.76f, 1.15f});
	c->init();
	useCollision = true;
}

Car::~Car() {

}

void Car::init() {
	setupEngine(100.f, 120.f);
	Vec3f::scale(&phys.centreOfMass, &headDir, 0.5f);
	Vec3f::add(&phys.centreOfMass, &phys.centreOfMass, &position);

	objl::Loader loader;
	if(! loader.LoadFile("res/Models/Car.obj")) {
		std::cout << "Car file did not load correctly" << std::endl;
		return;
	}
	this->vertices = loader.LoadedVertices;
	this->indices = loader.LoadedIndices;
	setupRendering();

	setup = true;
}

void Car::update(double dt) {

	float movingDir = Vec3f::dot(&headDir, &phys.velocity);
	if (phys.onGround){
		if(accelerating) {
			drive();
			turn(dt);
		}

		if(decelerating) {
			if(movingDir > 0) {
				brake();
			}
			else if(!accelerating){
				reverse();
				right *= -1;
				turn(dt);
				right *= -1;
			}
		}

	}


	if(usePhysics) {
		phys.update(dt);
		// Updates centre of mass.
		Vec3f::scale(&phys.centreOfMass, &headDir, 0.5f * scale.z);
		Vec3f::add(&phys.centreOfMass, &phys.centreOfMass, &position);

		phys.sizes = collider->sizes;
		phys.inputForce.set(0.f, 0.f, 0.f);
		phys.inputAngularForce.set(0.f, 0.f, 0.f);
	}


	// Apply turning rotation.
	rotationOther.y += turningAngle;
	turningAngle = 0.f;

	Vec3f::add(&rotationAngles, &rotationOther, &rotationAngles);

	if(useCollision) collider->update(dt);

	// Rotate direction normals.
	headDir.set(0.f, 0.f, -1.f);
	rightDir.set(1.f, 0.f, 0.f);
	upDir.set(0.f, 1.f, 0.f);
	Mat3f rotation;
	Vec3f newDir;
	Mat3f::rotate(&rotation, Vec3f{1.f, 1.f, 1.f}, rotationAngles);
	Mat3f::mult(&newDir, &rotation, &headDir);
	headDir = newDir;
	Mat3f::mult(&newDir, &rotation, &rightDir);
	rightDir = newDir;
	Mat3f::mult(&newDir, &rotation, &upDir);
	upDir = newDir;

	// Change camera angles
	Vec3f offset = Vec3f(0.f, 1.f, 1.f);
	Vec3f dir = headDir;
	dir.negate();
	Vec3f::scale(&offset, &dir, 6.f);
	Vec3f::add(&camPos1, &position, &offset);
	Vec3f::add(&camPos1, &camPos1, &Vec3f(0.f, 2.f, 0.f));
	dir = rightDir;
	Vec3f::scale(&offset, &dir, 2.f);
	Vec3f::add(&camPos2, &position, &offset);

	// Scale, then rotate and finally translate
	Mat4f::scaleVec(&modelMatrix, scale);
	Mat4f::rotate(&modelMatrix, Vec3f{1.f, 1.f, 1.f}, rotationAngles);
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

void Car::setupRendering() {

	setupModelWithoutIndex();

	shaderName = "Lighting";

	ShaderUniform u_Colour;
	u_Colour.name = "u_Colour";
	u_Colour.dataMatrix.setRowData(0, &colour);
	u_Colour.type = UniformType::VEC4;
	ShaderUniform textureSlot;
	textureSlot.name = "u_Texture";
	textureSlot.resourceName = "car-texture";
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

void Car::setupEngine(float drivingForce, float brakingForce) {
	this->drivingForce = drivingForce;
	this->brakingForce = brakingForce;
	this->turningForce = 10.f;
}
