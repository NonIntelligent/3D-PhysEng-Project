#pragma once

#include <vector>

#include "GL/glew.h"

#include "Maths/WorldMaths.h"
#include "Objects/Objects.h"


// Abstract class 
class Vehicle : public Model{
public:
	float topSpeed = 20.f; // Meters/s
	float currentSpeed = 0.f;
	float throttle = 0.f; // a percentage from 0.0 to 1.0

	float drivingForce = 0.f;
	float brakingForce = 0.f;
	float turningForce = 0.f;
	bool accelerating = false;
	bool decelerating = false;

	// Front of the vehicle
	Vec3f headDir;
	Vec3f rightDir;
	Vec3f upDir;

	// Turning direction
	Vec3f turningDir;
	float turningSpeed = 1.f;
	float turningAngle = 0.f;
	Vec3f rotationOther;
	int right = 0;

	Vec3f camPos1;
	Vec3f camPos2;
	int currentCamera = 1;

	// Name of the vehicle
	std::string name;

	// Driving force is applied in the direction of the head of the vehicle
	void drive();
	// Strong braking force to slow down car to a stop.
	void brake();
	// Reversing force is applied in the opposite direction of the head
	void reverse();

	void turn(double dt);

	bool isGround();

};

class Car : public Vehicle{
public:

	bool setup = false;

	Vec4f colour;

	Car(std::string name, Vec3f position);
	~Car();

	void init();
	void update(double dt);

private:
	void setupRendering();
	void setupEngine(float drivingForce, float brakingForce);

};