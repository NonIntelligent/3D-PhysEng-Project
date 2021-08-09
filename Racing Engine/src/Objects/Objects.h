#pragma once
#include <vector>
#include <string>
#include <unordered_map>

#include "Maths/WorldMaths.h"
#include "Graphics/VertexObjects.h"
#include "Graphics/Shader.h"
#include "Graphics/Texture.h"

#include "Vendor/OBJ_Loader.h"

struct ContactPoint;
struct ContactManifold;
class Collision;
class PlaneCollider;
class BoxCollider;
class SphereCollider;

// https://learnopengl.com/Model-Loading/Mesh

// Objects that require physics should instantiate this object
class Physics {
private:

	Vec3f kE;
public:
	// Position and direction data

	Vec3f* position; // points to object position
	Vec3f* rotation; // Rotation of parent
	Vec3f sizes; // Used to calculate inerta tensor
	Vec3f velocity;
	Vec3f acc;

	Vec3f xAxis;
	Vec3f yAxis;
	Vec3f zAxis;

	Vec3f angularVelocity; // Rads
	Vec3f angularAcc; // Rads
	Vec3f angularPos; // Rads
	// Total amount of forces acting on this object.

	// Input force into the system. Caused by divine intervention.
	Vec3f inputForce;
	Vec3f inputAngularForce;
	Vec3f frictionForce;
	Vec3f currentForce;

	Vec3f centreOfMass;
	Mat3f inertiaTensor;
	float mass = 1.f; // mass in Kg

	static Vec3f gravity;
	Vec3f currentDrag;
	float dragCoefficient = 0.36;

	float frictionCoeff_static = 1.f;
	float frictionCoeff_dynamic = 0.5f;
	float restitutionCoeff = 0.4f;

	// Which integration method to use for updating position.
	// 1 = semi-implicit Euler. 2 = Verlet. 3 = Runge_Kutta 2-order. 4 = Runge_Kutta 4-order.
	static unsigned int integration;

	bool onGround = false;

public:

	Physics() { };
	Physics(Vec3f* pos, Vec3f* rotate);
	~Physics() { };

	void update(double dt);
	
	// Add a force to the currentForce of the object.
	void incrementForce(Vec3f force);

	// Calculate force caused by gravity and drag.
	Vec3f calcGlobalForce(const Vec3f &pos, const Vec3f &vel);

	Vec3f calcFriction(Physics &b, const Vec3f &normal);

	// Update acceleration using given data.
	void calcAcc(const Vec3f &pos, const Vec3f &vel);

	// Update angular acceleration by slowing it down using drag.
	void calcAccAngular(const Vec3f &angularVel);

	/**	
		Change position using semi-implicit Euler integration.
	*
		Update position based on using the velocity at the next time step, resulting in slightly better accuracy than explicit Euler.
	*
		@see move_Verlet For a higher accuray integration method.
		@see move_Runge_Kutta for the highest accuracy intergration method.
	*/
	void move_Semi_Implicit_Euler(const double dt);

	// Higher accuracy than Euler with only a small increase to computation cost.
	// Velocity Verlet calulates both position and velocity.
	void move_Verlet(const double dt);

	// Higher accuracy than Verlet but is much more computationally expensive.
	// Predicts the next values and then corrects them (Second-Order scheme).
	void move_Runge_Kutta2(const double dt);

	// Higher accuracy than Verlet but is much more computationally expensive.
	// Predicts the next values and then corrects them (Fourth-Order scheme).
	void move_Runge_Kutta4(const double dt);

	// Returns the position of this object at the next timeframe without altering any current variables.
	Vec3f getNextPosition(const double dt);

	// https://www.youtube.com/watch?v=gUSQCWVIgrg
	Mat3f calcInertiaTensor();

	bool isGround();

	void applyImpulse(Physics &physB, bool usePhysB, const ContactManifold &manifold);

	void displace(const ContactManifold &manifold);

	inline Vec3f getKineticEnergy() { return kE; };
};

// Generic object class for all objects
class GameObject {
private:
	// Unused. Maybe implemented in the far future.
	unsigned int objectID = 0;

public:
	Vec3f position;
	Vec3f rotationAngles;
	Vec3f scale{1.f, 1.f, 1.f};

	Physics phys;

	bool usePhysics = false;
public:
	virtual void init() = 0;
	virtual void update(double dt) = 0;

	GameObject();
	// Virtual so that deleting a GameObject will instead use the latest child's deconstructor.
	virtual ~GameObject();

	virtual void deleteHeapAllocation() = 0;

	// Set the id of this gameObject. Currently serves no purpose right now.
	inline void setID(unsigned int id) {
		objectID = id;
	};
	inline unsigned int getID() const {
		return objectID;
	};
	
};

// Abstract Model Propagates GameObject virtual methods
class Model : public GameObject {
protected:
	bool hasIndexBuffer = false;
	void setupModelWithIndex();
	void setupModelWithoutIndex();
	virtual void setupRendering() = 0;

public:
	// Can be of any child type
	Collision* collider;
	bool useCollision = false;

	std::vector<objl::Vertex> vertices;
	std::vector<unsigned int> indices;

	VertexArray* va;
	VertexBuffer* vb;
	IndexBuffer* ib;

	Vec3f diffuse;
	Vec3f specular;
	float shininess;

	std::string shaderName;
	std::vector<ShaderUniform> uniforms;

	Mat4f modelMatrix;

public:
	Model();
	~Model();

	void deleteHeapAllocation();

	inline unsigned int isIndexBuffer() const {
		return hasIndexBuffer;
	};

};

struct ContactPoint {
	Vec3f position;
	float penetration;
};

struct ContactManifold {
	int pointCount;
	ContactPoint points[4];
	Vec3f normal;
	bool resolved = false;
};

// https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection

// Abstract class
class Collision {
public:
	Vec3f position;
	Vec3f posOffset;
	// Length, height and depth of the collision area. (From the center point).
	Vec3f sizes;

	Vec3f scale;
	// Rotation in degrees.
	Vec3f rotation;

	std::vector<ContactManifold> contacts;

	Model* parent;

	Mat4f modelMatrix;

	bool ground = false;
	// Still collide withot needing to apply impulse.
	bool solid = true;

public:
	virtual void init() = 0;

	// Updates the position and size of the collision area.
	// Does not do any collision detection or response.
	virtual void update(double dt) = 0;

	inline void setPosOffset(Vec3f offset) { posOffset = offset; };

	inline void setScale(Vec3f scale, Vec3f base = Vec3f(0.5f, 0.5f, 0.5f)) {
		this->scale = scale;
		sizes.set(base.x * scale.x, base.y * scale.y, base.z * scale.z);
	};
};

// 2D Plane of a limited size with the plane defaulting to X-Z axis as length and width.
// Normal is pointing in +Y-axis.
class PlaneCollider : public Collision{
public:
	Vec3f normal;

	Vec3f vertices[4];

public:
	PlaneCollider(Model* parent);
	~PlaneCollider();

	void init();
	void update(double dt);
};

class SphereCollider : public Collision {
public:
	float radius = 0;

public:
	SphereCollider(Model* parent);
	~SphereCollider();

	void init();
	void update(double dt);
	void setRadius(float radius);
	// @Overload. Function will change radius using only the x-component of scale.
	void setScale(Vec3f scale);
};

class BoxCollider : public Collision{
private:
	// Either axis-aligned or object-bound
	bool aabb = true;

	Vec3f faceNormals[3] = 
	{
		// Right
		{1.f, 0.f, 0.f},
		// Top
		{0.f, 1.f, 0.f},
		// Front
		{0.f, 0.f, 1.f}
	};

public:
	// Front vertices then back in counter-clockwise order.
	Vec3f vertices[8];

	// Min and max point only to be used in AABB scenarios.

	const Vec3f* const minPoint = vertices + 4;
	const Vec3f* const maxPoint = vertices + 2;

public:
	BoxCollider(Model* parent);
	~BoxCollider();

	void init();
	void update(double dt);

	inline bool isAABB() const {
		return aabb;
	};

	const Vec3f* getNormals() const{
		return faceNormals;
	};

	inline void setSize(Vec3f size) {
		sizes = size;
	};

	void setAABB(bool aabb);

	Vec3f closestPointTo(const Vec3f &point);

	Vec3f getNormalFromPoint(const Vec3f &point);
};
