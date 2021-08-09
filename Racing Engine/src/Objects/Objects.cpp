#include "Objects.h"

#include <math.h>
#include "Maths/WorldMaths.h"

GameObject::GameObject() {
}

GameObject::~GameObject() {

}

Model::Model() {
}

Model::~Model() {

}

void Model::deleteHeapAllocation() {
	vb->unBind();
	delete(vb);
	ib->unBind();
	delete(ib);
	va->unBind();
	delete(va);
}

void Model::setupModelWithIndex() {
	va = new VertexArray();

	// Use this buffer
	// Specify the data to use and its layout (x = vertecies * no. of components xyz)
	vb = new VertexBuffer(&vertices[0], vertices.size() * sizeof(objl::Vertex), vertices.size(), GL_STATIC_DRAW);

	// Bind attributes to vertex buffer and array
	VertexBufferLayout layout;
	// Vertex has 8 (3 coordinate 3 normal 2 texture) components and are all floats
	layout.push<float>(3); // position
	layout.push<float>(3); // normal
	layout.push<float>(2); // texCoord
	va->addBuffer((*vb), layout);

	ib = new IndexBuffer(&indices[0], indices.size(), GL_STATIC_DRAW);
	hasIndexBuffer = true;

	va->unBind();
	ib->unBind();
}

void Model::setupModelWithoutIndex() {
	va = new VertexArray();

	// Use this buffer
	// Specify the data to use and its layout (x = vertecies * no. of components xyz)
	vb = new VertexBuffer(&vertices[0], vertices.size() * sizeof(objl::Vertex), vertices.size(), GL_STATIC_DRAW);

	// Bind attributes to vertex buffer and array
	VertexBufferLayout layout;
	// Vertex has 8 (3 coordinate 3 normal 2 texture) components and are all floats
	layout.push<float>(3); // position
	layout.push<float>(3); // normal
	layout.push<float>(2); // texCoord
	va->addBuffer((*vb), layout);

	va->unBind();
	vb->unBind();
}

// Setup class static variables.
Vec3f Physics::gravity = {0.f, -9.81f, 0.f};
unsigned int Physics::integration = 2;

Physics::Physics(Vec3f * pos, Vec3f* rotate) {
	this->position = pos;
	this->rotation = rotate;
}

void Physics::update(double dt) {
	// Allows user to change which integration method is being used to move an object.
	switch(integration) {
	case 1:
		move_Semi_Implicit_Euler(dt);
		break;
	case 2:
		move_Verlet(dt);
		break;
	case 3:
		move_Runge_Kutta2(dt);
		break;
	case 4:
		move_Runge_Kutta4(dt);
		break;
	default:
		move_Semi_Implicit_Euler(dt);
		break;
	}

	// Kinetic energy at the end of this timestep.

	Vec3f velocitySquared;
	float multiplier = sqrtf(velocity.lengthSquare());
	velocitySquared.set(velocity.x * multiplier, velocity.y * multiplier, velocity.z * multiplier);

	Vec3f::scale(&kE, &velocitySquared, mass * 0.5f);

	// Update inertia tensor
	inertiaTensor = calcInertiaTensor();

	// Update rotation
	calcAccAngular(angularVelocity);

	Vec3f::increment(&angularVelocity, &angularAcc, dt * 0.5); // v = u + at / 2
	Vec3f::increment(&angularPos, &angularVelocity, dt); // multiply v by t to get the distance travelled and add to position
	
	// Limit values to within -2pi -> 2pi.
	angularPos.x = fmodf(angularPos.x, 2 * PI_f);
	angularPos.y = fmodf(angularPos.y, 2 * PI_f);
	angularPos.z = fmodf(angularPos.z, 2 * PI_f);

	// Convert back to degrees to rotate object.
	rotation->x = toDegreesf(angularPos.x);
	rotation->y = toDegreesf(angularPos.y);
	rotation->z = toDegreesf(angularPos.z);

	onGround = false;
}

Vec3f Physics::calcGlobalForce(const Vec3f &pos, const Vec3f &vel) {
	Vec3f result;

	// Weight force
	Vec3f weight;
	Vec3f::increment(&weight, &Physics::gravity, mass);

	// Input force
	Vec3f input = inputForce;

	// Air density = 1.225. Assumed surface area = 1.
	float calc = dragCoefficient * 0.5 * 1.225f * 1;

	Vec3f velocitySquared;
	float multiplier = sqrtf(velocity.lengthSquare());
	velocitySquared.set(vel.x * multiplier, vel.y * multiplier, vel.z * multiplier);

	Vec3f::scale(&result, &velocitySquared, calc);
	// Apply drag in opposite direction
	result.negate();
	// Add other forces to drag. Gravity, input.
	Vec3f::add(&result, &result, &weight);
	Vec3f::add(&result, &result, &input);

	// Applied force in opposite direction of friction - frictionForce.
	Vec3f forceAgainstFriction;
	Vec3f movementDir = frictionForce;
	movementDir.normalise();
	movementDir.negate();
	float scalar = Vec3f::dot(&result, &movementDir);
	Vec3f::scale(&forceAgainstFriction, &movementDir, scalar);

	if(forceAgainstFriction.lengthSquare() <= frictionForce.lengthSquare()) {
		Vec3f::sub(&result, &result, &forceAgainstFriction);
		frictionForce.set(0.f, 0.f, 0.f);
		return result;
	}

	Vec3f::add(&result, &result, &frictionForce);
	frictionForce.set(0.f, 0.f, 0.f);

	return result;
}


Vec3f Physics::calcFriction(Physics &b, const Vec3f &normal) {
	if(frictionForce.lengthSquare() == 0) {
		frictionForce.set(FLT_MAX, FLT_MAX, FLT_MAX);
	}

	Vec3f friction = velocity;
	Vec3f normalForce;
	float coefficent = b.frictionCoeff_static;

	// Friction in the opposite direction of velocity.
	friction.x *= (1 - normal.x);
	friction.y *= (1 - normal.y);
	friction.z *= (1 - normal.z);
	friction.normalise();
	friction.negate();

	Vec3f::scale(&normalForce, &gravity, mass);
	float incline = Vec3f::dot(&normalForce, &normal);
	incline /= sqrtf(normalForce.lengthSquare());

	Vec3f::scale(&normalForce, &normalForce, incline);
	// Use kinetic friction if the object is moving along ground. (Friction is currently velocity on ground).
	if(friction.lengthSquare() > 0.f) coefficent = b.frictionCoeff_dynamic;

	float force = sqrtf(normalForce.lengthSquare()) * coefficent;

	Vec3f::scale(&friction, &friction, force);

	// Use the lowest friction value.
	if(friction.lengthSquare() < frictionForce.lengthSquare()) {
		frictionForce = friction;
		return friction;
	}

	return friction;
}

void Physics::incrementForce(Vec3f force) {
}

void Physics::calcAcc(const Vec3f &pos, const Vec3f &vel) {
	Vec3f globalForce = calcGlobalForce(pos, vel);

	// a = F / m.
	if(mass != 0) {
		Vec3f::scale(&acc, &globalForce, 1 / mass);
	}
}

void Physics::calcAccAngular(const Vec3f & angularVel) {
	Vec3f drag;
	Vec3f result;

	// Air density = 1.225. Assumed surface area = 1.
	float calc = dragCoefficient * 0.5 * 1.225f * 1;

	Vec3f velocitySquared;
	float multiplier = sqrtf(angularVel.lengthSquare());
	velocitySquared.set(angularVelocity.x * multiplier, angularVelocity.y * multiplier, angularVel.z * multiplier);

	Vec3f::scale(&drag, &velocitySquared, calc);
	// Apply drag in opposite direction
	drag.negate();

	// Total up forces.
	Vec3f::add(&result, &drag, &inputAngularForce);

	// Divide by moment of inertia.
	Mat3f inverseInertia;
	Mat3f::inverse(&inverseInertia, &inertiaTensor);
	Mat3f::mult(&angularAcc, &inverseInertia, &result);

}

void Physics::move_Semi_Implicit_Euler(const double dt) {
	// Updates acceleration.
	calcAcc(*position, velocity);

	Vec3f::increment(&velocity, &acc, dt * 0.5); // v = u + at / 2
	Vec3f::increment(position, &velocity, dt); // multiply v by t to get the distance travelled and add to position
}

void Physics::move_Verlet(const double dt) {
	calcAcc(*position, velocity);
	Vec3f initialAcc = acc;

	// Calculate position.
	Vec3f temp = *position;
	Vec3f::increment(&temp, &velocity, dt);
	Vec3f::increment(&temp, &acc, dt * dt * 0.5);
	*position = temp;

	// Acceleration at next timestep.
	calcAcc(*position, velocity);

	// Calculate velocity.
	Vec3f::add(&temp, &initialAcc, &acc);
	Vec3f::increment(&velocity, &temp, dt * 0.5);

}

void Physics::move_Runge_Kutta2(const double dt) {
	Vec3f initialPos = *position;
	Vec3f initialVel = velocity;
	calcAcc(initialPos, initialVel);
	Vec3f initialAcc = acc;

	Vec3f temp;

	// Calculate second order
	Vec3f::increment(position, &initialVel, dt);
	Vec3f::increment(&velocity, &initialAcc, dt);
	calcAcc(*position, velocity);

	// Average order and update position.
	Vec3f::add(&temp, &initialVel, &velocity);
	Vec3f::scale(&temp, &temp, dt * 0.5f);
	Vec3f::add(position, &initialPos, &temp);

	// Average orders and update velocity.
	Vec3f::add(&temp, &initialAcc, &acc);
	Vec3f::scale(&temp, &temp, dt * 0.5f);
	Vec3f::add(&velocity, &initialVel, &temp);
}

void Physics::move_Runge_Kutta4(const double dt) {
	Vec3f initialPos = *position;
	Vec3f initialVel = velocity;
	calcAcc(initialPos, initialVel);
	Vec3f initialAcc = acc;

	Vec3f secondPos;
	Vec3f secondVel;
	Vec3f secondAcc;

	Vec3f thirdPos;
	Vec3f thirdVel;
	Vec3f thirdAcc;

	Vec3f temp;

	// Calculate second order
	Vec3f::increment(position, &initialVel, dt * 0.5f);
	secondPos = *position;
	Vec3f::increment(&velocity, &initialAcc, dt * 0.5f);
	secondVel = velocity;
	calcAcc(secondPos, secondVel);
	secondAcc = acc;

	// Calculate third order
	Vec3f::scale(&temp, &secondVel, dt * 0.5f);
	Vec3f::add(&thirdPos, &initialPos, &temp);
	Vec3f::scale(&temp, &secondAcc, dt * 0.5f);
	Vec3f::add(&thirdVel, &initialVel, &temp);
	calcAcc(thirdPos, thirdVel);
	thirdAcc = acc;

	//Calculate fourth order
	Vec3f::scale(&temp, &thirdVel, dt);
	Vec3f::add(position, &initialPos, &temp);
	Vec3f::scale(&temp, &thirdAcc, dt);
	Vec3f::add(&velocity, &initialVel, &temp);
	calcAcc(*position, velocity);

	// Average order and update position.
	// X(n+1) = X(n) + (v1 + 2*v2 + 2*v3 + v4) / 6 * dt
	Vec3f::add(&temp, &initialVel, &velocity);
	Vec3f::scale(&secondVel, &secondVel, 2.f);
	Vec3f::scale(&thirdVel, &thirdVel, 2.f);
	Vec3f::add(&temp, &temp, &secondVel);
	Vec3f::add(&temp, &temp, &thirdVel);
	Vec3f::scale(&temp, &temp, dt * 0.16666667f);
	Vec3f::add(position, &initialPos, &temp);

	// Average order and update position.
	// V(n+1) = V(n) + (a1 + 2*a2 + 2*a3 + a4) / 6 * dt
	Vec3f::add(&temp, &initialAcc, &acc);
	Vec3f::scale(&secondAcc, &secondAcc, 2.f);
	Vec3f::scale(&thirdAcc, &thirdAcc, 2.f);
	Vec3f::add(&temp, &temp, &secondAcc);
	Vec3f::add(&temp, &temp, &thirdAcc);
	Vec3f::scale(&temp, &temp, dt * 0.16666667f);
	Vec3f::add(&velocity, &initialVel, &temp);
}

Vec3f Physics::getNextPosition(const double dt) {
	Vec3f nextPosition;
	Vec3f currentPosition = Vec3f(position);
	Vec3f currentVelocity = velocity;

	// Use the specific integeration method for this object.
	switch(integration) {
	case 1:
		move_Semi_Implicit_Euler(dt);
		break;
	case 2:
		move_Verlet(dt);
		break;
	case 3:
		move_Runge_Kutta2(dt);
		break;
	case 4:
		move_Runge_Kutta4(dt);
		break;
	default:
		move_Semi_Implicit_Euler(dt);
		break;
	}

	nextPosition = Vec3f(position);

	// Set values back to current timestep.
	position->set(currentPosition.x, currentPosition.y, currentPosition.z);
	velocity = currentVelocity;

	return nextPosition;
}

Mat3f Physics::calcInertiaTensor() {
	Mat3f inertia;
	if(mass == 0) return inertia;

	float xx = mass * (sizes.y * sizes.y + sizes.z * sizes.z);
	float yy = mass * (sizes.x * sizes.x + sizes.z * sizes.z);
	float zz = mass * (sizes.x * sizes.x + sizes.y * sizes.y);

	float xy = -mass * sizes.x * sizes.y;
	float xz = -mass * sizes.x * sizes.z;
	float yz = -mass * sizes.y * sizes.z;

	inertia.matrix[0][0] = xx;
	inertia.matrix[1][1] = yy;
	inertia.matrix[2][2] = zz;

	inertia.matrix[0][1] = xy;
	inertia.matrix[0][2] = xz;

	inertia.matrix[1][0] = xy;
	inertia.matrix[1][2] = yz;

	inertia.matrix[2][0] = xz;
	inertia.matrix[2][1] = yz;

	return inertia;
}

bool Physics::isGround() {
	return false;
}

void Physics::applyImpulse(Physics &physB, bool usePhysB, const ContactManifold &manifold) {
	Vec3f relativeVelocity;
	float restitution = restitutionCoeff * physB.restitutionCoeff;
	Vec3f temp1;
	Vec3f temp2;
	Vec3f velocityCollisionA;
	Vec3f velocityCollisionB;
	Vec3f CollisionToMassA;
	Vec3f CollisionToMassB;
	Mat3f inverseInertiaA = calcInertiaTensor();
	Mat3f inverseInertiaB = physB.calcInertiaTensor();
	Mat3f::inverse(&inverseInertiaA, &inverseInertiaA);
	Mat3f::inverse(&inverseInertiaB, &inverseInertiaB);

	Vec3f::sub(&CollisionToMassA, position, &(manifold.points[0].position));
	Vec3f::sub(&CollisionToMassB, physB.position, &(manifold.points[0].position));

	Vec3f::cross(&temp1, &angularVelocity, &CollisionToMassA);
	Vec3f::add(&velocityCollisionA, &velocity, &temp1);

	Vec3f::cross(&temp1, &physB.angularVelocity, &CollisionToMassB);
	Vec3f::add(&velocityCollisionB, &physB.velocity, &temp1);

	Vec3f::sub(&relativeVelocity, &velocityCollisionA, &velocityCollisionB);

	float totalVelocity = Vec3f::dot(&relativeVelocity, &manifold.normal);
	totalVelocity *= -1 - restitution;

	Vec3f::cross(&temp1, &CollisionToMassA, &manifold.normal);
	Mat3f::mult(&temp2, &inverseInertiaA, &temp1);
	Vec3f::cross(&temp2, &temp2, &CollisionToMassA);
	float overallMass = Vec3f::dot(&manifold.normal, &temp2);

	Vec3f::cross(&temp1, &CollisionToMassB, &manifold.normal);
	Mat3f::mult(&temp2, &inverseInertiaB, &temp1);
	Vec3f::cross(&temp2, &temp2, &CollisionToMassB);
	overallMass += Vec3f::dot(&manifold.normal, &temp2);

	overallMass += (1 / mass) + (1 / physB.mass);

	float impulse = totalVelocity / overallMass;

	// New linear velocity after collision.
	Vec3f::scale(&temp1, &manifold.normal, impulse / mass);
	Vec3f::add(&velocity, &velocity, &temp1);

	// New Angular velocity after collsion.
	Vec3f::scale(&temp1, &manifold.normal, impulse);
	Vec3f::cross(&temp1, &CollisionToMassA, &temp1);
	Mat3f::mult(&temp2, &inverseInertiaA, &temp1);
	Vec3f::add(&angularVelocity, &angularVelocity, &temp2);

	Vec3f debug1;
	Vec3f debug2;

	// New linear velocity after collision.
	Vec3f::scale(&temp1, &manifold.normal, impulse / mass);
	debug1 = temp1;
	//velocity = debug1;

	// New Angular velocity after collsion.
	Vec3f::scale(&temp1, &manifold.normal, impulse);
	Vec3f::cross(&temp1, &CollisionToMassA, &temp1);
	Mat3f::mult(&temp2, &inverseInertiaA, &temp1);
	debug2 = temp2;
	//angularVelocity = debug2;

	if(usePhysB) {
		// New linear velocity after collision.
		Vec3f::scale(&temp1, &manifold.normal, impulse / mass);
		Vec3f::sub(&physB.velocity, &physB.velocity, &temp1);
		temp1.negate();
		//physB.velocity = temp1;

		// New Angular velocity after collsion.
		Vec3f::scale(&temp1, &manifold.normal, impulse);
		Vec3f::cross(&temp1, &CollisionToMassB, &temp1);
		Mat3f::mult(&temp2, &inverseInertiaB, &temp1);
		Vec3f::sub(&angularVelocity, &angularVelocity, &temp2);
		temp2.negate();
		//physB.angularVelocity = temp2;
	}

}

void Physics::displace(const ContactManifold & manifold) {
	Vec3f displacement;
	Vec3f::scale(&displacement, &manifold.normal, manifold.points[0].penetration + 0.001f);
	Vec3f::add(position, position, &displacement);
	Vec3f::add(&centreOfMass, &centreOfMass, &displacement);
}

BoxCollider::BoxCollider(Model * parent) {
	this->parent = parent;
	position = parent->position;
	rotation = parent->rotationAngles;
	setScale(parent->scale);
}

BoxCollider::~BoxCollider() {
}

void BoxCollider::init() {
	vertices[0] = {position.x - sizes.x, position.y - sizes.y, position.z + sizes.z};
	vertices[1] = {position.x + sizes.x, position.y - sizes.y, position.z + sizes.z};
	vertices[2] = {position.x + sizes.x, position.y + sizes.y, position.z + sizes.z}; // Max point
	vertices[3] = {position.x - sizes.x, position.y + sizes.y, position.z + sizes.z};
	vertices[4] = {position.x - sizes.x, position.y - sizes.y, position.z - sizes.z}; // Min point
	vertices[5] = {position.x + sizes.x, position.y - sizes.y, position.z - sizes.z};
	vertices[6] = {position.x + sizes.x, position.y + sizes.y, position.z - sizes.z};
	vertices[7] = {position.x - sizes.x, position.y + sizes.y, position.z - sizes.z};

	for(int i = 0; i < 8; i++) {
		Vec3f::add(vertices + i, vertices + i, &posOffset);
	}

	faceNormals[0] = {1.f, 0.f, 0.f};
	faceNormals[1] = {0.f, 1.f, 0.f};
	faceNormals[2] = {0.f, 0.f, 1.f};

	if(aabb) {
		return;
	}

	Mat3f rotation;
	Mat3f::rotate(&rotation, {1.f, 1.f, 1.f}, this->rotation);
	Vec3f temp;

	// Orient vertices based on parent's rotation.
	for(int i = 0; i < 8; i++) {
		Mat3f::mult(&temp, &rotation, vertices + i);
		vertices[i] = temp;
	}

	// Orient face normals.
	Mat3f::mult(&temp, &rotation, faceNormals + 0);
	faceNormals[0] = temp;
	Mat3f::mult(&temp, &rotation, faceNormals + 1);
	faceNormals[1] = temp;
	Mat3f::mult(&temp, &rotation, faceNormals + 2);
	faceNormals[2] = temp;

}

void BoxCollider::update(double dt) {
	position = parent->position;
	rotation = parent->rotationAngles;

	vertices[0] = {position.x - sizes.x, position.y - sizes.y, position.z + sizes.z};
	vertices[1] = {position.x + sizes.x, position.y - sizes.y, position.z + sizes.z};
	vertices[2] = {position.x + sizes.x, position.y + sizes.y, position.z + sizes.z}; // Max point
	vertices[3] = {position.x - sizes.x, position.y + sizes.y, position.z + sizes.z};
	vertices[4] = {position.x - sizes.x, position.y - sizes.y, position.z - sizes.z}; // Min point
	vertices[5] = {position.x + sizes.x, position.y - sizes.y, position.z - sizes.z};
	vertices[6] = {position.x + sizes.x, position.y + sizes.y, position.z - sizes.z};
	vertices[7] = {position.x - sizes.x, position.y + sizes.y, position.z - sizes.z};

	for(int i = 0; i < 8; i++) {
		Vec3f::add(vertices + i, vertices + i, &posOffset);
	}

	faceNormals[0] = {1.f, 0.f, 0.f};
	faceNormals[1] = {0.f, 1.f, 0.f};
	faceNormals[2] = {0.f, 0.f, 1.f};

	if(!aabb) {
		Mat3f rotation;
		Mat3f::rotate(&rotation, {1.f, 1.f, 1.f}, this->rotation);
		Vec3f temp;

		// Orient vertices based on parent's rotation.
		for(int i = 0; i < 8; i++) {
			Vec3f posToVertex;
			Vec3f result;
			Vec3f::sub(&posToVertex, vertices + i, &position);
			Mat3f::mult(&result, &rotation, &posToVertex);
			Vec3f::add(&result, &position, &result);
			vertices[i] = result;
		}

		// Orient face normals.
		Mat3f::mult(&temp, &rotation, faceNormals + 0);
		faceNormals[0] = temp;
		Mat3f::mult(&temp, &rotation, faceNormals + 1);
		faceNormals[1] = temp;
		Mat3f::mult(&temp, &rotation, faceNormals + 2);
		faceNormals[2] = temp;
	}

}

void BoxCollider::setAABB(bool aabb) {
	this->aabb = aabb;
}

Vec3f BoxCollider::closestPointTo(const Vec3f & point) {
	Vec3f posToPoint;
	Vec3f::sub(&posToPoint, &point, &position);
	const Vec3f *basis = getNormals();

	float distX = Vec3f::dot(&posToPoint, basis + 0);
	float distY = Vec3f::dot(&posToPoint, basis + 1);
	float distZ = Vec3f::dot(&posToPoint, basis + 2);

	// Clamp values
	if(distX > sizes.x) distX = sizes.x;
	else if(distX < -sizes.x) distX = -sizes.x;

	if(distY > sizes.y) distY = sizes.y;
	else if(distY < -sizes.y) distY = -sizes.y;

	if(distZ > sizes.z) distZ = sizes.z;
	else if(distZ < -sizes.z) distZ = -sizes.z;

	Vec3f contactPoint = position;
	Vec3f::increment(&contactPoint, basis + 0, distX);
	Vec3f::increment(&contactPoint, basis + 1, distY);
	Vec3f::increment(&contactPoint, basis + 2, distZ);

	return contactPoint;
}

Vec3f BoxCollider::getNormalFromPoint(const Vec3f & point) {
	Vec3f normal;
	Vec3f posToPoint;
	float min = FLT_MAX;
	float distance;
	Vec3f::sub(&posToPoint, &point, &position);

	distance = fabsf(sizes.x - fabsf(posToPoint.x));

	if(distance < min) {
		min = distance;
		Vec3f::scale(&normal, &Vec3f(1.f, 0.f, 0.f), !signbit(posToPoint.x));
	}

	distance = fabsf(sizes.y - fabsf(posToPoint.y));

	if(distance < min) {
		min = distance;
		Vec3f::scale(&normal, &Vec3f(0.f, 1.f, 0.f), !signbit(posToPoint.y));
	}

	distance = fabsf(sizes.z - fabsf(posToPoint.z));

	if(distance < min) {
		min = distance;
		Vec3f::scale(&normal, &Vec3f(0.f, 0.f, 1.f), !signbit(posToPoint.z));
	}

	return normal;
}

SphereCollider::SphereCollider(Model * parent) {
	this->parent = parent;
	position = parent->position;
	rotation = parent->rotationAngles;
	float largestSize = fmaxf(scale.x, scale.y);
	float largeSize = fmaxf(scale.z, largestSize);
	setRadius(largestSize);
	scale.set(1.f, 1.f, 1.f);
}

SphereCollider::~SphereCollider() {
}

void SphereCollider::init() {
	position = parent->position;
	Vec3f::add(&position, &position, &posOffset);
}

void SphereCollider::update(double dt) {
	position = parent->position;
	Vec3f::add(&position, &position, &posOffset);
	rotation = parent->rotationAngles;
}

void SphereCollider::setRadius(float radius) {
	this->radius = radius;
	sizes.set(radius, radius, radius);
	scale.set(radius, radius, radius);
}

void SphereCollider::setScale(Vec3f scale) {
	this->radius = scale.x;
	sizes.set(radius, radius, radius);
}

PlaneCollider::PlaneCollider(Model * parent) {
	this->parent = parent;
	position = parent->position;
	sizes.set(0.5f, 0.5f, 0.5f);
	scale.set(1.f, 1.f, 1.f);
}

PlaneCollider::~PlaneCollider() {
}

void PlaneCollider::init() {
	position = parent->position;
	rotation = parent->rotationAngles;

	vertices[0] = {position.x - sizes.x, position.y - sizes.y, position.z + sizes.z}; // Min vertex
	vertices[1] = {position.x + sizes.x, position.y - sizes.y, position.z + sizes.z};
	vertices[2] = {position.x + sizes.x, position.y + sizes.y, position.z + sizes.z}; // Max vertex
	vertices[3] = {position.x - sizes.x, position.y + sizes.y, position.z + sizes.z};

	for(int i = 0; i < 4; i++) {
		Vec3f::add(vertices + i, vertices + i, &posOffset);
	}

	normal = {0.f, 1.f, 0.f};

	Mat3f rotation;
	Mat3f::rotate(&rotation, {1.f, 1.f, 1.f}, this->rotation);
	Vec3f temp;

	// Orient vertices based on parent's rotation.
	for(int i = 0; i < 4; i++) {
		Mat3f::mult(&temp, &rotation, vertices + i);
		vertices[i] = temp;
	}

	// Orient face normals.
	Mat3f::mult(&temp, &rotation, &normal);
	normal = temp;
}

void PlaneCollider::update(double dt) {
	position = parent->position;
	rotation = parent->rotationAngles;

	vertices[0] = {position.x - sizes.x, position.y - sizes.y, position.z + sizes.z}; // Min vertex
	vertices[1] = {position.x + sizes.x, position.y - sizes.y, position.z + sizes.z};
	vertices[2] = {position.x + sizes.x, position.y + sizes.y, position.z + sizes.z}; // Max vertex
	vertices[3] = {position.x - sizes.x, position.y + sizes.y, position.z + sizes.z};

	for(int i = 0; i < 4; i++) {
		Vec3f::add(vertices + i, vertices + i, &posOffset);
	}

	Mat3f rotation;
	Mat3f::rotate(&rotation, {1.f, 1.f, 1.f}, this->rotation);
	Vec3f temp;

	// Orient vertices based on parent's rotation.
	for(int i = 0; i < 4; i++) {
		Mat3f::mult(&temp, &rotation, vertices + i);
		vertices[i] = temp;
	}

	// Orient face normals.
	Mat3f::mult(&temp, &rotation, &normal);
	normal = temp;
}

