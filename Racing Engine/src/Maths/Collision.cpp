#include "Collision.h"

#include <math.h>
#include <float.h>

bool CollisionDetect::satTest(const Vec3f vertA[], const Vec3f vertB[], Vec3f axis, int length, float &depth, Vec3f &normal) {
	// https://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat

	Vec3f temp;

	// There is no seperating axis perpendicular to a seperating plane.
	if(axis.compare(temp)) {
		return true;
	}

	float aMin = FLT_MAX;
	float aMax = -FLT_MAX;
	float bMin = FLT_MAX;
	float bMax = -FLT_MAX;

	// Calculate maximum and minimum distance of the object projected onto the axis.
	for(int i = 0; i < length; i++) {
		float aDist = Vec3f::dot(vertA + i, &axis);
		aMin = (aDist < aMin) ? aDist : aMin;
		aMax = (aDist > aMax) ? aDist : aMax;

		float bDist = Vec3f::dot(vertB + i, &axis);
		bMin = (bDist < bMin) ? bDist : bMin;
		bMax = (bDist > bMax) ? bDist : bMax;

	}

	float longSpan = std::fmaxf(aMax, bMax) - std::fminf(aMin, bMin);
	float sumSpan = aMax - aMin + bMax - bMin;
	float overlap = sumSpan - longSpan;

	// Points are overlapping on this axis.
	if(overlap >= 0) {
		if(depth > overlap) {
			depth = overlap;
			normal = axis;
		}
		return true;
	}

	// Does not overlap
	return false;
}

bool CollisionDetect::broadCheck(const Vec3f &posA, const Vec3f &sizeA, const Vec3f &posB, const Vec3f &sizeB) {
	bool xAxis = posA.x + sizeA.x >= posB.x - sizeB.x && posA.x - sizeA.x <= posB.x + sizeB.x;

	bool yAxis = posA.y + sizeA.y >= posB.y - sizeB.y && posA.y - sizeA.y <= posB.y + sizeB.y;

	bool zAxis = posA.z + sizeA.z >= posB.z - sizeB.z && posA.z - sizeA.z <= posB.z + sizeB.z;

	return xAxis && yAxis && zAxis;
}

bool CollisionDetect::boxBox(BoxCollider* a, BoxCollider* b) {

	bool axis_aligned_A = a->isAABB();
	bool axis_aligned_B = b->isAABB();

	// Both aabb
	if(axis_aligned_A && axis_aligned_B) {
		bool collision = (a->maxPoint->x >= b->minPoint->x && a->minPoint->x <= b->maxPoint->x) &&
			(a->maxPoint->y >= b->minPoint->y && a->minPoint->y <= b->maxPoint->y) &&
			(a->maxPoint->z >= b->minPoint->z && a->minPoint->z <= b->maxPoint->z);

		if(collision) {
			Vec3f contactPointInA;
			Vec3f contactPointInB;

			contactPointInA.set(fmaxf(b->minPoint->x, fminf(a->position.x, b->maxPoint->x)),
				fmaxf(b->minPoint->y, fminf(a->position.y, b->maxPoint->y)),
				fmaxf(b->minPoint->z, fminf(a->position.z, b->maxPoint->z)));

			/*contactPointInB.set(fmaxf(a->minPoint->x, fminf(b->position.x, a->maxPoint->x)),
				fmaxf(a->minPoint->y, fminf(b->position.y, a->maxPoint->y)),
				fmaxf(a->minPoint->z, fminf(b->position.z, a->maxPoint->z)));

			Vec3f contactBA;
			Vec3f::sub(&contactBA, &contactPointInA, &contactPointInB);
			float penetrationDepth = sqrtf(contactBA.lengthSquare());*/

			Vec3f normal = b->getNormalFromPoint(contactPointInA);

			Vec3f posAToContactA;
			Vec3f distance;
			distance.set(a->sizes.x * normal.x, a->sizes.y * normal.y, a->sizes.z * normal.z);
			Vec3f::sub(&posAToContactA, &contactPointInA, &a->position);
			float penetrationDepth = sqrtf(distance.lengthSquare()) - sqrtf(posAToContactA.lengthSquare());

			ContactManifold manifold;
			manifold.normal = normal;
			manifold.points[0] = {contactPointInA, penetrationDepth};
			manifold.pointCount = 1;
			a->contacts.push_back(manifold);

			return true;
		}

		return false;
	}

	// At least 1 object is obb.

	// https://www.jkh.me/files/tutorials/Separating%20Axis%20Theorem%20for%20Oriented%20Bounding%20Boxes.pdf

	const Vec3f *normalAxisA = a->getNormals();
	const Vec3f *normalAxisB = b->getNormals();

	// Calculate closest points and check if there is any overlap. End check early and set points.

	Vec3f contactPointInA = b->closestPointTo(a->position);
	float penetrationDepth = FLT_MAX;
	Vec3f normal;

	/*Vec3f contactPointInB = a->closestPointTo(b->position);
	Vec3f contactBA;
	Vec3f::sub(&contactBA, &contactPointInA, &contactPointInB);
	float penetrationDepth = sqrtf(contactBA.lengthSquare());*/

	for(int i = 0; i < 3; i++) {
		if(!satTest(a->vertices, b->vertices, normalAxisA[i], 8, penetrationDepth, normal)) {
			return false;
		}

		if(!satTest(a->vertices, b->vertices, normalAxisB[i], 8, penetrationDepth, normal)) {
			return false;
		}

		// Check if the edges are colliding.
		Vec3f cross;
		for(int j = 0; j < 3; j++) {
			Vec3f::cross(&cross, normalAxisA + i, normalAxisB + j);
			if (!satTest(a->vertices, b->vertices, &cross, 8, penetrationDepth, normal)) {
				return false;
			}
		}
	}

	// Overlap detected on all axis after checking everything.
	normal.negate();
	ContactManifold manifold;
	manifold.normal = normal;
	manifold.points[0] = {contactPointInA, penetrationDepth};
	manifold.pointCount = 1;
	a->contacts.push_back(manifold);
	return true;
}

bool CollisionDetect::sphereSphere(SphereCollider* a, SphereCollider* b) {
	// Detect collision.

	Vec3f AB;
	Vec3f::sub(&AB, &b->position, &a->position);
	float distanceAB = sqrtf(AB.lengthSquare());
	float penetrationDepth = a->radius + b->radius - distanceAB;

	// Find contact point.
	if(penetrationDepth >= 0) {

		Vec3f contactPoint;
		float multiplier = (a->radius - penetrationDepth) / a->radius;
		Vec3f::scale(&contactPoint, &AB, multiplier);
		Vec3f::add(&contactPoint, &contactPoint, &a->position);

		// Normal is towards the first object
		Vec3f contactNormal = AB;
		contactNormal.normalise();

		ContactManifold manifold;
		manifold.normal = contactNormal;
		manifold.points[0] = {contactPoint, penetrationDepth};
		manifold.pointCount = 1;
		a->contacts.push_back(manifold);

		return true;
	}

	return false;
}

bool CollisionDetect::planePlane(PlaneCollider* a, PlaneCollider* b) {
	float penetrationDepth = FLT_MAX;
	Vec3f normal;
	if(!satTest(a->vertices, b->vertices, b->normal, 4, penetrationDepth, normal)) {
		return false;
	}

		Vec3f BA;
		Vec3f::sub(&BA, &a->position, &b->position);
		Vec3f::scale(&BA, &BA, 0.5f);
		Vec3f contact;
		Vec3f::add(&contact, &b->position, &BA);

		ContactManifold manifold;
		manifold.normal = normal;
		manifold.points[0] = {contact, penetrationDepth};
		manifold.pointCount = 1;
		a->contacts.push_back(manifold);

	return true;
}

bool CollisionDetect::boxSphere(BoxCollider* a, SphereCollider* b) {
	if(a->isAABB()) {
		// Closest point along the box.
		Vec3f closestPoint;
		closestPoint.set(fmaxf(a->minPoint->x, fminf(b->position.x, a->maxPoint->x)),
				fmaxf(a->minPoint->y, fminf(b->position.y, a->maxPoint->y)),
				fmaxf(a->minPoint->z, fminf(b->position.z, a->maxPoint->z)));

		Vec3f AB;
		Vec3f::sub(&AB, &closestPoint, &b->position);
		float penetrationDepth = b->radius - sqrtf(AB.lengthSquare());

		if(penetrationDepth >= 0) {
			Vec3f contactNormal = AB;
			contactNormal.negate();
			contactNormal.normalise();

			ContactManifold manifold;
			manifold.normal = contactNormal;
			manifold.points[0] = {closestPoint, penetrationDepth};
			manifold.pointCount = 1;
			a->contacts.push_back(manifold);

			return true;
		}

		return false;
	}

	// Box is OBB
	// https://gamedev.stackexchange.com/questions/163873/separating-axis-theorem-obb-vs-sphere

	Vec3f boxToSphere;
	Vec3f newSpherePos;
	Mat3f rotationMatrix;

	// Orient the sphere position so that is axis-aligned relative to the box.
	Vec3f::sub(&boxToSphere, &b->position, &a->position);
	Mat3f::rotate(&rotationMatrix, Vec3f{1.f, 1.f, 1.f}, a->rotation);
	Mat3f::mult(&newSpherePos, &rotationMatrix, &boxToSphere);
	Vec3f::add(&newSpherePos, &a->position, &newSpherePos);

	// Now solve as if box is axis aligned using the new sphere position.
	Vec3f closestPoint;
	closestPoint.set(fmaxf(a->minPoint->x, fminf(newSpherePos.x, a->maxPoint->x)),
		fmaxf(a->minPoint->y, fminf(newSpherePos.y, a->maxPoint->y)),
		fmaxf(a->minPoint->z, fminf(newSpherePos.z, a->maxPoint->z)));

	Vec3f sphereToPoint;
	Vec3f::sub(&sphereToPoint, &closestPoint, &b->position);
	float penetrationDepth = b->radius - sqrtf(sphereToPoint.lengthSquare());

	Vec3f normal = a->getNormalFromPoint(closestPoint);

	if(penetrationDepth >= 0) {
		Vec3f contactNormal = sphereToPoint;
		contactNormal.normalise();

		ContactManifold manifold;
		manifold.normal = contactNormal;
		manifold.points[0] = {closestPoint, penetrationDepth};
		manifold.pointCount = 1;
		a->contacts.push_back(manifold);

		return true;
	}


	return false;
}

// https://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?page=7

bool CollisionDetect::boxPlane(BoxCollider* a, PlaneCollider* b) {
	const Vec3f* normals = a->getNormals();
	float dotX = Vec3f::dot(normals + 0, &b->normal);
	float dotY = Vec3f::dot(normals + 1, &b->normal);
	float dotZ = Vec3f::dot(normals + 2, &b->normal);

	Vec3f closestPoint = Vec3f(a->sizes.x * dotX, a->sizes.y * dotY, a->sizes.z * dotZ);
	Vec3f::add(&closestPoint, &a->position, &closestPoint);

	Vec3f planeToBox;
	Vec3f::sub(&planeToBox, &a->position, &b->position);
	float distance = Vec3f::dot(&b->normal, &planeToBox);
	distance = fabsf(distance);
	planeToBox.normalise();

	float penetrationDepth = (a->sizes.x * dotX + a->sizes.y * dotY + a->sizes.z * dotZ) - distance;

	if(penetrationDepth >= 0) {
		ContactManifold manifold;
		manifold.normal = planeToBox;
		manifold.points[0] = {closestPoint, penetrationDepth};
		manifold.pointCount = 1;
		a->contacts.push_back(manifold);
		return true;
	}

	return false;
}

bool CollisionDetect::spherePlane(SphereCollider* a, PlaneCollider* b) {
	Vec3f planeToSphere;
	Vec3f::sub(&planeToSphere, &a->position, &b->position);
	float distance = Vec3f::dot(&b->normal, &planeToSphere);
	distance = fabsf(distance);
	planeToSphere.normalise();

	float penetrationDepth = a->radius - distance;

	Vec3f contactPoint;
	float multiplier = (a->radius - penetrationDepth) / a->radius;
	Vec3f::scale(&contactPoint, &b->normal, multiplier);
	Vec3f::add(&contactPoint, &contactPoint, &b->position);

	if(penetrationDepth >= 0) {
		ContactManifold manifold;
		manifold.normal = planeToSphere;
		manifold.points[0] = {contactPoint, penetrationDepth};
		manifold.pointCount = 1;
		a->contacts.push_back(manifold);
		return true;
	}

	return false;
}
