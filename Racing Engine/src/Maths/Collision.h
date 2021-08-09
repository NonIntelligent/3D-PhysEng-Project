#pragma once

#include "WorldMaths.h"
#include "Objects/Objects.h"

// https://steamcdn-a.akamaihd.net/apps/valve/2015/DirkGregorius_Contacts.pdf

class CollisionDetect {
private:
	CollisionDetect() {	};

	static bool satTest(const Vec3f vertA[], const Vec3f vertB[], Vec3f axis, int length, float &depth, Vec3f &normal);

public:
	static bool broadCheck(const Vec3f &posA, const Vec3f &sizeA, const Vec3f &posB, const Vec3f &sizeB);

	static bool boxBox(BoxCollider* a, BoxCollider* b);
	static bool sphereSphere(SphereCollider* a, SphereCollider* b);
	static bool planePlane(PlaneCollider* a, PlaneCollider* b);

	static bool boxSphere(BoxCollider* a, SphereCollider* b);
	static bool boxPlane(BoxCollider* a, PlaneCollider* b);
	static bool spherePlane(SphereCollider* a, PlaneCollider* b);
};