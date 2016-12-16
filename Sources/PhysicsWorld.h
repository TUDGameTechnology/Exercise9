#pragma once

#include <Kore/Math/Vector.h>
#include <Kore/Math/Matrix.h>
#include <Kore/Math/Core.h>
#include "ObjLoader.h"
#include "Collision.h"
#include "PhysicsObject.h"

using namespace Kore;


class PhysicsObject;

// Handles all physically simulated objects.
class PhysicsWorld {
	
	int maxPhysicsObjects;

public:
	
	// The ground plane
	PlaneCollider plane;

	TriangleCollider triangle1;
	TriangleCollider triangle2;

	TriangleMeshCollider meshCollider;

	// null terminated array of PhysicsObject pointers
	PhysicsObject** physicsObjects;

	PhysicsWorld(int inMaxPhysicsObjects = 100);
	
	// Integration step
	void Update(float deltaT);

	// Handle the collisions
	void HandleCollisions(float deltaT);

	// Add an object to be simulated
	void AddObject(PhysicsObject* po);

};
