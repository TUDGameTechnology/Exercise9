#include "pch.h"

#include <Kore/IO/FileReader.h>
#include <Kore/Math/Core.h>
#include <Kore/Math/Random.h>
#include <Kore/System.h>
#include <Kore/Input/Keyboard.h>
#include <Kore/Input/Mouse.h>
#include <Kore/Audio1/Audio.h>
#include <Kore/Graphics1/Image.h>
#include <Kore/Graphics4/Graphics.h>
#include <Kore/Graphics4/PipelineState.h>
#include <Kore/Log.h>

#include "ObjLoader.h"
#include "Collision.h"
#include "PhysicsWorld.h"
#include "PhysicsObject.h"

using namespace Kore;

namespace {
	const int width = 512;
	const int height = 512;
	double startTime;
	Graphics4::Shader* vertexShader;
	Graphics4::Shader* fragmentShader;
	Graphics4::PipelineState* pipeline;

	// controls
	bool left = false;
	bool right = false;
	bool up = false;
	bool down = false;

	// null terminated array of MeshObject pointers
	MeshObject* objects[] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

	// The sound to play for the winning condition
	Sound* winSound;
	
	// Was the sound already played?
	bool playedSound = false;

	// The view projection matrix aka the camera
	mat4 P;
	mat4 View;
	mat4 PV;

	// Camera-related variables
	vec3 cameraPosition;
	vec3 targetCameraPosition;
	vec3 oldCameraPosition;

	vec3 lookAt;
	vec3 targetLookAt;
	vec3 oldLookAt;

	// The sphere and the associated physics object
	MeshObject* sphere;
	PhysicsObject* po;

	PhysicsWorld physics;
	
	// uniform locations - add more as you see fit
	Graphics4::TextureUnit tex;
	Graphics4::ConstantLocation pvLocation;
	Graphics4::ConstantLocation mLocation;

	/************************************************************************/
	/* Task P9.2 - Initialize the box collider                           */
	/************************************************************************/
	BoxCollider boxCollider;

	double lastTime = 0.0;

	void update() {
		double t = System::time() - startTime;
		double deltaT = t - lastTime;
		lastTime = t;

		Kore::Audio2::update();
		
		Graphics4::begin();
		Graphics4::clear(Graphics4::ClearColorFlag | Graphics4::ClearDepthFlag, 0xff9999FF, 1.0f);

		Graphics4::setPipeline(pipeline);

		// set the camera
		targetCameraPosition = physics.physicsObjects[0]->GetPosition();
		targetCameraPosition = targetCameraPosition + vec3(-10, 5, 10);
		vec3 targetLookAt = physics.physicsObjects[0]->GetPosition();

		
		// Interpolate the camera to not follow small physics movements
		float alpha = 0.3f;
		cameraPosition = oldCameraPosition * (1.0f - alpha) + targetCameraPosition * alpha;
		oldCameraPosition = cameraPosition;
		lookAt = oldLookAt * (1.0f - alpha) + targetLookAt * alpha;
		oldLookAt = lookAt;

		// Follow the ball with the camera
		P = mat4::Perspective(60.0f * Kore::pi / 180.0f, (float)width / (float)height, 0.1f, 100);
		View = mat4::lookAt(cameraPosition, lookAt, vec3(0, 1, 0)); 
		PV = P * View;

		Graphics4::setMatrix(pvLocation, PV);

		// Render the mesh objects
		MeshObject** current = &objects[0];
		while (*current != nullptr) {
			// set the model matrix
			Graphics4::setMatrix(mLocation, (*current)->M);
			(*current)->render(tex);
			++current;
		} 

		physics.Update((float) deltaT);
		PhysicsObject** currentP = &physics.physicsObjects[0];
	

		// Handle mouse inputs
		float forceX = 0.0f;
		float forceZ = 0.0f;
		if (up) forceX += 1.0f;
		if (down) forceX -= 1.0f;
		if (left) forceZ -= 1.0f;
		if (right) forceZ += 1.0f;

		// Apply gravity
		vec3 force(forceX, 0.0f, forceZ);
		force = force * 20.0f;
		(*currentP)->ApplyForceToCenter(force);

		// Render the meshes
		while (*currentP != nullptr) {
			(*currentP)->UpdateMatrix();
			Graphics4::setMatrix(mLocation, (*currentP)->Mesh->M);
			(*currentP)->Mesh->render(tex);
			++currentP;
		}


		/************************************************************************/
		/* Task P9.2 - Check the box collider for collision and play a sound    */
		/************************************************************************/
		PhysicsObject* SpherePO = physics.physicsObjects[0];
		bool result = SpherePO->Collider.IntersectsWith(boxCollider);
		if (result && !playedSound) {
			playedSound = true;
			Audio1::play(winSound);
		}
			
		Graphics4::end();
		Graphics4::swapBuffers();
	}

	void SpawnSphere(vec3 Position, vec3 Velocity) {
		PhysicsObject* po = new PhysicsObject();
		po->SetPosition(Position);
		po->Velocity = Velocity;
		po->Collider.radius = 0.5f;
		po->Mass = 5;
		po->Mesh = sphere;
			
		// The impulse should carry the object forward

		po->ApplyImpulse(Velocity);
		physics.AddObject(po);
	}

	void handleKeyEvent(KeyCode code, bool isDown)
	{
		if (code == KeyUp || code == KeyW) {
			up = isDown;
		} else if (code == KeyDown || code == KeyS) {
			down = isDown;
		} else if (code == KeyLeft || code == KeyA) {
			right = isDown;
		} else if (code == KeyRight || code == KeyD) {
			left = isDown;
		}
	}

	void keyDown(KeyCode code) {
		handleKeyEvent(code, true);
	}

	void keyUp(KeyCode code) {
		handleKeyEvent(code, false);
	}

	void mouseMove(int windowId, int x, int y, int movementX, int movementY) {
	}
	
	void mousePress(int windowId, int button, int x, int y) {
	}

	void mouseRelease(int windowId, int button, int x, int y) {
	}

	void init() {
		FileReader vs("shader.vert");
		FileReader fs("shader.frag");
		vertexShader = new Graphics4::Shader(vs.readAll(), vs.size(), Graphics4::VertexShader);
		fragmentShader = new Graphics4::Shader(fs.readAll(), fs.size(), Graphics4::FragmentShader);

		// This defines the structure of your Vertex Buffer
		Graphics4::VertexStructure structure;
		structure.add("pos", Graphics4::Float3VertexData);
		structure.add("tex", Graphics4::Float2VertexData);
		structure.add("nor", Graphics4::Float3VertexData);

		pipeline = new Graphics4::PipelineState;
		pipeline->inputLayout[0] = &structure;
		pipeline->inputLayout[1] = nullptr;
		pipeline->vertexShader = vertexShader;
		pipeline->fragmentShader = fragmentShader;
		pipeline->depthMode = Graphics4::ZCompareLess;
		pipeline->depthWrite = true;
		pipeline->compile();

		tex = pipeline->getTextureUnit("tex");
		pvLocation = pipeline->getConstantLocation("PV");
		mLocation = pipeline->getConstantLocation("M");

		objects[0] = new MeshObject("Level/Level.obj", "Level/basicTiles6x6.png", structure);
		objects[1] = new MeshObject("Level/Level_yellow.obj", "Level/basicTiles3x3yellow.png", structure);
		objects[2] = new MeshObject("Level/Level_red.obj", "Level/basicTiles3x3red.png", structure);

		sphere = new MeshObject("ball_at_origin.obj", "Level/unshaded.png", structure);
		float pos = -10.0f;

		SpawnSphere(vec3(-pos, 5.5f, pos), vec3(0, 0, 0));
		physics.meshCollider.mesh = objects[0];

		// Sound source: http://opengameart.org/content/level-up-sound-effects
		/************************************************************************/
		/* Task P9.2: Play this sound when the goal is reached                   */
		/************************************************************************/
		winSound = new Sound("chipquest.wav");
		
		Graphics4::setTextureAddressing(tex, Graphics4::U, Graphics4::Repeat);
		Graphics4::setTextureAddressing(tex, Graphics4::V, Graphics4::Repeat);
	}
}

int kore(int argc, char** argv) {
	Kore::System::init("Exercise 9", width, height);

	Kore::Audio1::init();
	Kore::Audio2::init();

	init();

	Kore::System::setCallback(update);

	startTime = System::time();

	Keyboard::the()->KeyDown = keyDown;
	Keyboard::the()->KeyUp = keyUp;
	Mouse::the()->Move = mouseMove;
	Mouse::the()->Press = mousePress;
	Mouse::the()->Release = mouseRelease;

	Kore::System::start();

	return 0;
}
