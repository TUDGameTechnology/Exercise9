#include "pch.h"

#include <Kore/IO/FileReader.h>
#include <Kore/Math/Core.h>
#include <Kore/Math/Random.h>
#include <Kore/System.h>
#include <Kore/Input/Keyboard.h>
#include <Kore/Input/Mouse.h>
#include <Kore/Audio/Mixer.h>
#include <Kore/Graphics/Image.h>
#include <Kore/Graphics/Graphics.h>
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
	Shader* vertexShader;
	Shader* fragmentShader;
	Program* program;

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
	TextureUnit tex;
	ConstantLocation pvLocation;
	ConstantLocation mLocation;

	/************************************************************************/
	/* Task P9.2 - Initialize the box collider                           */
	/************************************************************************/
	BoxCollider boxCollider;

	double lastTime = 0.0;

	void update() {
		double t = System::time() - startTime;
		double deltaT = t - lastTime;
		lastTime = t;

		Kore::Audio::update();
		
		Graphics::begin();
		Graphics::clear(Graphics::ClearColorFlag | Graphics::ClearDepthFlag, 0xff9999FF, 1000.0f);

		program->set();

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

		Graphics::setMatrix(pvLocation, PV);

		// Render the mesh objects
		MeshObject** current = &objects[0];
		while (*current != nullptr) {
			// set the model matrix
			Graphics::setMatrix(mLocation, (*current)->M);
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
			Graphics::setMatrix(mLocation, (*currentP)->Mesh->M);
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
			Mixer::play(winSound);
		}
			
		Graphics::end();
		Graphics::swapBuffers();
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
		if (code == Key_Up || code == Key_W) {
			up = isDown;
		} else if (code == Key_Down || code == Key_S) {
			down = isDown;
		} else if (code == Key_Left || code == Key_A) {
			right = isDown;
		} else if (code == Key_Right || code == Key_D) {
			left = isDown;
		}
	}

	void keyDown(KeyCode code, wchar_t character) {
		handleKeyEvent(code, true);
	}

	void keyUp(KeyCode code, wchar_t character) {
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
		vertexShader = new Shader(vs.readAll(), vs.size(), VertexShader);
		fragmentShader = new Shader(fs.readAll(), fs.size(), FragmentShader);

		// This defines the structure of your Vertex Buffer
		VertexStructure structure;
		structure.add("pos", Float3VertexData);
		structure.add("tex", Float2VertexData);
		structure.add("nor", Float3VertexData);

		program = new Program;
		program->setVertexShader(vertexShader);
		program->setFragmentShader(fragmentShader);
		program->link(structure);

		tex = program->getTextureUnit("tex");
		pvLocation = program->getConstantLocation("PV");
		mLocation = program->getConstantLocation("M");

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
		Mixer::play(winSound);

		Graphics::setRenderState(DepthTest, true);
		Graphics::setRenderState(DepthTestCompare, ZCompareLess);

		Graphics::setTextureAddressing(tex, U, Repeat);
		Graphics::setTextureAddressing(tex, V, Repeat);
	}
}

int kore(int argc, char** argv) {
	Kore::System::setName("TUD Game Technology - ");
	Kore::System::setup();
	Kore::WindowOptions options;
	options.title = "Solution 9";
	options.width = width;
	options.height = height;
	options.x = 100;
	options.y = 100;
	options.targetDisplay = -1;
	options.mode = WindowModeWindow;
	options.rendererOptions.depthBufferBits = 16;
	options.rendererOptions.stencilBufferBits = 8;
	options.rendererOptions.textureFormat = 0;
	options.rendererOptions.antialiasing = 0;
	Kore::System::initWindow(options);

	Kore::Mixer::init();
	Kore::Audio::init();

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
