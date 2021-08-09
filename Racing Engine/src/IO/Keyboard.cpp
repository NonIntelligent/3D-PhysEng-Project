#include "Keyboard.h"

// Init static array variable.
KeyState Keyboard::keys[348];

Keyboard::Keyboard() {
	for(int i = 0; i < 349; i++) {
		keys[i] = KeyState::NOT_PRESSED;
	}
}

void Keyboard::updateKeyStates() {
	for(int i = 0; i < 349; i++) {
		// Change state from just released to not being pressed
		if(keys[i] == KeyState::JUST_RELEASED) {
			keys[i] = KeyState::NOT_PRESSED;
		}
		else if(keys[i] == KeyState::JUST_PRESSED) {
			keys[i] = KeyState::LIMBO;
		}
	}
}

KeyState Keyboard::getKeyState(int keyCode) {
	return Keyboard::keys[keyCode];
}

void Keyboard::key_callback(GLFWwindow * window, int key, int scancode, int action, int mods) {
	if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		keys[GLFW_KEY_ESCAPE] = KeyState::JUST_PRESSED;
		glfwSetWindowShouldClose(window, GL_TRUE);
	}

	// If key was WASD
	bool isMovementKey = (key == GLFW_KEY_W || key == GLFW_KEY_A || key == GLFW_KEY_S || key == GLFW_KEY_D);

	if(action == GLFW_PRESS) {
		keys[key] = KeyState::JUST_PRESSED;
	}

	if(action == GLFW_REPEAT) {

	}

	if(action == GLFW_RELEASE) {
		keys[key] = KeyState::JUST_RELEASED;
	}
}
