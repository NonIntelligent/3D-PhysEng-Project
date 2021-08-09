#pragma once

#include "GLFW/glfw3.h"

#include <unordered_map>

enum class KeyState{
	JUST_RELEASED,
	JUST_PRESSED,
	HELD_DOWN,
	NOT_PRESSED,
	// No longer considered to be in any state (i.e. Just_Released -> Held_Down)
	LIMBO
};

class Keyboard {
private:
	static KeyState keys[348];
private:
	// Static class to stop multiple object creation
	Keyboard();

public:
	long holdDelay = 5;

public:
	// Run at the end of input handling to update state of all keys.
	static void updateKeyStates();
	static KeyState getKeyState(int keyCode);

	static void key_callback(GLFWwindow * window, int key, int scancode, int action, int mods);

};
