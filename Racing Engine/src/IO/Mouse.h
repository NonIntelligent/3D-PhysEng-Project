#pragma once

class Mouse {

public:
	double lastMouseX = 0.0;
	double lastMouseY = 0.0;
	double mouseX = 0.0;
	double mouseY = 0.0;
	double mouseOffsetX = 0.0;
	double mouseOffsetY = 0.0;
	// Very first mouse movement to set last mouse position
	bool firstMouse = true;
	// Moving mouse across the entire window will rotate by 180 degrees at value of 1.0 .
	float mouseSensitivity = 1.0f;

	bool freeMouse = false;

	const int clickTickDelay = 30;
	int currentDelay = 0;
};

