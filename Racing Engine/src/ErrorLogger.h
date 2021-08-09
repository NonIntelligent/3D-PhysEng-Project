#pragma once

// Code taken from https://www.youtube.com/watch?v=FBbPWSOQ0-w&list=PLlrATfBNZ98foTJPJ_Ev03o2oq3-GGOS2&index=10

#define ASSERT(x) if (!(x)) __debugbreak();
#define GLCall(x) glClearErrors();\
	x;\
	ASSERT(glCheckError(#x, __FILE__, __LINE__))

void glClearErrors();
bool glCheckError(const char* function, const char* file, int line);