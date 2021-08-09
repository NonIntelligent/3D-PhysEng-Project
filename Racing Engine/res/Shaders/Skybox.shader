#shader vertex
#version 430 core

layout(location = 0) in vec3 position;

layout(std140) uniform camera
{
	mat4 proj;
	mat4 view;
	vec3 viewPos;
};

// Used to get rid of translation caused by view matrix.
mat4 viewStatic;

out vec3 texCoords;

void main() {
	texCoords = position;
	viewStatic = mat4(mat3(view));

	vec4 pos = proj * viewStatic * vec4(position, 1.0);
	// Trick to optimise number of pixels drawn but the skybox needs to be rendered last.
	gl_Position = pos.xyww;
};

// Following this guide to make the skybox. https://www.youtube.com/watch?v=QYvi1akO_Po

#shader fragment
#version 430 core

layout(location = 0) out vec4 colour;

in vec3 texCoords;

uniform samplerCube u_Texture;

void main() {
	colour = texture(u_Texture, texCoords);
};