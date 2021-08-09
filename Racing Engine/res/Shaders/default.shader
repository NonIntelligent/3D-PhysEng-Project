#shader vertex
#version 430 core

layout(location = 0) in vec3 position;

layout(std140) uniform camera
{
	mat4 proj;
	mat4 view;
	vec3 viewPos;
};

void main() {
	gl_Position = proj * view * vec4(position, 1.0);
};

#shader fragment
#version 430 core

layout(location = 0) out vec4 colour;


void main() {
	colour = vec4(1.0, 0.0, 0.0, 1.0);
};