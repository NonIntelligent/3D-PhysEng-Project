#shader vertex
#version 430 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_texCoord;

layout(std140) uniform camera
{
	mat4 proj;
	mat4 view;
	vec3 viewPos;
};

out vec2 v_TexCoord;
out vec3 fragPos;
out vec3 normal;
out vec3 viewPosition;

uniform mat4 model;

void main() {
	gl_Position = proj * view * model * vec4(a_position, 1.0);
	fragPos = vec3(model * vec4(a_position, 1.0));
	v_TexCoord = a_texCoord;
	viewPosition = viewPos;
	normal = mat3(transpose(inverse(model))) * a_normal;
};

#shader fragment
#version 430 core

struct Material {
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

struct DirLight {
	vec3 direction;

	vec3 lightColour;
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

struct PointLight {
	vec3 position;

	float constant;
	float linear;
	float quadratic;

	vec3 lightColour;
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

struct SpotLight {
	vec3 position;
	vec3 direction;
	float cutOff;
	float outerCutOff;

	float constant;
	float linear;
	float quadratic;

	vec3 lightColour;
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};


layout(location = 0) out vec4 colour;

#define MAX_DIR_LIGHTS 2
#define MAX_POINT_LIGHTS 16
#define MAX_SPOT_LIGHTS 16

in vec2 v_TexCoord;
in vec3 fragPos;
in vec3 normal;
in vec3 viewPosition;

uniform Material material;
uniform DirLight dirLights[MAX_DIR_LIGHTS];
uniform PointLight pointLights[MAX_POINT_LIGHTS];
uniform SpotLight spotLights[MAX_SPOT_LIGHTS];

uniform int num_dirLights;
uniform int num_pointLights;
uniform int num_spotLights;

uniform vec3 u_Colour;
uniform sampler2D u_Texture;

// Lighting shader created following this guide https://learnopengl.com/Lighting/Multiple-lights

// Method instatiation.
vec3 calcDirLight(DirLight light, vec3 normal, vec3 viewDir);
vec3 calcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir);
vec3 calcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir);

void main() {

	vec3 norm = normalize(normal);
	vec3 viewDir = normalize(viewPosition - fragPos);

	// Calculate directional, point and spot lights in that order
	vec3 result = vec3(0.0);

	for(int i = 0; i < num_dirLights && i < MAX_DIR_LIGHTS; i++) {
		result += calcDirLight(dirLights[i], norm, viewDir);
	}

	for(int i = 0; i < num_pointLights && i < MAX_POINT_LIGHTS; i++) {
		result += calcPointLight(pointLights[i], norm, fragPos, viewDir);
	}

	for(int i = 0; i < num_spotLights && i < MAX_SPOT_LIGHTS; i++) {
		result += calcSpotLight(spotLights[i], norm, fragPos, viewDir);
	}


	colour = vec4(result, 1.0);
};

vec3 calcDirLight(DirLight light, vec3 normal, vec3 viewDir) {
	vec3 lightDir = normalize(-light.direction);

	// Diffuse lighting
	float diff = max(dot(normal, lightDir), 0.0);

	// Specular lighting
	vec3 reflectDir = reflect(-lightDir, normal);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);

	// Combine lighting
	vec3 ambient = light.lightColour * light.ambient * vec3(texture(u_Texture, v_TexCoord));
	vec3 diffuse = light.lightColour * light.diffuse * diff * vec3(texture(u_Texture, v_TexCoord));
	vec3 specular = light.lightColour * light.specular * spec * material.specular;

	return (ambient + diffuse + specular);
}

vec3 calcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir) {
	vec3 lightDir = normalize(light.position - fragPos);

	// Diffuse lighting
	float diff = max(dot(normal, lightDir), 0.0);

	// Specular lighting
	vec3 reflectDir = reflect(-lightDir, normal);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);

	// Attenuation
	float distance = length(light.position - fragPos);
	float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));

	// Combine lighting
	vec3 ambient = light.lightColour * light.ambient * vec3(texture(u_Texture, v_TexCoord));
	vec3 diffuse = light.lightColour * light.diffuse * diff * vec3(texture(u_Texture, v_TexCoord));
	vec3 specular = light.lightColour * light.specular * spec * material.specular;
	ambient *= attenuation;
	diffuse *= attenuation;
	specular *= attenuation;

	return (ambient + diffuse + specular);
}

vec3 calcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir) {
	vec3 lightDir = normalize(light.position - fragPos);

	// Diffuse lighting
	float diff = max(dot(normal, lightDir), 0.0);

	// Specular lighting
	vec3 reflectDir = reflect(-lightDir, normal);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);

	// Attenuation
	float distance = length(light.position - fragPos);
	float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));

	// Spotlight intensity
	float theta = dot(lightDir, normalize(-light.direction));
	float epsilon = light.cutOff - light.outerCutOff;
	float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);

	// Combine lighting
	vec3 ambient = light.lightColour * light.ambient * vec3(texture(u_Texture, v_TexCoord));
	vec3 diffuse = light.lightColour * light.diffuse * diff * vec3(texture(u_Texture, v_TexCoord));
	vec3 specular = light.lightColour * light.specular * spec * material.specular;
	ambient *= attenuation * intensity;
	diffuse *= attenuation * intensity;
	specular *= attenuation * intensity;

	return (ambient + diffuse + specular);
}