#version 330

in vec3 position;
in vec2 texcoord;
//uniform mat4 projectionMat;
uniform mat4 viewMat;
out vec2 texCoordFs;

void main(void)
{
	texCoordFs = texcoord;
	//gl_Position = projectionMat * (viewMat * vec4(position, 1.0));
	//gl_Position = vec4(position, 1.0) * viewMat;
	gl_Position = viewMat * vec4(position, 1.0);
}
