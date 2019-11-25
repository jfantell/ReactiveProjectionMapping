#version 330 core

layout (location = 0) in vec3 vertex_position;

uniform mat4 Shadow_MVP;

void main(void)
{	gl_Position = Shadow_MVP * vec4(vertex_position,1.0);
}