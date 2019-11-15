#version 330 core
layout (location = 0) in vec3 vertex_position;
out vec4 fragment_color;

uniform mat4 MVP;

void main() {
    gl_Position = MVP * vec4(vertex_position, 1.0f);
    //gl_Position = vec4(vertex_position, 1.0f);
    //fragment_color = vec4(vertex_color,1.0f);
    fragment_color = vec4(1.0f, 1.0f, 1.0f, 1.0f);
}