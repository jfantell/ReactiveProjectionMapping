#version 330 core
in vec4 fragment_color;
out vec4 diffuse_color;
void main() {
    diffuse_color = fragment_color;
}