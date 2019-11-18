#version 410

in vec3 fCol;
out vec4 fragColor;

void main(void)
{    fragColor = vec4(fCol,1.0);
}
