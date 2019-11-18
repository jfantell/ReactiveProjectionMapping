#version 410

layout (location=0) in vec3 vPos;
layout (location=1) in vec3 vCol;
out vec3 fCol;

uniform mat4 mv_matrix;
uniform mat4 proj_matrix;

void main(void)
{	gl_Position = proj_matrix * mv_matrix * vec4(vPos,1.0);
	fCol = vCol;
} 