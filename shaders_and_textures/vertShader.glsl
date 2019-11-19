#version 410

layout (location=0) in vec3 vPos;
layout (location=1) in vec3 vCol;
out vec3 fCol;

uniform mat4 mv_matrix;
uniform mat4 proj_matrix;

float min_z = 0;
float max_z = 1.5;

vec3 coloring(vec3 vCol){
	vec3 yCol;
	yCol[0] = vCol[0]; //this is RED
	yCol[1] = vCol[1]; //this is GREEN
	yCol[2] = vCol[2]; //this is BLUE

	return yCol;
}


void main(void)
{	gl_Position = proj_matrix * mv_matrix * vec4(vPos,1.0);
	fCol = coloring(vCol);
//	fCol = vCol / (vPos[2])/(max_z-min_z);
//	fCol[0] = vCol[0]; //this is RED
//	fCol[1] = vCol[1]; //this is GREEN
//	fCol[2] = vCol[2]; //this is BLUE

} 