#version 410

layout (location=0) in vec3 vPos;
layout (location=1) in vec3 vCol;
layout (location=2) in vec2 vTex;
out vec3 fCol;

uniform mat4 mv_matrix;
uniform mat4 proj_matrix;
uniform int use_texture;
uniform sampler2D samp;

float min_z = 0;
float max_z = 1.5;
float desiredStripes = 15;

vec3 coloring(vec3 vCol, vec3 vPos, float stripeRange, float range, float max_z){
	vec3 yCol;
	//Depth Stripes
	if ((int(vPos[2]/stripeRange))%3 == 0){ //Identify what number stripe this is, then identify if it is odd
		yCol[0] = vCol[0]*1; //this is RED
		yCol[1] = vCol[1]*.2; //this is GREEN
		yCol[2] = vCol[2]*.2; //this is BLUE
	}
	else if ((int(vPos[2]/stripeRange))%3 == 1){ //Identify what number stripe this is, then identify if it is odd (trunc(vPos[2]/stripeRange)%2 == 0)
		yCol[0] = vCol[0] *.2; //this is RED
		yCol[1] = vCol[1] *.2; //this is GREEN
		yCol[2] = vCol[2] *1; //this is BLUE
	}
	else { //Only used if 3 lines chosen
		yCol[0] = vCol[0] *.2; //this is RED
		yCol[1] = vCol[1] *1; //this is GREEN
		yCol[2] = vCol[2] *.2; //this is BLUE
	}

	//now to apply depth shading
	yCol = yCol * ((max_z - vPos[2])/(range));

	return yCol;
}

void main(void)
{
	gl_Position = proj_matrix * mv_matrix * vec4(vPos,1.0);
	if(use_texture == 1){
		fCol = vec3(vTex,1);
	}
	else{
		//Determing stripe values
		float range = max_z -min_z;
		float stripeRange = range/desiredStripes;
		//color
		fCol = coloring(vCol, vPos, stripeRange, range, max_z);
	}
} 