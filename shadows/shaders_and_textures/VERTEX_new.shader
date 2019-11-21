#version 330 core

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec2 vertex_tex_coord;
layout (location = 3) in vec3 vertex_color;

out vec3 fragment_color;
out vec3 normal;
out vec3 fragment_position;
out vec2 frag_tex_coord;

uniform mat4 MVP;
uniform mat4 Model;
uniform int USE_TEX;

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

void main(void) {
    gl_Position = MVP * vec4(vertex_position, 1.0f);

    if(USE_TEX == 1){
        frag_tex_coord = vertex_tex_coord;
        fragment_color = vec3(1.0,1.0,1.0);
    }
    else{
        //Determing stripe values
        float range = max_z -min_z;
        float stripeRange = range/desiredStripes;
        //color
        fragment_color = coloring(vertex_color, vertex_position, stripeRange, range, max_z);
        frag_tex_coord = vec2(0,0);
    }

    fragment_position = vec3(Model * vec4(vertex_position, 1.0f));
    normal = vertex_normal;
}