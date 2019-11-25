#version 330 core

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec2 vertex_tex_coord;
layout (location = 3) in vec3 vertex_colors;

out vec3 fragment_color;
out vec3 normal;
out vec3 fragment_position;
out vec2 frag_tex_coord;
out vec3 vNormal, vLightDir, vVertPos, vHalfVec;
out vec4 shadow_coord;

layout (binding=0) uniform sampler2DShadow shadowTex;
layout (binding=1) uniform sampler2D textureimage;

uniform mat4 Model; //rendables
uniform mat4 View;
uniform mat4 ModelView;
uniform mat4 Perpective;
uniform int Use_Text;
uniform mat4 Shadow_MVP;
uniform mat4 InvTr_MV;
uniform float ambientLightStrength; //point light
uniform float diffuseLightStrength;
uniform vec3 ambientLightColor;
uniform float specularStrength;
uniform vec3 lightPosition;
uniform vec3 viewPosition; //camera

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
    gl_Position = MVP * vec4(vertex_position, 1.0);

    if(USE_TEX == 1){
        frag_tex_coord = vertex_tex_coord;
    }
    else{
        //Determing stripe values
        float range = max_z -min_z;
        float stripeRange = range/desiredStripes;
        //color
        fragment_color = coloring(vertex_colors, vertex_position, stripeRange, range, max_z);
    }

    //output the vertex position to the rasterizer for interpolation
    vVertPos = (ModelView * vec4(vertex_position,1.0)).xyz;

    //get a vector from the vertex to the light and output it to the rasterizer for interpolation
    vLightDir = lightPosition - vVertPos;

    //get a vertex normal vector in eye space and output it to the rasterizer for interpolation
    vNormal = (InvTr_MV * vec4(vertex_normal,1.0)).xyz;

    // calculate the half vector (L+V)
    vHalfVec = (vLightDir-vVertPos).xyz;

    shadow_coord = shadowMVP * vec4(vertPos,1.0);

    fragment_position = vec3(Model * vec4(vertex_position, 1.0f));
    normal = vertex_normal;
}