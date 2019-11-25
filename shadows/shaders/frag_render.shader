#version 330 core

in vec3 fragment_color;
in vec3 normal;
in vec3 fragment_position;
in vec2 frag_tex_coord;
in vec3 vNormal, vLightDir, vVertPos, vHalfVec;
in shadow_coord;

out vec4 diffuse_color;

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

layout (binding=0) uniform sampler2DShadow shadowTex;
layout (binding=1) uniform sampler2D textureimage;

void main(void) {
    vec3 L = normalize(vLightDir);
    vec3 N = normalize(vNormal);
    vec3 V = normalize(-vVertPos);
    vec3 H = normalize(vHalfVec);

    vec3 ambientLight = ambientLightStrength * ambientLightColor;

    vec3 norm = normalize(normal);
    vec3 lightDir = normalize(lightPosition - fragment_position);

    float diff = max(dot(norm, lightDir), 0.0f);
    vec3 diffuse = diff * ambientLightColor;

    vec3 viewDir = normalize(viewPosition - fragment_position);
    vec3 reflectDir = reflect(-lightDir, norm);

    float spec = pow(max(dot(viewDir, reflectDir), 0.0f), 128);
    vec3 specular = specularStrength * spec * ambientLightColor;

    float inShadow = textureProj(shadowTex, shadow_coord);

    //vec3 result = (ambientLight + diffuse + specular) * vec3(fragment_color);
    vec3 result;
    if(USE_TEX == 1){
        result = (ambientLight + diffuseLightStrength * diffuse + specular) * vec3(texture(textureimage, frag_tex_coord));
        diffuse_color = vec4(result, 1.0f);
    }
    else{
        result = (ambientLight + diffuseLightStrength * diffuse + specular) * fragment_color;
        diffuse_color = vec4(result, 1.0f);
    }

    if (inShadow != 0.0)
    {	fragColor += light.diffuse * material.diffuse * max(dot(L,N),0.0)
    		+ light.specular * material.specular
    		* pow(max(dot(H,N),0.0),material.shininess*3.0);
    }

    diffuse_color = vec4(result, 1.0f);
}