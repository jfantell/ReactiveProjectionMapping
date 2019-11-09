#version 410

in vec3 fragment_color;
in vec3 normal;
in vec3 fragment_position;
in vec2 frag_tex_coord;
in vec3 vNormal, vLightDir, vVertPos, vHalfVec;
in vec4 shadow_coord;

out vec4 diffuse_color;

uniform sampler2DShadow shadowTex;
uniform sampler2D textureimage;

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

    vec3 result;
    if(Use_Text == 1){
        result = (ambientLight + diffuseLightStrength * diffuse + specular) * vec3(texture(textureimage, frag_tex_coord));
    }
    else{
        result = (ambientLight + diffuseLightStrength * diffuse + specular) * fragment_color;
    }

    if (inShadow != 0.0){
        result += diffuse  * max(dot(L,N), 0.0) + specular * pow(max(dot(H,N),0.0),16);
    }

    diffuse_color = vec4(result, 1.0f);
}