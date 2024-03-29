#version 330 core
in vec4 fragment_color;
in vec3 normal;
in vec3 fragment_position;
in vec2 frag_tex_coord;

out vec4 diffuse_color;

uniform float ambientLightStrength;
uniform float diffuseLightStrength;
uniform vec3 ambientLightColor;
uniform float specularStrength;
uniform vec3 lightPosition;
uniform vec3 viewPosition;

uniform sampler2D textureimage;

void main() {

    vec3 ambientLight = vec3(ambientLightStrength * ambientLightColor);

    vec3 norm = normalize(normal);
    vec3 lightDir = normalize(lightPosition - fragment_position);

    float diff = max(dot(norm, lightDir), 0.0f);
    vec3 diffuse = vec3(diff * ambientLightColor);

    vec3 viewDir = normalize(viewPosition - fragment_position);
    vec3 reflectDir = reflect(-lightDir, norm);

    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 128);
    vec3 specular = specularStrength * spec * ambientLightColor;

    //vec3 result = (ambientLight + diffuse + specular) * vec3(fragment_color);
    vec3 result = (ambientLight + diffuseLightStrength * diffuse + specular) * vec3(texture(textureimage, frag_tex_coord));

    diffuse_color = vec4(result, 1.0f);

}