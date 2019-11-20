#version 410

in vec3 fCol;
out vec4 fragColor;

uniform mat4 mv_matrix;
uniform mat4 proj_matrix;
uniform mat4 norm_matrix;
uniform int use_texture;
uniform sampler2D samp;

void main(void)
{
    if(use_texture == 1){
        fragColor = texture(samp, vec2(fCol.x,fCol.y));
    }
    else{
        fragColor = vec4(fCol,1.0);
    }
}
