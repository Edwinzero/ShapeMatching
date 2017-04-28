#version 330 core

in vec3 out_normal;
uniform vec3 color;

layout (location = 0) out vec4 out_color;

void main(){
    float gamma = 2.2;
    //vec3 corrected_color = pow(out_normal, vec3(1.0/gamma));
    vec3 corrected_color = pow(color, vec3(1.0/gamma));
    out_color = vec4(corrected_color, 1.0);
}