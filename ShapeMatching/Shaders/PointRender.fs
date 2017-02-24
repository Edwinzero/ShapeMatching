#version 330 core

in vec3 out_normal;
uniform vec3 color;

layout (location = 0) out vec4 out_color;

void main(){
    out_color = vec4(out_normal, 1.0);
}