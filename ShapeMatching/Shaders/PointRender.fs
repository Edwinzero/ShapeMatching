#version 330 core

in vec3 normals;

layout (location = 0) out vec4 out_color;

void main(){
    out_color = vec4(1.0, 0.0, 0.0, 1.0);
}