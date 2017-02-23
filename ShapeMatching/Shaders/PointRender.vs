#version 330 core
layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

out vec3 normals;

void main(){
    mat4 vm = view * model;
    mat4 mvp = proj * vm;
    normals = (vm * vec4(normal, 1.0)).xyz;
    gl_Position = mvp * vec4(pos, 1.0);
} 