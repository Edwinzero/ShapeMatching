#version 330 core
layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

out vec3 out_normal;
out vec3 in_color;

void main(){
    out_normal = (transpose(inverse(model)) * vec4(normal, 1.0)).xyz;
    vec4 model_pos = model * vec4(pos, 1.0);
    vec4 view_pos = view * model_pos; 
    in_color = color; 
    gl_Position = proj * view_pos;
    gl_PointSize = 3.0;
} 