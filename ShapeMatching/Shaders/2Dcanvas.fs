#version 330 core

in vec2 texCoord;

uniform sampler2D image;

layout (location = 0) out vec4 out_color;

void main(){
    //gl_FragColor = texture2D(image, texCoord);
    out_color = vec4(1.0, 1.0, 0.0, 1.0);
}