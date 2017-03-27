#version 330 core

in vec2 texCoord;

uniform sampler2D image;

void main(){
    //gl_FragColor = texture2D(image, texCoord);
    gl_FragColor = vec4(1.0, 1.0, 0.0, 1.0);
}