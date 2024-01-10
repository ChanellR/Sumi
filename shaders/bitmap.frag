#version 330 core

in vec2 TexCoord;

out vec4 Fragcolor;

uniform sampler2D Bitmap;

void main() {
    Fragcolor = texture(Bitmap, TexCoord);
}