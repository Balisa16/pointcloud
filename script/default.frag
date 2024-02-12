#version 330 core

// Outputs colors in RGBA
out vec4 FragColor;


in vec3 color;
in vec2 texCoord;

// Gets the Texture Unit from the main function
uniform sampler2D tex0;


void main()
{
	FragColor = texture(tex0, texCoord);
}