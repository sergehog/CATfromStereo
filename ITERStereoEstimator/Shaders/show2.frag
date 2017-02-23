#version 330 core

in vec2 UV;
uniform sampler2D Texture1;

// Ouput data
out vec4 colorOut;


void main()
{		
	float y = texture(Texture1, UV).r;
	colorOut = vec4(y, y, 0, 1);
	//colorOut = texture(Texture1, UV).rgb;
}


