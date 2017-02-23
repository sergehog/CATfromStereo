#version 330 core

in vec2 UV;
uniform sampler2D Texture1;
uniform sampler2D DepthTexture;
// Ouput data
out vec4 colorOut;


void main()
{		
	float y = texture(Texture1, UV).r;
	float d = texture(DepthTexture, UV).r;
	colorOut = vec4(y, d, 0, 1);
	//colorOut = texture(Texture1, UV).rgb;
}


