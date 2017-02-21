#version 330 core

in vec2 UV;
uniform sampler2D Texture1;
uniform sampler2D Texture2;
uniform sampler2D Depth1;
uniform sampler2D Depth2;

// Ouput data
out vec3 colorOut;

void main()
{
	//float c1 = texture(Texture1, UV).r;		
	//float c2 = texture(Texture2, UV).r;	
	float d1 = texture(Depth1, UV).r;	
	//colorOut = texture(Texture1, UV).rgb;
	//colorOut = vec3(c1, c1, c1);
	colorOut = vec3(d1*2, d1*2, d1*2);
	//colorOut = vec3(d1, d1, d1);
	//colorOut = vec3(c1, c2, c1+c2);
}
