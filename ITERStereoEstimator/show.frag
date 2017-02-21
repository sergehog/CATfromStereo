#version 330 core

in vec2 UV;
uniform sampler2D Texture1;
uniform sampler2D Texture2;
uniform sampler2D TextureDepth;

uniform mat4 C2C1inv;
uniform mat4 C1C2inv;

uniform float minZ;
uniform float maxZ;
uniform uint layers;
uniform uint width;
uniform uint height;

// Ouput data
out vec3 colorOut;


void main()
{		
	vec2 UV1 = vec2(UV.x/width, UV.y/height);
	//float color1 = textureLod(Texture1, UV1, 0.0).r;	
	vec3 color = textureLod(Texture1, UV1, 1.0).rgb;	
	//colorOut = vec3(color1, color1, color1);
	
	//colorOut = textureLod(Texture1, UV1, 0.0).rgb;
	
	float d1  = textureLod(TextureDepth, vec2(UV1.x, 0.5+UV1.y/2), 0.0).r;	
	colorOut = vec3(d1, d1, d1);	
	/*
	
	float grad = abs(color.g - 0.5) + abs(color.b - 0.5);
	//if(grad > 0.01)
	//{
	//	colorOut = vec3(d1, d1, d1);	
	//}
	//else
	//{
	//	colorOut = vec3(0, 0, 0);	
	//}
	
	

	//colorOut = vec3(color1, 0, d1);		
	
	//colorOut = vec3(color, d, color+d);	
	//float color2 = texture(Texture2, UV).r;	
	//float d2  = texture(TextureDepth, vec2(UV.x, 0.5-UV.y/2)).r;		
	//colorOut = vec3(color2, d2, color+d2);	
	
		
	//float z = 1.f/(d1*(1.0/minZ - 1.0/maxZ) + 1.0/maxZ); // real Z hypothesis		
	float z = maxZ - d1*(maxZ-minZ);
	vec4 uvz1 = vec4(UV.x*z, UV.y*z, z, 1);
	vec4 uvz2 = C2C1inv*uvz1;
	vec2 UV2 = vec2((uvz2.x/uvz2.z)/float(width), (uvz2.y/uvz2.z)/float(height));
	//float color2 = textureLod(Texture2, UV2, 0.0).r;	
	
	//colorOut = vec3(color2, 0, color1);	
	
	//colorOut = vec3(color1, (color1+color2)/2, color2);	
	//colorOut = vec3(color2, color2, color2);	
	
	float d2 = textureLod(TextureDepth, vec2(UV2.x, UV2.y/2), 0.0).r;		
	//colorOut = vec3(d1, d2, d1+d2);
	//colorOut = vec3(d2, d2, d2);
	//return;
	
	if(abs(d1-d2) < 0.05 && grad > 0.01 && color.r > 0.01)
	{		
		float d = (d1 + d2)/2;
		colorOut = vec3(d, d, d);
		//colorOut = vec3(d1, d1, d1);
	}
	else
	{
		colorOut = vec3(0,0,0);
	}
	
	//colorOut = vec3(color, color, color+depth);
	//colorOut = vec3(UV.x, UV.y, UV.x+UV.y);
	//*/	
}


