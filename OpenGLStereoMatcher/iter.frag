#version 330 core

in vec2 UV1;
flat in int disp;
//in vec2 UV2;
//in float dx;
uniform sampler2D Texture1;
uniform sampler2D Texture2;
uniform uint width;
uniform uint height;
uniform mat4 C2C1inv;
uniform float minZ;
uniform float maxZ;
uniform uint layers;

//uniform float average1;
//uniform float average2;

const float average = 6.5;

// Ouput data
out float colorOut;

void main()
{	

	vec2 UV = vec2(UV1.x*float(width), UV1.y*float(height));
	float d = float(disp)/(float(layers)-1); 			
	float z = 1.f/(d*(1.0/minZ - 1.0/maxZ) + 1.0/maxZ); // real Z hypothesis		
	//float z = maxZ - d*(maxZ-minZ); // check if linear scaling better
	vec4 uvz1 = vec4(UV.x*z, UV.y*z, z, 1);
	vec4 uvz2 = C2C1inv*uvz1;
	vec2 UV2 = vec2((uvz2.x/uvz2.z)/float(width), (uvz2.y/uvz2.z)/float(height));

	vec4 c1 = textureLod( Texture1, UV1, 0.0);
	vec4 c2 = textureLod( Texture2, UV2, 0.0);
	//vec4 c2u = textureLod( Texture2, UV2-vec2(0, 1.0/height), 0.0);
	//vec4 c2d = textureLod( Texture2, UV2+vec2(0, 1.0/height), 0.0);
	//vec4 c1a = textureLod( Texture1, UV1, average);
	//vec4 c2a = textureLod( Texture2, UV2, average);
		
	//float rdiff = c1.r - average1*c2.r/average2;
	vec3 abs1 = abs(c1.rgb-c2.rgb);
	//vec3 abs1 = abs(c1.rgb-c2.rgb-c1a.rgb+c2a.rgb);
	//abs1.x = abs1.x > 0.1 ? 0.1 : abs1.x;
	//abs1.y = abs1.y > 0.1 ? 0.1 : abs1.y;
	//abs1.z = abs1.z > 0.1 ? 0.1 : abs1.z;
	//float SAD = 10*abs(c1.r-c2.r);	
	float SAD = (abs1.x + 10.0*abs1.g + 10.0*abs1.b);	
	//float SAD = 10.0*(abs1.r);	
	//float SAD = 10.0*(abs1.g + abs1.b);	

	//vec3 abs1u = abs(c1.rgb-c2u.rgb);
	//vec3 abs1d = abs(c1.rgb-c2d.rgb);
	//float SADu = 10.0*(abs1u.r + abs1u.g + abs1u.b);	
	//float SADd = 10.0*(abs1d.r + abs1d.g + abs1d.b);	
	//SAD = SADu < SAD ? SADu : SAD;
	//SAD = SADd < SAD ? SADd : SAD;

	//float SAD = 10.0*(abs1.g+abs1.b);	
	SAD = SAD > 1.0 ? 1.0 : SAD;

	colorOut = SAD;
}
