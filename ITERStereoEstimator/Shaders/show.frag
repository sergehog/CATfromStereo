#version 330 core

varying float z;
//uniform sampler2D Texture1;
//uniform mat4 C1;

//uniform float minZ;
//uniform float maxZ;
//uniform uint layers;
//uniform uint width;
//uniform uint height;

// Ouput data
out vec4 colorOut;


void main()
{		
	colorOut = vec4(0, z, z, 0.5);
	//colorOut = vec3(1.0, 1.0, 1.0);
}


