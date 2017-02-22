#version 330 core

in varying float z;
uniform sampler2D Texture1;
uniform mat4 C1;

uniform float minZ;
uniform float maxZ;
uniform uint layers;
uniform uint width;
uniform uint height;

// Ouput data
out vec3 colorOut;


void main()
{		
	colorOut = vec3(z, z, z);
}


