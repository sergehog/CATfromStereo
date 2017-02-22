#version 330 core

layout(location = 0) in vec3 stl;

varying float z;
uniform mat4 Transform;
uniform mat4 C1;
uniform uint width;
uniform uint height;
uniform uint layers;
uniform float minZ;
uniform float maxZ;

void main()
{			
	vec4 uvz = C1*Transform*(vec4(stl, 1));		
	float x = 2*(uvz.x/uvz.z)/float(width)-1.0;
	float y = 1.0-2.0*(uvz.y/uvz.z)/float(height);	
	z = (uvz.z-minZ)/(maxZ-minZ);	
	
	gl_Position = vec4(x, y, 0, 1.0);
}