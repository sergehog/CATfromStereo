#version 330 core

layout(location = 0) in vec3 stl;

varying out float z;
uniform mat4 Transform;
uniform mat4 C1;
uniform uint width;
uniform uint height;
uniform uint layers;
uniform float minZ;
uniform float maxZ;

void main()
{			
	vec4 uvz = C1*Transform*vec4(stl, 1);
	z = (uvz.z-minZ)/(maxZ-minZ);
	gl_Position = vec4((uvz.x/uvz.z)/width, (uvz.u/uvz.z)/height, (uvz.z-minZ)/(maxZ-minZ), 1.0);
}