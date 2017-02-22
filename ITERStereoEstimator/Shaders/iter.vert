#version 330 core

layout(location = 0) in ivec2 uvMap; //layout(location = 1) in float disparity;

out vec2 UV;
out int layer;

uniform mat4 C2C1inv;
uniform float minZ;
uniform float maxZ;

uniform uint layers;
uniform uint width;
uniform uint height;

void main()
{				
	layer = gl_InstanceID;
	vec2 UV1 = vec2(float(uvMap[0])/float(width), float(uvMap[1])/float(height));
	UV = vec2(uvMap);	
	gl_Position = vec4(2.0*UV1.x-1.0, 1.0-2.0*UV1.y, 0.0, 1.0);
}