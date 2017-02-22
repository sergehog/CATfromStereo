#version 330 core
layout(location = 0) in ivec2 uvMap; //layout(location = 1) in float disparity;

out vec2 UV;

uniform uint width;
uniform uint height;

void main()
{			
	UV = vec2(float(uvMap[0])/float(width), float(uvMap[1])/float(height));
	//UV = vec2(uvMap);	
	gl_Position = vec4(2.0*UV.x-1.0, 1.0-2.0*UV.y, 0.0, 1.0);

}