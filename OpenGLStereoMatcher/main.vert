#version 330 core

layout(location = 0) in ivec2 uvMap; //layout(location = 1) in float disparity;

out vec2 UV;
out float dx;
uniform float minZ;
uniform float maxZ;
//uniform uint layer;
uniform uint layers;
uniform uint width;
uniform uint height;

void main()
{				
	UV = vec2(float(uvMap.x/float(width)), float(uvMap.y)/float(height));
	
	//float d = float(gl_InstanceID)/float(layers); 
	//float dx = float(layer)/float(layers); 			
	
	gl_Position = vec4(1.0-2.0*UV.x, 1.0-2.0*UV.y, 0.0, 1.0);
}