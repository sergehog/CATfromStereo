#version 330 core

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

//uniform sampler2D Texture1;
//uniform sampler2D Texture2;


uniform uint width;
uniform uint height;


in vec2 UV[];
in int layer[];

out vec2 UV1;
flat out int disp;
//out vec2 UV2;

void main()
{		
	gl_Layer = layer[0];
	
	for(int i = 0; i < gl_in.length(); ++i)
	{			
		gl_Position = gl_in[i].gl_Position;
		UV1 = vec2(UV[i].x/float(width), UV[i].y/float(height));
		disp = int(gl_Layer);
		//float d = float(layer[0])/(float(layers)-1); 			
		//float z = 1.f/(d*(1.0/minZ - 1.0/maxZ) + 1.0/maxZ); // real Z hypothesis		
		//float z = maxZ - d*(maxZ-minZ); // check if linear scaling better
		//vec4 uvz1 = vec4(UV[i].x*z, UV[i].y*z, z, 1);
		//vec4 uvz2 = C2C1inv*uvz1;
		//UV2 = vec2((uvz2.x/uvz2.z)/float(width), (uvz2.y/uvz2.z)/float(height));
		EmitVertex();
	}
	EndPrimitive();	
}