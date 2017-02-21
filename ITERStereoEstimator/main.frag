#version 330 core

in vec2 UV;
//in float dx;
uniform sampler2D Texture1;
uniform sampler2D Texture2;
uniform sampler2DArray TextureCost;

uniform uint layers;

// Ouput data
out vec3 colorOut;

const float average = 2.5;

void main()
{		
	//colorOut = texture(Texture1, UV).rgb;
	//return;
	float min_cost = textureLod(TextureCost, vec3(UV, 0.0), average).r;	
	//colorOut = vec3(min_cost, min_cost, min_cost);
	float dx = 0.0;
	for(int layer=1; layer<int(layers); layer++)
	{		
		float cost = textureLod(TextureCost, vec3(UV, float(layer)), average).r;
		
		if(cost <= min_cost)
		{
			dx = float(layer)/(float(layers)-1);
			min_cost = cost;
		}
	}		
	colorOut = vec3(dx, dx, dx);
	//colorOut = vec3(min_cost, min_cost, min_cost);

}
