#pragma once

#include <glm/glm.hpp>
#include <iostream>
#include <fstream>
#include <utility>

namespace config
{	

	class camera_loader
	{
	public:

		//! returns intrinsic and extrinsic matrixes
		static const std::pair<const glm::mat3, const glm::mat4x3> read_settings2(std::string camera_settings, std::string camera_name, float &o1, float &o2)
		{
			std::ifstream camfile(camera_settings);
			char buffer[1024];
			camfile.getline(buffer, 1024);

			while (std::string(buffer).find(camera_name) == std::string::npos && !camfile.eof())
			{
				camfile.getline(buffer, 1024);
			}

			if (std::string(buffer).find(camera_name) == std::string::npos || camfile.eof())
			{
				throw std::exception("No specified camera parameters found!");
			}

			float a, b, c;
			float d, e, f;
			float g, h, i;
			camfile >> a >> b >> c;
			camfile >> d >> e >> f;
			camfile >> g >> h >> i;

			//! Column-major format here!!! THAT'S WRONG: //glm::mat3 intrinsics = glm::mat3 (a, b, c,  d, e, f,   g, h, i); 
			glm::mat3 intrinsics = glm::mat3(a, d, g, b, e, h, c, f, i);


			//optical distortions
			camfile >> o1 >> o2;

			float j, k, l;
			camfile >> a >> b >> c >> d;
			camfile >> e >> f >> g >> h;
			camfile >> i >> j >> k >> l;

			glm::mat4x3 extrinsics = glm::mat4x3(a, e, i, b, f, j, c, g, k, d, h, l);

			camfile.close();

			return std::make_pair(intrinsics, extrinsics);
		}

		//! returns intrinsic and extrinsic matrixes
		static const std::pair<const glm::mat3, const glm::mat4x3> read_settings(std::string camera_settings, std::string camera_name)
		{
			std::ifstream camfile(camera_settings);
			char buffer[1024];
			camfile.getline(buffer, 1024);

			while (std::string(buffer).find(camera_name) == std::string::npos && !camfile.eof())
			{
				camfile.getline(buffer, 1024);
			}

			if (std::string(buffer).find(camera_name) == std::string::npos || camfile.eof())
			{
				throw std::exception("No specified camera parameters found!");
			}

			float a, b, c;
			float d, e, f;
			float g, h, i;
			camfile >> a >> b >> c;
			camfile >> d >> e >> f;
			camfile >> g >> h >> i;

			//! Column-major format here!!! THAT'S WRONG: //glm::mat3 intrinsics = glm::mat3 (a, b, c,  d, e, f,   g, h, i); 
			glm::mat3 intrinsics = glm::mat3(a, d, g, b, e, h, c, f, i);


			float b1, b2;
			camfile >> b1 >> b2;

			float j, k, l;
			camfile >> a >> b >> c >> d;
			camfile >> e >> f >> g >> h;
			camfile >> i >> j >> k >> l;

			glm::mat4x3 extrinsics = glm::mat4x3(a, e, i, b, f, j, c, g, k, d, h, l);

			camfile.close();

			return std::make_pair(intrinsics, extrinsics);
		}


	};


	std::unique_ptr<float[]> prepare_lookup2(const glm::mat3 K1, const float cam1_k1, const float cam1_k2, const int width, const int height)
	{
		const float cam1_ox = K1[2][0];
		const float cam1_oy = K1[2][1];
		const float cam1_fx = K1[0][0];
		const float cam1_fy = K1[1][1];

		const long HW = width * height;
		std::unique_ptr<float[]> lookup = std::unique_ptr<float[]>(new float[width*height * 2]);
		float * const lookup_table = lookup.get();
#pragma omp parallel for
		for (int yi = 0; yi < height; yi++)
		{
			const long row = yi*width;
			for (int xi = 0; xi < width; xi++)
			{
				float x1 = (xi + 1 - cam1_ox) / cam1_fx;
				float y1 = (yi + 1 - cam1_oy) / cam1_fy;
				const float r1 = (x1*x1 + y1*y1);
				x1 = x1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
				y1 = y1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
				lookup_table[row + xi] = (x1*cam1_fx + cam1_ox);
				lookup_table[row + xi + HW] = (y1*cam1_fy + cam1_oy);
			}
		}

		return lookup;
	}

	GLuint prepare_lookup(const glm::mat3 K1, const float cam1_k1, const float cam1_k2, const int width, const int height)
	{
		const float cam1_ox = K1[2][0];
		const float cam1_oy = K1[2][1];
		const float cam1_fx = K1[0][0];
		const float cam1_fy = K1[1][1];

		//const float cam2_ox = K2[2][0];
		//const float cam2_oy = K2[2][1];
		//const float cam2_fx = K2[0][0];
		//const float cam2_fy = K2[1][1];

		std::unique_ptr<float[]> lookup = std::unique_ptr<float[]>(new float[width*height * 2]);
		float * const lookup_table = lookup.get();
		for (int yi = 0; yi < height; yi++)
		{
			const long row = yi*width * 2;
			for (int xi = 0; xi < width; xi++)
			{
				float x1 = (xi - cam1_ox) / cam1_fx;
				float y1 = (yi - cam1_oy) / cam1_fy;
				const float r1 = (x1*x1 + y1*y1);
				x1 = x1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
				y1 = y1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
				lookup_table[row + xi * 2] = (x1*cam1_fx + cam1_ox) / width;
				lookup_table[row + xi * 2 + 1] = (y1*cam1_fy + cam1_oy) / height;

				//float x2 = (xi - cam2_ox) / cam2_fx;
				//float y2 = (yi - cam2_oy) / cam2_fy;			
				//const float r2 = (x2*x2 + y2*y2);
				//x2 = x2 * (1 + cam2_k1*r2 + cam2_k2*r2*r2);
				//y2 = y2 * (1 + cam2_k1*r2 + cam2_k2*r2*r2);
				//lookup_table[row + xi * 4 + 2] = (x2*cam2_fx + cam2_ox) / width;
				//lookup_table[row + xi * 4 + 3] = (y2*cam2_fy + cam2_oy) / height;			
			}
		}

		GLuint lookupTextureID;
		glGenTextures(1, &lookupTextureID);

		// "Bind" the newly created texture : all future texture functions will modify this texture
		glBindTexture(GL_TEXTURE_2D, lookupTextureID);

		// Texture filtering
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RG, width, height, 0, GL_RG, GL_FLOAT, lookup_table);
		glGenerateMipmap(GL_TEXTURE_2D);
		return lookupTextureID;
	}

	/*
	GLuint prepare_lookup_inverse(const glm::mat3 K1, const glm::mat3 K2, float cam1_k1, const float cam1_k2, const float cam2_k1, const float cam2_k2, const int width, const int height)
	{
	const float cam1_ox = K1[2][0];
	const float cam1_oy = K1[2][1];
	const float cam1_fx = K1[0][0];
	const float cam1_fy = K1[1][1];

	const float cam2_ox = K2[2][0];
	const float cam2_oy = K2[2][1];
	const float cam2_fx = K2[0][0];
	const float cam2_fy = K2[1][1];


	std::unique_ptr<float[]> lookup = std::unique_ptr<float[]>(new float[width*height * 4]);
	float * const lookup_table = lookup.get();
	for (int yi = 0; yi < height; yi++)
	{
	long row = yi*width * 4;
	for (int xi = 0; xi < width; xi++)
	{
	float x1 = (xi - cam1_ox) / cam1_fx;
	float y1 = (yi - cam1_oy) / cam1_fy;
	const float r1 = (x1*x1 + y1*y1);
	x1 = x1 * (1 - cam1_k1*(r1) + (3 * cam1_k1*cam1_k1 - cam1_k2)*r1*r1);
	y1 = y1 * (1 - cam1_k1*(r1) + (3 * cam1_k1*cam1_k1 - cam1_k2)*r1*r1);
	lookup_table[row + xi * 4] = (x1*cam1_fx + cam1_ox) / width;
	lookup_table[row + xi * 4 + 1] = (y1*cam1_fy + cam1_oy) / height;

	float x2 = (xi - cam2_ox) / cam2_fx;
	float y2 = (yi - cam2_oy) / cam2_fy;
	const float r2 = x2*x2 + y2*y2;
	x2 = x2 * (1 - cam2_k1*(r2) + (3 * cam2_k1*cam2_k1 - cam2_k2)*r2*r2);
	y2 = y2 * (1 - cam2_k1*(r2) + (3 * cam2_k1*cam2_k1 - cam2_k2)*r2*r2);
	lookup_table[row + xi * 4 + 2] = (x2*cam2_fx + cam2_ox) / width;
	lookup_table[row + xi * 4 + 3] = (y2*cam2_fy + cam2_oy) / height;
	}
	}

	GLuint lookupTextureID;
	glGenTextures(1, &lookupTextureID);

	"Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, lookupTextureID);

	Texture filtering
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_FLOAT, lookup_table);
	glGenerateMipmap(GL_TEXTURE_2D);
	return lookupTextureID;
	}
	*/

};


