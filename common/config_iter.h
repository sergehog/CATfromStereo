/*	config_iter.h - external parameters to the program
	@author Sergey Smirnov sergei.smirnov@gmail.com
	@date 2016
*/
#pragma once

#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>


#define CONFIG_PARAM(a,b,c)
#define CONFIG_PARAM_OPTIONAL(a,b,c,d)

#define CONFIG_PARAMS_LIST \
	CONFIG_PARAM(string, camera1_ip, "config.camera1ip") \
	CONFIG_PARAM(string, camera2_ip, "config.camera2ip") \
	CONFIG_PARAM(unsigned, input_width, "config.input_width") \
	CONFIG_PARAM(unsigned, input_height, "config.input_height") \
	CONFIG_PARAM(string, camera_file, "config.camera_file") \
	CONFIG_PARAM(string, stl_file, "config.stl_file") \
	CONFIG_PARAM_OPTIONAL(string, points_file, "config.points_file", "points.bin") \
	CONFIG_PARAM(string, camera1_name, "config.camera1_name") \
	CONFIG_PARAM(string, camera2_name, "config.camera2_name") \
	CONFIG_PARAM_OPTIONAL(unsigned, camera_binning, "config.camera_binning", 1) \
	CONFIG_PARAM(float, minZ, "config.minZ") \
	CONFIG_PARAM(float, maxZ, "config.maxZ") \
	CONFIG_PARAM(int, layers, "config.layers") \
	CONFIG_PARAM_OPTIONAL(unsigned, frame_width, "config.frame_width", input_width) \
	CONFIG_PARAM_OPTIONAL(unsigned, frame_height, "config.frame_height", input_height) \
	CONFIG_PARAM_OPTIONAL(unsigned, screen_width, "config.screen_width", frame_width) \
	CONFIG_PARAM_OPTIONAL(unsigned, screen_height, "config.screen_height", frame_height) \
	CONFIG_PARAM_OPTIONAL(unsigned, buffer_frame_number, "config.buffer_frame_number", 2) \
	CONFIG_PARAM_OPTIONAL(float, param1, "config.param1", 0.f) \
	CONFIG_PARAM_OPTIONAL(float, param2, "config.param2", 0.f) \
	CONFIG_PARAM_OPTIONAL(float, plane_threshold, "config.plane_threshold", 20.f) \
	CONFIG_PARAM_OPTIONAL(float, scene_threshold, "config.plane_threshold", 200.f) \
	CONFIG_PARAM(string, vertex_shader, "config.vertex_shader") \
	CONFIG_PARAM(string, fragment_shader, "config.fragment_shader") \
	CONFIG_PARAM_OPTIONAL(string, geometry_shader, "config.geometry_shader", "") \
	CONFIG_PARAM(string, main_vertex_shader, "config.main_vertex_shader") \
	CONFIG_PARAM(string, main_fragment_shader, "config.main_fragment_shader") \
	CONFIG_PARAM(string, show_vertex_shader, "config.show_vertex_shader") \
	CONFIG_PARAM(string, show_fragment_shader, "config.show_fragment_shader")

using namespace std;
namespace config
{

	class config_iter
	{

#undef CONFIG_PARAM
#undef CONFIG_PARAM_OPTIONAL
#define CONFIG_PARAM(type, name, path)  const type name, 
#define CONFIG_PARAM_OPTIONAL(type, name, path, value) const type name,

	private:
		const int empty;
		//! private constructor , please use static load_config instead
		config_iter(CONFIG_PARAMS_LIST
			int empty
			) :
#undef CONFIG_PARAM
#undef CONFIG_PARAM_OPTIONAL
#define CONFIG_PARAM(type, name, path)  name(name), 
#define CONFIG_PARAM_OPTIONAL(type, name, path, value) name(name),
			CONFIG_PARAMS_LIST
			empty(0)
			{

			};

	public:

#undef CONFIG_PARAM
#undef CONFIG_PARAM_OPTIONAL
#define CONFIG_PARAM(type, name, path)  const type name;
#define CONFIG_PARAM_OPTIONAL(type, name, path, value) const type name;
		CONFIG_PARAMS_LIST

		static config_iter load_config(const char* filename)
		{

				boost::property_tree::ptree pt;
				boost::property_tree::read_xml(std::string(filename), pt);
				if (pt.empty())
				{
					printf("Config file \"%s\" is not valuid XML file!\n", filename);
					throw new std::exception("Config file is not valuid XML file!");
				}

#undef CONFIG_PARAM
#undef CONFIG_PARAM_OPTIONAL
#define CONFIG_PARAM(type, name, path)  const type name = pt.get<type>(path);
#define CONFIG_PARAM_OPTIONAL(type, name, path, value) const type name = pt.get<type>(path, value);

				CONFIG_PARAMS_LIST

#undef CONFIG_PARAM
#undef CONFIG_PARAM_OPTIONAL
#define CONFIG_PARAM(type, name, path)  name,
#define CONFIG_PARAM_OPTIONAL(type, name, path, value) name,

					return config_iter(CONFIG_PARAMS_LIST 0);
			}


	};


};
