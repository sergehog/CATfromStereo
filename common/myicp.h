#pragma once
#include <memory>
#include <Eigen/Dense>
#include <nanoflann.hpp>

namespace myicp
{
	struct Point
	{
		float x;
		float y;
		float z;
	};

	class MyICP
	{
		std::unique_ptr<Point[]> Model;

	public:

		MyICP(std::unique_ptr<Point[]> _Model)
		{
			Model.swap(_Model);

		}

		Eigen::Matrix4f Register()
	};
	

}