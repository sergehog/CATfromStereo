#pragma once

//#pragma pack(push, 1)
//union Rgb {
//	struct {
//		uint8_t r;
//		uint8_t g;
//		uint8_t b;
//	} ch;
//	unsigned char data[3];
//};
//#pragma pack(pop)

#pragma pack(push, 1)
union Rgba {
	struct {
		uint8_t r;
		uint8_t g;
		uint8_t b;
		uint8_t a;
	} ch;
	uint8_t data[4];
	uint32_t value;
};
#pragma pack(pop)


template<typename T>
class simple_image
{
private:
	const bool allocated;
	

public:	
	T* const buffer;
	const int width;
	const int height;
	const int layers;
	const long HW;

	simple_image(T* const _buffer, const int _width, const int _height, const int _layers = 1) :
		buffer(_buffer), width(_width), height(_height), layers(_layers), HW(_width*_height), allocated(false)
	{
		;
	}

	simple_image(const int _width, const int _height, const int _layers = 1) :
		buffer(new T[_width * _height * _layers]),
		width(_width), height(_height), layers(_layers), HW(_width*_height), allocated(true)
	{
		;
	}

	T &operator() (const int x, const int y, const int l=0) const
	{
		return buffer[l * HW + y * width + x];
	}

	T &value(const int x, const int y, const int l = 0) const
	{
		return buffer[l * HW + y * width + x];
	}

	void set(const T value)
	{
		#pragma omp parallel for
		for (long i = 0; i < HW*layers; i++)
		{
			buffer[i] = value;
		}
	}	
	
	// Returns interpolated color (nearest neighbour by default)
	T interpolated(const float u, const float v, const int layer = 0) const
	{		
		const int x = (int(round(u)) < 0) ? 0 : (int(round(u)) >= width ? width - 1 : int(round(u)));
		const int y = (int(round(v)) < 0) ? 0 : (int(round(v)) >= height ? height - 1 : int(round(v)));
		return value(x, y, layer);		
	}

	~simple_image()
	{
		if (allocated)
		{
			delete[] buffer;
		}
	}

private:
	simple_image(simple_image<T> &copy) : buffer(nullptr), width(0), height(0), layers(0), HW(0), allocated(false)
	{

	}
};

// for floating point image interpolation is bi-linear
template<> float simple_image<float>::interpolated(const float u, const float v, const int layer) const
{
	const float x = (u < 0.f) ? 0.f : (u > width - 1.f ? width - 1.f : u);
	const float y = (v < 0.f) ? 0.f : (v > height - 1.f ? height - 1.f : v);
	
	const int xl = int(floor(x));
	const int xr = int(ceil(x));
	const int yu = int(floor(y));
	const int yd = int(ceil(y));
	const float dy = float(yd) - y;
	const float dx = float(xr) - x;
	
	const float h1 = value(xl, yu, layer) * dy + value(xl, yd, layer) * (1.f - dy);
	const float h2 = value(xr, yu, layer) * dy + value(xr, yd, layer) * (1.f - dy);
	return (h1 * dx + h2 * (1.f - dx) + 0.5f);		
}

// for uint8_t images we also have bi-linear interpolation
template<> uint8_t simple_image<uint8_t>::interpolated(const float u, const float v, const int layer) const
{
	const float x = (u < 0.f) ? 0.f : (u > width - 1.f ? width - 1.f : u);
	const float y = (v < 0.f) ? 0.f : (v > height - 1.f ? height - 1.f : v);

	const int xl = int(floor(x));
	const int xr = int(ceil(x));
	const int yu = int(floor(y));
	const int yd = int(ceil(y));
	const float dy = float(yd) - y;
	const float dx = float(xr) - x;
	const uint8_t ld = value(xl, yd, layer);
	const uint8_t lu = value(xl, yu, layer);
	const uint8_t rd = value(xr, yd, layer);
	const uint8_t ru = value(xr, yu, layer);

	const float h1 = float(lu) * dy + float(ld) * (1.f - dy);
	const float h2 = float(ru) * dy + float(rd) * (1.f - dy);
	
	return uint8_t(round(h1 * dx + h2 * (1.f - dx)));
}

//
//// for Rgba images we also have bi-linear interpolation
//template<> Rgba simple_image<Rgba>::interpolated(const float u, const float v, const int layer) const
//{
//	Rgba rt;
//	
//	const float x = (u < 0.f) ? 0.f : (u > width - 1.f ? width - 1.f : u);
//	const float y = (v < 0.f) ? 0.f : (v > height - 1.f ? height - 1.f : v);
//	
//	const int xl = floor(x);
//	const int xr = ceil(x);
//	const int yu = floor(y);
//	const int yd = ceil(y);
//	const float dy = yd - y;
//	const float dx = xr - x;
//	const Rgba ld = value(xl, yd, layer);
//	const Rgba lu = value(xl, yu, layer);
//	const Rgba rd = value(xr, yd, layer);
//	const Rgba ru = value(xr, yu, layer);
//
//	const float h1r = float(lu.ch.r) * dy + float(ld.ch.r) * (1 - dy);
//	const float h1g = float(lu.ch.g) * dy + float(ld.ch.g) * (1 - dy);
//	const float h1b = float(lu.ch.b) * dy + float(ld.ch.b) * (1 - dy);
//	const float h1a = float(lu.ch.a) * dy + float(ld.ch.a) * (1 - dy);
//	
//	const float h2r = float(ru.ch.r) * dy + float(rd.ch.r) * (1 - dy);
//	const float h2g = float(ru.ch.g) * dy + float(rd.ch.g) * (1 - dy);
//	const float h2b = float(ru.ch.b) * dy + float(rd.ch.b) * (1 - dy);
//	const float h2a = float(ru.ch.a) * dy + float(rd.ch.a) * (1 - dy);
//
//	rt.ch.r = uint8_t(round(h1r * dx + h2r * (1 - dx)));
//	rt.ch.g = uint8_t(round(h1g * dx + h2g * (1 - dx)));
//	rt.ch.b = uint8_t(round(h1b * dx + h2b * (1 - dx)));
//	rt.ch.a = uint8_t(round(h1a * dx + h2a * (1 - dx)));
//
//	return rt;
//}


class integral_image
{
private:
	float * const buffer;
	const bool allocated;
	bool integrated;
	float *averages;

public:

	const int width;
	const int height;
	const int layers;
	const long HW;

	integral_image(const simple_image<float> &image)
		: buffer(new float[image.width*image.height*image.layers]), allocated(false),
		width(image.width), height(image.height), layers(image.layers), HW(image.width*image.height),
		averages(new float[image.layers])
	{
		update(image);
	}

	integral_image(const int _width, const int _height, const int _layers) : 
		buffer(new float[_width*_height*_layers]), allocated(true),
		width(_width), height(_height), layers(_layers), HW(_width*_height),
		averages(new float[_layers])
	{
		integrated = false;
	}

	void update(const simple_image<float> &image)
	{
		clear();

		#pragma omp parallel for
		for (int l = 0; l < layers; l++)
		{
			averages[l] = float(layer_average(image, l));
			integrate(image, l);
		}
	}

	~integral_image()
	{
		delete[] averages;
		if (allocated)
		{
			delete[] buffer;
		}
	}

	float average(int x, int y, int radius, int layer = 0)
	{
		int x_low = x - radius - 1;
		int x_up = x + radius;
		int y_low = y - radius - 1;
		int y_up = y + radius;

		x_low = (x_low<0) ? 0 : x_low;
		y_low = (y_low<0) ? 0 : y_low;
		x_up = (x_up >= width) ? width - 1 : x_up;
		y_up = (y_up >= height) ? height - 1 : y_up;
		
		const int size = (x_up - x_low)*(y_up - y_low);

		float avg = value(x_low, y_low, layer);
		avg += value(x_up, y_up, layer);
		avg -= value(x_up, y_low, layer);
		avg -= value(x_low, y_up, layer);
		avg /= size;

		return avg + averages[layer];
	}

private:

	void clear()
	{
		integrated = false;
		#pragma omp parallel for
		for (long i = 0; i < HW*layers; i++)
		{
			buffer[i] = 0.f;
		}
	}

	double layer_average(const simple_image<float> &image, const int l = 0) const
	{
		double value = 0.0;
		#pragma omp parallel for reduction (+:value)
		for (long i = 0; i < image.HW; i++)
		{			
			const int x = i % image.width; 
			const int y = i / image.width;			
			value += double(image(x, y, l));
		}
		return value / image.HW;
	}

	void integrate(const simple_image<float> & image, const int layer)
	{		
		#pragma omp parallel for
		for (int l = 0; l < layers; l++)
		{
			float avg = averages[l];
			for (int y = 0; y < height; y++)
			{
				for (int x = 0; x < width; x++)
				{
					value(x, y, l) += (image(x, y, l) - avg);
					if (x > 0 && y > 0)
					{
						value(x, y, l) += image(x, y - 1, l);
						value(x, y, l) += image(x - 1, y, l);
						value(x, y, l) -= image(x - 1, y - 1, l);
					}
					else if (x > 0)
					{
						value(x, y, l) += image(x - 1, y, l);
					}
					else if (y > 0)
					{
						value(x, y, l) += image(x, y - 1, l);
					}
				}
			}
		}

		integrated = true;
	}

	float& value(const int x, const int y, const int l = 0)
	{
		return buffer[l*HW + y * width + x];
	}

	integral_image(integral_image &copy) : buffer(nullptr), allocated(false),
		width(0), height(0), layers(0), HW(0), averages(nullptr)
	{

	}

};


