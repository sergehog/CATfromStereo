#include <atomic>
#include <iostream>
#include <chrono>

#include "../common/vimba_helper.h"
#include "../common/simple_image.h"

#pragma once


// frame observer that reacts on new frames
class FrameObserver : public IFrameObserver
{
private:
	//VmbUchar_t * const buffer; // frame buffer
	//const float * const lookup; // optical distortions lookup table
	//const simple_image<const float> & Lookup;
	//simple_image<Rgba> Output;
	//std::weak_ptr<simple_image<Rgba>>  BufferPtr;
	//std::weak_ptr<const simple_image<const float>> LookupPtr;
	const float * const lookup;
	VmbUchar_t * const buffer;
	const simple_image<float> gradient;
	const int width, height;
	


public:
	
	//// frame buffer and lookup are transmitted here
	//FrameObserver(CameraPtr pCamera, VmbUchar_t * const _buffer, const float * const _lookup) : IFrameObserver(pCamera), buffer(_buffer), lookup(_lookup)
	//{

	//}

	//FrameObserver(CameraPtr pCamera, std::weak_ptr<simple_image<Rgba>>  _BufferPtr, std::weak_ptr<const simple_image<const float>> _LookupPtr) : IFrameObserver(pCamera), BufferPtr(_BufferPtr), LookupPtr(_LookupPtr)
	FrameObserver(CameraPtr pCamera, VmbUchar_t * const _Buffer, const float * const _Lookup, const int _width, const int _height) : IFrameObserver(pCamera), 
		buffer(_Buffer), lookup(_Lookup), width(_width), height(_height), gradient(_width, _height, 2)
	{
		
	}

	FramePtr LastFrame;
	std::atomic<bool> frame_arrived;

	// 
	void FrameReceived(const FramePtr pFrame)
	{
		std::string camid;
		VmbUint64_t frameid;
		m_pCamera->GetID(camid);
		pFrame->GetFrameID(frameid);
		//std::cout << time(NULL) << " " << camid << ": " << frameid << std::endl;
		VmbUint32_t width, height, imsize;
		VmbUchar_t * _buffer1;
		pFrame->GetWidth(width);
		pFrame->GetHeight(height);
		pFrame->GetImage(_buffer1);
		pFrame->GetImageSize(imsize);
		simple_image<VmbUchar_t> Input(_buffer1, width, height, 1);
		const simple_image<const float> Lookup(lookup, width, height, 2);
		simple_image<Rgba> Output((Rgba*)buffer, width, height);		
			
		#pragma omp parallel for
		for (int y = 0; y < int(height); y++)
		{
			const int y_up = y > 0 ? y - 1 : 0;
			const int y_down = y < height - 1 ? y + 1 : height - 1;

			for (int x = 0; x < int(width); x++)
			{
				const int x_left = x > 0 ? x - 1 : 0;
				const int x_right = y < width - 1 ? x + 1 : width - 1;
				gradient(x, y, 0) = float(int(Input(x_right, y)) - int(Input(x_left, y)));
				gradient(x, y, 1) = float(int(Input(x, y_down)) - int(Input(x, y_up)));
			}
		}
		
		float average = 0.f;
		#pragma omp parallel for reduction(+:average)
		for (int v = 0; v < int(height); v++)
		{
			//const long row = v * width * 3;
			for (int u = 0; u < int(width); u++)
			{				
				const float x = Lookup(u, v, 0);
				const float y = Lookup(u, v, 1);				

				Rgba a;
				a.ch.r = Input.interpolated(x, y);
				float gx = gradient.interpolated(x, y, 0) + 127.f;
				gx = gx < 0.f ? 0.f : (gx > 255.f ? 255.f : gx);
				float gy = gradient.interpolated(x, y, 1) + 127.f;
				gy = gy < 0.f ? 0.f : (gy > 255.f ? 255.f : gy);
				a.ch.g = uint8_t(gx + 0.5);
				a.ch.b = uint8_t(gy + 0.5);
				a.ch.a = 255;
				Output(u, v) = a;				
			}
		}		
		
		LastFrame = pFrame;
		frame_arrived = true;
		

		// When you are finished copying the frame , re-queue it
		//m_pCamera->QueueFrame(pFrame);
	}
};

