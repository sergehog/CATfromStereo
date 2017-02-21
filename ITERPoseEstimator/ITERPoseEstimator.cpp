/*	ITERStereoEstimator.cpp - Main program file for ITER Pose Estimator app
@author Sergey Smirnov sergei.smirnov@gmail.com
@date 2016
*/

#define GLEW_STATIC
#include <GL/glew.h>
#undef APIENTRY
#define GLFW_STATIC
//GLFW_DLL
#include <GLFW/glfw3.h>

#include <VimbaCPP/Include/VimbaCPP.h>
#include <VimbaImageTransform/Include/VmbTransform.h>

#ifndef _DEBUG
#define GLM_FORCE_SSE2 
#endif
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <string>
#include <memory>
#include <future>
#include <chrono>
#include <thread>
#include <mutex>
#include <cmath>
#include <omp.h>	
#include "../common/shaders.h"
#include "../common/config_iter.h"
#include "../common/camera_loader.h"
#include "../common/simple_image.h"
#include "../common/vimba_helper.h"
#include "frame_observer.h"

using namespace AVT::VmbAPI;
using namespace std::chrono_literals;
using namespace glm;

void checkGLError(const char * a);
std::unique_ptr<float[]> prepare_lookup2(const glm::mat3 K1, const float cam1_k1, const float cam1_k2, const int width, const int height);
std::shared_ptr<const simple_image<const float>> prepare_lookup3(const glm::mat3 K1, const float cam1_k1, const float cam1_k2, const int width, const int height);


void calculateCost(std::shared_ptr<const simple_image<Rgba>> Image1Ptr, std::shared_ptr<const simple_image<Rgba>> Image2Ptr, std::shared_ptr<simple_image<float>> CostPtr, glm::mat4 C2C1inv, const float minZ, const float maxZ, const float max_cost = 30.f)
{
		
	//std::unique_ptr<float[]> zlayers = std::unique_ptr<float[]>(new float[CostPtr->layers]);
	
	#pragma omp parallel for
	for (int l = 0; l < CostPtr->layers; l++)
	{
		const float z = 1.f / ((float(l) / CostPtr->layers)*(1.f / minZ - 1.f / maxZ) + 1.f / maxZ);
		for (int v = 0; v < Image1Ptr->height; v++)
		{
			for (int u = 0; u < Image1Ptr->width; u++)
			{
				Rgba c1 = Image1Ptr->value(u, v);

				//for (int l = 0; l < CostPtr->layers; l++)
				{
					glm::vec4 uvz1 = glm::vec4(u*z, v*z, z, 1.0);
					glm::vec4 uvz2 = C2C1inv * uvz1;
					const float u2 = uvz2.x / uvz2.z;
					const float v2 = uvz2.y / uvz2.z;
					Rgba c2 = Image2Ptr->interpolated(u2, v2);
					const float cost = float(abs(int(c1.ch.r) - int(c2.ch.r)) + abs(int(c1.ch.g) - int(c2.ch.g)) + abs(int(c1.ch.b) - int(c2.ch.b)));
					CostPtr->value(u, v, l) = cost > max_cost ? 1.f : cost / max_cost;
				}
			}
		}
	}
}


void wta_simple(std::shared_ptr<simple_image<float>> CostPtr, std::shared_ptr<simple_image<uint8_t>> DepthPtr)
{
	DepthPtr->set(0.f);

	#pragma omp parallel for
	for (long i=0; i<DepthPtr->HW; i++)
	{
		const int x = i % DepthPtr->width;
		const int y = i / DepthPtr->width;

		for (int l = 1; l < CostPtr->layers; l++)
		{
			if (CostPtr->value(x, y, l) < CostPtr->value(x, y, 0))
			{
				DepthPtr->value(x, y) = uint8_t(l);
				CostPtr->value(x, y, 0) = CostPtr->value(x, y, l);
			}
		}
	}	
}

int main(int argc, char* argv[])
{
	omp_set_num_threads(omp_get_max_threads());
	omp_set_dynamic(omp_get_max_threads() - 1);

	VimbaSystem &system = VimbaSystem::GetInstance();

	//int ret_code = 0;	

	try
	{
		if (argc < 2)
		{
			throw std::exception((std::string("CORRECT USAGE: ") + argv[0] + " config-file.xml").c_str());
		}

		const static config::config_iter config = config::config_iter::load_config(argv[1]);
		const unsigned width = config.input_width;
		const unsigned height = config.input_height;
		const unsigned long HW = width * height;

		float cam1_k1 = 0.f, cam1_k2 = 0.f, cam2_k1 = 0.f, cam2_k2 = 0.f;
		const std::pair<const mat3, const mat4x3> camera1_pair = config::camera_loader::read_settings2(config.camera_file, config.camera1_name, cam1_k1, cam1_k2);
		const std::pair<const mat3, const mat4x3> camera2_pair = config::camera_loader::read_settings2(config.camera_file, config.camera2_name, cam2_k1, cam2_k2);

		glm::mat4 C1(camera1_pair.first * camera1_pair.second);
		glm::mat4 C2(camera2_pair.first * camera2_pair.second);
		glm::mat4 C2C1inv = C2*glm::inverse(C1);		
		//glm::mat4 C1C2inv = C1*glm::inverse(C2);		

		
		//std::unique_ptr<float[]> lookup1 = prepare_lookup2(camera1_pair.first, cam1_k1, cam1_k2, config.frame_width, config.frame_height);
		//std::unique_ptr<float[]> lookup2 = prepare_lookup2(camera2_pair.first, cam2_k1, cam2_k2, config.frame_width, config.frame_height);
		//const simple_image<const float> Lookup1(lookup1.get(), config.frame_width, config.frame_height, 2);
		//const simple_image<const float> Lookup2(lookup2.get(), config.frame_width, config.frame_height, 2);

		simple_image<float>  Cost1(config.frame_width, config.frame_height, config.layers);
		integral_image  Integral1(config.frame_width, config.frame_height, config.layers);
		simple_image<uint8_t>  Depth1(config.frame_width, config.frame_height);
		simple_image<Rgba> Image1(config.frame_width, config.frame_height, 1);
		simple_image<Rgba> Image2(config.frame_width, config.frame_height, 1);

		//std::unique_ptr<VmbUchar_t[]> buffer1(new VmbUchar_t[PayloadSize * 4]);
		//std::unique_ptr<VmbUchar_t[]> buffer2(new VmbUchar_t[PayloadSize * 4]);
		//simple_image<Rgba> Image1((Rgba*)buffer1.get(), config.frame_width, config.frame_height, 1);
		//simple_image<Rgba> Image2((Rgba*)buffer2.get(), config.frame_width, config.frame_height, 1);


		
		std::shared_ptr<simple_image<float>> Cost1Ptr(&Cost1);
		std::shared_ptr<integral_image> Integral1Ptr(&Integral1);
		std::shared_ptr<simple_image<uint8_t>> Depth1Ptr(&Depth1);
		std::shared_ptr<simple_image<Rgba>> Image1Ptr(&Image1);
		std::shared_ptr<simple_image<Rgba>> Image2Ptr(&Image2);


		checkStatus(system.Startup());

		// Initialise GLFW	
		if (!glfwInit())
		{
			throw std::exception("Failed to initialize GLFW");
		}

		glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		//glfwWindowHint(GLFW_SAMPLES, config.fsaa);

		
		GLFWwindow* window = glfwCreateWindow(config.screen_width, config.screen_height, "ITER Pose Estimator", NULL, NULL);

		// Open a window and create its OpenGL context		
		if (!window)
		{
			glfwTerminate();
			throw std::exception("Failed to open GLFW window.");
		}

		glfwMakeContextCurrent(window);
		checkGLError("OpenGL Window Opening");

		int screen_width, screen_height;
		glfwGetWindowSize(window, &screen_width, &screen_height);
		//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
		glfwSwapInterval(-1);

		// Initialize GLEW
		glewExperimental = true; // Needed for core profile
		if (glewInit() != GLEW_OK)
		{
			throw std::exception("Failed to initialize GLEW.");
		}

		// Ensure we can capture the escape key being pressed below
		glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

		// Crappy color to clearly see non-rendered areas
		//glClearColor(0.9f, 0.9f, 0.0f, 0.0f);

		// Enable depth test		
		//glEnable(GL_DEPTH_TEST);
		// Accept fragment if it closer to the camera than the former one
		//glDepthFunc(GL_LESS);
		checkGLError("GLEW Initializatiopn");

		const GLuint programID = config.geometry_shader.empty() ? LoadShaders(config.vertex_shader.c_str(), config.fragment_shader.c_str()) : Load3Shaders(config.vertex_shader.c_str(), config.geometry_shader.c_str(), config.fragment_shader.c_str());
		glUseProgram(programID);
		//glUniform1f(glGetUniformLocation(programID, "minZ"), config.minZ);
		//glUniform1f(glGetUniformLocation(programID, "maxZ"), config.maxZ);
		//glUniform1ui(glGetUniformLocation(programID, "layers"), config.layers);
		glUniform1ui(glGetUniformLocation(programID, "width"), config.frame_width);
		glUniform1ui(glGetUniformLocation(programID, "height"), config.frame_height);
		//glUniformMatrix4fv(glGetUniformLocation(programID, "C2C1inv"), 1, GL_FALSE, &C2C1inv[0][0]);

		//const GLuint mainProgramID = LoadShaders(config.main_vertex_shader.c_str(), config.main_fragment_shader.c_str());
		//glUseProgram(mainProgramID);
		//glUniform1f(glGetUniformLocation(mainProgramID, "minZ"), config.minZ);
		//glUniform1f(glGetUniformLocation(mainProgramID, "maxZ"), config.maxZ);
		//glUniform1ui(glGetUniformLocation(mainProgramID, "layers"), config.layers);
		//glUniform1ui(glGetUniformLocation(mainProgramID, "width"), config.frame_width);
		//glUniform1ui(glGetUniformLocation(mainProgramID, "height"), config.frame_height);
		//checkGLError("GLSL shader programs loading");

		checkGLError("Shaders loading.");

		//GLuint lookup1TextureID = prepare_lookup(camera1_pair.first, cam1_k1, cam1_k2, config.frame_width, config.frame_height);
		//GLuint lookup2TextureID = prepare_lookup(camera2_pair.first, cam2_k1, cam2_k2, config.frame_width, config.frame_height);


		//GLuint layerID = glGetUniformLocation(programID, "layer");
		//GLuint mainLayerID = glGetUniformLocation(mainProgramID, "layer");

		GLuint vao;
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);

		//// The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
		//GLuint framebufferID = 0;
		//glGenFramebuffers(1, &framebufferID);
		//glBindFramebuffer(GL_FRAMEBUFFER, framebufferID);

		//// The texture we're going to render to
		//GLuint renderedTextureID;
		//glGenTextures(1, &renderedTextureID);

		//// "Bind" the newly created texture : all future texture functions will modify this texture
		//glBindTexture(GL_TEXTURE_2D_ARRAY, renderedTextureID);

		//// Texture filtering
		//glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		//glTexStorage3D(GL_TEXTURE_2D_ARRAY, 10, GL_RGB32F, config.frame_width, config.frame_height, config.layers);
		////glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RED, config.frame_width, config.frame_height*2, 0, GL_RED, GL_UNSIGNED_BYTE, 0);

		//// Set "renderedTexture" as our colour attachement #0
		//glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTextureID, 0);
		//checkGLError("Render-to-Texture Framebuffer creation");

		////GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
		//GLenum drawBuffer = GL_COLOR_ATTACHMENT0;
		//glDrawBuffers(1, &drawBuffer); // "1" is the size of DrawBuffers

		//							   // check either our framebuffer is ok
		//if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		//{
		//	throw std::exception("Framebuffer is not OK!");
		//}
		cout << "1" << std::endl;
		// Two triangles are always the same, so can be initialized just one in advance				
		GLuint elementbuffer, uvbuffer;
		GLuint index_buffer[] = { 0, 1, 3, 1, 2, 3 };
		GLint uv_buffer[] = { 0, 0, GLint(config.frame_width), 0, GLint(config.frame_width), GLint(config.frame_height), 0, GLint(config.frame_height) };
		glGenBuffers(1, &elementbuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(index_buffer), index_buffer, GL_STATIC_DRAW);

		glGenBuffers(1, &uvbuffer);
		glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(uv_buffer), uv_buffer, GL_STATIC_DRAW);

		GLuint texture1ID, texture2ID;
		glGenTextures(1, &texture1ID);
		glBindTexture(GL_TEXTURE_2D, texture1ID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

		glGenTextures(1, &texture2ID);
		glBindTexture(GL_TEXTURE_2D, texture2ID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		checkGLError("Buffers and Textures init");

		GLuint depth1ID, depth2ID;
		glGenTextures(1, &depth1ID);
		glBindTexture(GL_TEXTURE_2D, depth1ID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

		glGenTextures(1, &depth2ID);
		glBindTexture(GL_TEXTURE_2D, depth2ID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

		cout << "2" << std::endl;
		CameraPtr camera1, camera2;
		FeaturePtr feature1, feature2;

		checkStatus(system.OpenCameraByID(config.camera1_ip.c_str(), VmbAccessModeFull, camera1));
		checkStatus(system.OpenCameraByID(config.camera2_ip.c_str(), VmbAccessModeFull, camera2));
		cout << "2a" << std::endl;
		setCamera(camera1, feature1, config.frame_width, config.frame_height);
		setCamera(camera2, feature2, config.frame_width, config.frame_height);
		cout << "2b" << std::endl;

		VmbInt64_t PayloadSize = readFeature(camera1, feature1, "PayloadSize");


		//simple_image<uint8_t> Buffer1(buffer1.get(), config.frame_width, config.frame_height, 1);
		//simple_image<uint8_t> Buffer2(buffer1.get(), config.frame_width, config.frame_height, 1);

		//std::shared_ptr<const simple_image<const float>> Lookup1Ptr = prepare_lookup3(camera1_pair.first, cam1_k1, cam1_k2, config.frame_width, config.frame_height);
		//std::shared_ptr<const simple_image<const float>> Lookup2Ptr = prepare_lookup3(camera2_pair.first, cam2_k1, cam2_k2, config.frame_width, config.frame_height);
		// these will be destroyed at the end of a app
		std::unique_ptr<float[]> lookup1 = prepare_lookup2(camera1_pair.first, cam1_k1, cam1_k2, config.frame_width, config.frame_height);
		std::unique_ptr<float[]> lookup2 = prepare_lookup2(camera2_pair.first, cam2_k1, cam2_k2, config.frame_width, config.frame_height);

		IFrameObserverPtr pObserver1(new FrameObserver(camera1, Image1.buffer, lookup1.get()));
		IFrameObserverPtr pObserver2(new FrameObserver(camera2, Image1.buffer, lookup2.get()));


		setFeature(camera1, feature1, "AcquisitionMode", "Continuous");
		setFeature(camera1, feature1, "TriggerSource", "Software");
		setFeature(camera1, feature1, "SyncOutSource", "Imaging");


		setFeature(camera2, feature2, "AcquisitionMode", "Continuous");
		setFeature(camera2, feature2, "TriggerSource", "Line1");

		cout << "3" << std::endl;

		// A list of frames for streaming. We chose to queue 2 frames.		
		FramePtrVector frames1(config.buffer_frame_number), frames2(config.buffer_frame_number);
		//for(unsigned i=0; i<config.buffer_frame_number; i++)
		{
			unsigned i = 0;
			frames1.at(i).reset(new Frame(PayloadSize));
			frames1.at(i)->RegisterObserver(pObserver1);
			camera1->AnnounceFrame(frames1.at(i));

			frames2.at(i).reset(new Frame(PayloadSize));
			frames2.at(i)->RegisterObserver(pObserver2);
			camera2->AnnounceFrame(frames2.at(i));
		}

		((FrameObserver*)pObserver1.get())->LastFrame = frames1[0];
		((FrameObserver*)pObserver2.get())->LastFrame = frames2[0];


		camera1->StartCapture();
		camera2->StartCapture();

		for (unsigned i = 0; i < config.buffer_frame_number; i++)
		{
			camera1->QueueFrame(frames1.at(i));
			camera2->QueueFrame(frames2.at(i));
		}

		cout << "4" << std::endl;
		runCommand(camera1, feature1, "AcquisitionStart");
		runCommand(camera2, feature2, "AcquisitionStart");

		((FrameObserver*)pObserver1.get())->frame_arrived = false;
		((FrameObserver*)pObserver2.get())->frame_arrived = false;
		runCommand(camera1, feature1, "TriggerSoftware");
		time_t triggered = time(NULL);
		std::cout << triggered << " Triggered Frames" << std::endl;
		//glBindFramebuffer(GL_FRAMEBUFFER, 0);
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//glUseProgram(mainProgramID);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

		//int layerz = 0;
		int frameNum = 0;
		
		glUseProgram(programID);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		checkGLError("Before Cycle");
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);			
		cout << "5" << std::endl;
		//std::unique_ptr<float[]> CostVolume1 = std::unique_ptr<float[]>(new float[width*height*config.layers]);
		//std::unique_ptr<float[]> CostVolume2 = std::unique_ptr<float[]>(new float[width*height*config.layers]);


		while (!glfwWindowShouldClose(window) && glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS)
		{
			checkGLError("Iteration #1");
			std::cout << "GL Frame " << frameNum++ << std::endl;

			// Clear both framebuffers at the beginning of a frame
			//glBindFramebuffer(GL_FRAMEBUFFER, framebufferID);
			//glViewport(0, 0, config.frame_width, config.frame_height*2);
			//glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);			
			//glViewport(0, 0, config.frame_width, config.frame_height);

			//glUseProgram(mainProgramID);

			//glBindFramebuffer(GL_FRAMEBUFFER, 0);
			//glViewport(0, 0, config.frame_width, config.frame_height*2);

			//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			//glUseProgram(mainProgramID);


			// wait for both frames, re-trigger if no answer for long time
			while (!((FrameObserver*)pObserver1.get())->frame_arrived || !((FrameObserver*)pObserver2.get())->frame_arrived)
			{
				if (difftime(time(NULL), triggered) > 0.5)
				{
					std::cout << triggered << " Re-Triggering Frames due to long delay" << std::endl;
					FramePtr frame1 = ((FrameObserver*)pObserver1.get())->LastFrame;
					FramePtr frame2 = ((FrameObserver*)pObserver2.get())->LastFrame;
					camera1->QueueFrame(frame1);
					camera2->QueueFrame(frame2);
					((FrameObserver*)pObserver1.get())->frame_arrived = false;
					((FrameObserver*)pObserver2.get())->frame_arrived = false;
					runCommand(camera1, feature1, "TriggerSoftware");
					triggered = time(NULL);
					
				}
				std::this_thread::sleep_for(1ms);
			}

			FramePtr frame1 = ((FrameObserver*)pObserver1.get())->LastFrame;
			FramePtr frame2 = ((FrameObserver*)pObserver2.get())->LastFrame;


			//VmbUchar_t * _buffer1;
			//frame1->GetImage(_buffer1);

			//VmbUchar_t * _buffer2;
			//frame2->GetImage(_buffer2);
						
			
			calculateCost(Image1Ptr, Image2Ptr, Cost1Ptr, C2C1inv, config.minZ, config.maxZ);
			
			Integral1.update(*Cost1Ptr.get());
			#pragma omp parallel for
			for (int l = 0; l < config.layers; l++)
			{
				for (int y = 0; y < config.frame_height; y++)
				{
					for (int x = 0; x < config.frame_width; x++)
					{
						Cost1Ptr->value(x, y, l) = Integral1.average(x, y, 5, l);
					}
				}
			}
			wta_simple(Cost1Ptr, Depth1Ptr);


			//glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1ID);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, config.input_width, config.input_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, Image1.buffer);
			glGenerateMipmap(GL_TEXTURE_2D);

			//glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, texture2ID);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, config.input_width, config.input_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, Image2.buffer);
			glGenerateMipmap(GL_TEXTURE_2D);

			glBindTexture(GL_TEXTURE_2D, depth1ID);			
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, config.input_width, config.input_height, 0, GL_RED, GL_FLOAT, Cost1.buffer+HW);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, config.input_width, config.input_height, 0, GL_RED, GL_UNSIGNED_BYTE, Depth1.buffer);
			glGenerateMipmap(GL_TEXTURE_2D);

			checkGLError("Iteration #2");

			camera1->QueueFrame(frame1);
			camera2->QueueFrame(frame2);

			((FrameObserver*)pObserver1.get())->frame_arrived = false;
			((FrameObserver*)pObserver2.get())->frame_arrived = false;

			runCommand(camera1, feature1, "TriggerSoftware");
			triggered = time(NULL);
			std::cout << triggered << " Triggered Frames normally" << std::endl;


			
			glUseProgram(programID);
			checkGLError("Iteration #3");

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1ID);

			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, texture2ID);

			glActiveTexture(GL_TEXTURE2);
			glBindTexture(GL_TEXTURE_2D, depth1ID);

			glActiveTexture(GL_TEXTURE3);
			glBindTexture(GL_TEXTURE_2D, depth2ID);

			checkGLError("Iteration #4");

			glEnableVertexAttribArray(0);
			checkGLError("Iteration #5a");
			glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
			checkGLError("Iteration #5b");
			glVertexAttribIPointer(0, 2, GL_INT, 0, (void*)0);
			checkGLError("Iteration #5c");
			glUniform1i(glGetUniformLocation(programID, "Texture1"), 0);
			glUniform1i(glGetUniformLocation(programID, "Texture2"), 1);
			glUniform1i(glGetUniformLocation(programID, "Depth1"), 2);
			glUniform1i(glGetUniformLocation(programID, "Depth2"), 3);
			

			checkGLError("Iteration #6");

			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (void*)0);			
			

			glDisableVertexAttribArray(0);

			
			checkGLError("Iteration End");
			
			glfwSwapBuffers(window);
			glfwPollEvents();

		}

		glDisableVertexAttribArray(0);
		glDeleteBuffers(1, &uvbuffer);
		glDeleteBuffers(1, &elementbuffer);
		glDeleteVertexArrays(1, &vao);
		glDeleteTextures(1, &texture1ID);
		glDeleteTextures(1, &texture2ID);
		//glDeleteTextures(1, &renderedTextureID);
		glDeleteProgram(programID);
		//glDeleteProgram(mainProgramID);
		//glDeleteFramebuffers(1, &framebufferID);

		runCommand(camera1, feature1, "AcquisitionStop");
		runCommand(camera2, feature2, "AcquisitionStop");

		camera1->EndCapture();
		camera1->FlushQueue();
		camera1->RevokeAllFrames();
		camera1->Close();

		camera2->EndCapture();
		camera2->FlushQueue();
		camera2->RevokeAllFrames();
		camera2->Close();

		pObserver1.reset();
		pObserver1.reset();

		//std::this_thread::sleep_for(1s);

		system.Shutdown();
	}
	catch (std::exception ex)
	{
		std::cout << "Error Occured " << std::endl;
		std::cout << ex.what() << std::endl;
		system.Shutdown();
		return -1;
	}


	return 0;
}



void checkGLError(const char * a)
{
	GLenum err = glGetError();
	if (err != GL_NO_ERROR)
	{
		char errTxt[400];
		//printf("%d. OpenGL Error 0x%.4x!\n", a, err);
		sprintf_s(errTxt, "%s. OpenGL Error 0x%.4x!", a, err);
		throw std::exception(errTxt);
	}
}


std::shared_ptr<const simple_image<const float>> prepare_lookup3(const glm::mat3 K1, const float cam1_k1, const float cam1_k2, const int width, const int height)
{

	const float cam1_ox = K1[2][0];
	const float cam1_oy = K1[2][1];
	const float cam1_fx = K1[0][0];
	const float cam1_fy = K1[1][1];

	const long HW = width * height;
	std::shared_ptr<const simple_image<const float>> lookupPtr(new simple_image<const float>(width, height, 2));
	//std::unique_ptr<float[]> lookup = std::unique_ptr<float[]>(new float[width*height * 2]);

	float * const lookup_table = (float *)lookupPtr->buffer;	

	#pragma omp parallel for
	for (int yi = 0; yi < height; yi++)
	{
		const long row = yi*width;
		for (int xi = 0; xi < width; xi++)
		{
			float x1 = (xi - cam1_ox) / cam1_fx;
			float y1 = (yi - cam1_oy) / cam1_fy;
			const float r1 = (x1*x1 + y1*y1);
			x1 = x1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
			y1 = y1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
			lookup_table[row + xi] = (x1*cam1_fx + cam1_ox);
			lookup_table[row + xi + HW] = (y1*cam1_fy + cam1_oy);
		}
	}

	return lookupPtr;
}


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
			float x1 = (xi - cam1_ox) / cam1_fx;
			float y1 = (yi - cam1_oy) / cam1_fy;
			const float r1 = (x1*x1 + y1*y1);
			x1 = x1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
			y1 = y1 * (1 + cam1_k1*r1 + cam1_k2*r1*r1);
			lookup_table[row + xi] = (x1*cam1_fx + cam1_ox);
			lookup_table[row + xi + HW] = (y1*cam1_fy + cam1_oy);		
		}
	}

	return lookup;
}

//
//void calculateCostVolume(VmbUchar_t * const buffer1, VmbUchar_t * const buffer2, glm::mat4 C2C1inv, const int width, const int height, const int layers, const float minZ, const float maxZ, float * const Cost);
//

////void checkStatus(const int status);
//
//
//
//
//// creates a cost volume
//void aggregateCostVolume(const int width, const int height, const int layers, float * const Cost, float * const Buffer)
//{
//	const long HW = width * height;
//#pragma omp parallel for
//	for (long i = 0; i < HW*layers; i++)
//	{
//		Buffer[i] = 0;
//	}
//
//	float average = 0.0;
//
//#pragma omp parallel for reduction(+:average)
//	for (long i = 0; i < HW*layers; i++)
//	{
//		average += Cost[i];
//	}
//	average /= HW*layers;
//
//#pragma omp parallel for
//	for (int l = 0; l < layers; l++)
//	{
//		for (int y = 0; y < height; y++)
//		{
//			for (int x = 0; x < width; x++)
//			{
//				Buffer[HW*l + y*width + x] += (Cost[HW*l + y*width + x] - average);
//				if (x > 0 & y > 0)
//				{
//					Buffer[HW*l + y*width + x] += Buffer[HW*l + (y - 1)*width + x];
//					Buffer[HW*l + y*width + x] += Buffer[HW*l + y*width + x - 1];
//					Buffer[HW*l + y*width + x] -= Buffer[HW*l + (y - 1)*width + x - 1];
//				}
//				else if (x > 0)
//				{
//					Buffer[HW*l + y*width + x] += Buffer[HW*l + (y - 1)*width + x];
//				}
//				else if (y > 0)
//				{
//					Buffer[HW*l + y*width + x] += Buffer[HW*l + y*width + x - 1];
//				}
//			}
//		}
//	}
//
//
//}
//
//
//
//template<unsigned N>
//// Returns a color value taking into account optical distortion
//color<N> getInterpolatedColor(VmbUchar_t * const buffer, const float u, const float v, const int width, const int height)
//{
//	const int HW = height * width;
//	color<N> value;
//	color<N> h1;
//	color<N> h2;
//	float x = u;
//	float y = v;
//	x = (x < 0.f) ? 0.f : (x > width - 1.f ? width - 1.f : x);
//	y = (y < 0.f) ? 0.f : (y > height - 1.f ? height - 1.f : y);
//	const int xl = floor(x);
//	const int xr = ceil(x);
//	const int yu = floor(y);
//	const int yd = ceil(y);
//	const float dy = yd - y;
//	const float dx = xr - x;
//#pragma omp parallel for 
//	for (int n = 0; n < N; n++)
//	{
//		const float h1 = float(buffer[yd*width * 3 + xl * 3 + n]) * dy + float(buffer[yu*width * 3 + xl * 3 + n]) * (1 - dy);
//		const float h2 = float(buffer[yd*width * 3 + xr * 3 + n]) * dy + float(buffer[yu*width * 3 + xr * 3 + n]) * (1 - dy);
//		value.color[n] = VmbUchar_t(h1 * dx + h2 * (1 - dx) + 0.5);
//	}
//	return value;
//}
//
//
//
//
//

//
//template <typename T>
//inline T abc(T a) { return a > 0 ? a : -a; }
//
//// creates a cost volume
//void calculateCostVolume(VmbUchar_t * const buffer1, VmbUchar_t * const buffer2, glm::mat4 C2C1inv, const int width, const int height, const int layers, const float minZ, const float maxZ, float * const Cost)
//{
//	const long HW = width * height;
//	std::unique_ptr<float[]> zlayers = std::unique_ptr<float[]>(new float[layers]);
//
//	for (int l = 0; l < layers; l++)
//	{
//		const float z = 1.f / ((float(l) / layers)*(1.f / minZ - 1.f / maxZ) + 1.f / maxZ);
//		zlayers[l] = z;
//	}
//
//	for (int v = 0; v < height; v++)
//	{
//		for (int u = 0; u < width; u++)
//		{
//			color<3> c1 = getInterpolatedColor<3>(buffer1, float(u), float(v), width, height);
//
//			for (int l = 0; l < layers; l++)
//			{
//				const float z = zlayers[l];
//				glm::vec4 uvz1 = glm::vec4(u, v, z, 1.0);
//				glm::vec4 uvz2 = C2C1inv * uvz1;
//				const float u2 = uvz2.x / uvz2.z;
//				const float v2 = uvz2.y / uvz2.z;
//				color<3> c2 = getInterpolatedColor<3>(buffer2, float(u2), float(v2), width, height);
//				float cost = abs<float>(float(c1.color[0]) - c2.color[0]) + abs<float>(float(c1.color[1]) - c2.color[1]) + abs<float>(float(c1.color[2]) - c2.color[2]);
//
//				Cost[layers * HW + u*width + v] = cost;
//			}			
//		}
//	}
//	
//}

