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
#include <omp.h>	
#include "../common/shaders.h"
#include "../common/config_iter.h"
#include "../common/camera_loader.h"
#include "../common/vimba_helper.h"
#include "../common/frame_observer.h"

#include "../GoICP/jly_goicp.h"

using namespace AVT::VmbAPI;
using namespace std::chrono_literals;
using namespace glm;

void checkGLError(const char * a);


glm::vec3 findMajourPlane(const POINT3D* const Points1, const int length, const float z_threshold);
std::pair<float*, uint32_t> read_stl(const char * const filename);
std::pair<POINT3D*, uint32_t> read_points(const char * const filename);

GLuint loadProgramAndSetup(string vertex, string fragment, string geometry, const static config::config_iter config)
{
	const GLuint programID = config.geometry_shader.empty() ? LoadShaders(config.vertex_shader.c_str(), config.fragment_shader.c_str()) : Load3Shaders(config.vertex_shader.c_str(), config.geometry_shader.c_str(), config.fragment_shader.c_str());
	glUseProgram(programID);
	glUniform1f(glGetUniformLocation(programID, "minZ"), config.minZ);
	glUniform1f(glGetUniformLocation(programID, "maxZ"), config.maxZ);
	glUniform1ui(glGetUniformLocation(programID, "layers"), config.layers);
	glUniform1ui(glGetUniformLocation(programID, "width"), config.frame_width);
	glUniform1ui(glGetUniformLocation(programID, "height"), config.frame_height);
	return programID;
}

GLuint createTexture(GLenum target, GLint mag_filter, GLint min_filter)
{
	GLuint textureID;
	glGenTextures(1, &textureID);
	glBindTexture(target, textureID);
	glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(target, GL_TEXTURE_MAG_FILTER, mag_filter);
	glTexParameteri(target, GL_TEXTURE_MIN_FILTER, min_filter);
	return textureID;
}

int main(int argc, char* argv[])
{
	omp_set_num_threads(omp_get_max_threads());
	omp_set_dynamic(omp_get_max_threads()-1);	

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
		glm::mat4 C1inv = glm::inverse(C1);
		glm::mat4 C2C1inv = C2*glm::inverse(C1);
		glm::mat4 C1C2inv = C1*glm::inverse(C2);
		//C1[3][3] = 1;
		
		std::pair<float*, uint32_t> stl_pair = read_stl(config.stl_file.c_str());
		std::unique_ptr<float[]> model_vertices(stl_pair.first);
		uint32_t model_triangles = stl_pair.second;
		std::pair<POINT3D*, uint32_t> points_pair = read_points(config.points_file.c_str());
		std::unique_ptr<POINT3D[]> pModel(points_pair.first);
		std::unique_ptr<POINT3D[]> Points1 = std::unique_ptr<POINT3D[]>(new POINT3D[config.frame_width*config.frame_height+1]);

		GoICP goicp;
		goicp.MSEThresh = 5;
		goicp.trimFraction = 0.2;
		goicp.doTrim = true;
		goicp.pModel = pModel.get();
		goicp.Nm = points_pair.second;
		goicp.pData = Points1.get();
		goicp.Nd = 0;		
		goicp.dt.SIZE = 300; //config.getI("distTransSize");
		goicp.dt.expandFactor = 2.f;// config.getF("distTransExpandFactor");


		clock_t  clockBegin, clockEnd;
		cout << "Building Distance Transform..." << flush;
		clockBegin = clock();
		goicp.BuildDT();
		clockEnd = clock();
		cout << (double)(clockEnd - clockBegin) / CLOCKS_PER_SEC << "s (CPU)" << endl;


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

		//GLFWwindow* window = glfwCreateWindow(1024, 768, "ITER Stereo Pose Estimator", glfwGetPrimaryMonitor(), NULL);
		//GLFWwindow* window = glfwCreateWindow(config.screen_width, config.screen_height, "ITER Stereo Pose Estimator", NULL, NULL);
		GLFWwindow* window = glfwCreateWindow(config.frame_width, config.frame_height, "ITER Stereo Pose Estimator", NULL, NULL);

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
		

		checkGLError("GLEW Initializatiopn");		

		// calculation of both cost volumes in a GL_TEXTURE_2D_ARRAY
		const GLuint costvolProgramID = loadProgramAndSetup(config.vertex_shader, config.fragment_shader, config.geometry_shader, config);
		glUniformMatrix4fv(glGetUniformLocation(costvolProgramID, "C2C1inv"), 1, GL_FALSE, &C2C1inv[0][0]);

		// aggregation and best depth selection (Winner-Takes-All)
		const GLuint wtaProgramID = loadProgramAndSetup(config.main_vertex_shader, config.main_fragment_shader, "", config);

		// Left-to-Right correspondance check + some heuristic filters
		const GLuint l2rFilterProgramID = loadProgramAndSetup(config.depth_vertex_shader, config.depth_fragment_shader, "", config);
		glUniformMatrix4fv(glGetUniformLocation(l2rFilterProgramID, "C2C1inv"), 1, GL_FALSE, &C2C1inv[0][0]);
		glUniformMatrix4fv(glGetUniformLocation(l2rFilterProgramID, "C1C2inv"), 1, GL_FALSE, &C1C2inv[0][0]);

		const GLuint displayProgramID = loadProgramAndSetup(config.show_vertex_shader, config.show_fragment_shader, "", config);		
		glUniformMatrix4fv(glGetUniformLocation(l2rFilterProgramID, "C1"), 1, GL_FALSE, &C1[0][0]);
		checkGLError("GLSL Shader Programs loaded");


		//GLuint layerID = glGetUniformLocation(programID, "layer");
		//GLuint mainLayerID = glGetUniformLocation(mainProgramID, "layer");

		GLuint vao;
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);		


		///////////////////////////////////// FIRST FRAMEBUFFER ///////////////////////////////////////
		// The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
		GLuint costvolFramebufferID;
		glGenFramebuffers(1, &costvolFramebufferID);
		glBindFramebuffer(GL_FRAMEBUFFER, costvolFramebufferID);		
		GLuint costvolTextureID = createTexture(GL_TEXTURE_2D_ARRAY, GL_LINEAR, GL_LINEAR_MIPMAP_LINEAR);
		//// The texture we're going to render to
		//glGenTextures(1, &costvolTextureID);
		//// "Bind" the newly created texture : all future texture functions will modify this texture
		//glBindTexture(GL_TEXTURE_2D_ARRAY, costvolTextureID);		
		//// Texture filtering
		//glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		//glTexStorage3D(GL_TEXTURE_2D_ARRAY, 10, GL_RGB32F, config.frame_width, config.frame_height, config.layers);
		glTexStorage3D(GL_TEXTURE_2D_ARRAY, 5, GL_R16, config.frame_width, config.frame_height*2, config.layers);
		// Set "renderedTexture" as our colour attachement #0
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, costvolTextureID, 0);
		checkGLError("Render-to-Texture Framebuffer creation");
		//GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
		GLenum drawBuffer =  GL_COLOR_ATTACHMENT0 ;
		glDrawBuffers(1, &drawBuffer); // "1" is the size of DrawBuffers
		// check either our framebuffer is ok
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		{
			throw std::exception("Framebuffer #1 is not OK!");
		}		

		///////////////////////////////////// SECOND FRAMEBUFFER ///////////////////////////////////////
		GLuint depthFramebufferID;
		glGenFramebuffers(1, &depthFramebufferID);
		glBindFramebuffer(GL_FRAMEBUFFER, depthFramebufferID);
		GLuint depthTextureID = createTexture(GL_TEXTURE_2D, GL_LINEAR, GL_LINEAR);
		//glGenTextures(1, &depthTextureID);
		//// "Bind" the newly created texture : all future texture functions will modify this texture
		//glBindTexture(GL_TEXTURE_2D, depthTextureID);
		//// Texture filtering
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8, config.frame_width, config.frame_height * 2);
		// Set "renderedTexture" as our colour attachement #0
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, depthTextureID, 0);
		checkGLError("Render-to-Texture Framebuffer 2 creation");
		//GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
		drawBuffer = GL_COLOR_ATTACHMENT0;
		glDrawBuffers(1, &drawBuffer); // "1" is the size of DrawBuffers

									   // check either our framebuffer is ok
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		{
			throw std::exception("Framebuffer#2 is not OK!");
		}

		///////////////////////////////////// THIRD FRAMEBUFFER	//////////////////////////////////////////	
		GLuint pointsFramebufferID;
		glGenFramebuffers(1, &pointsFramebufferID);
		glBindFramebuffer(GL_FRAMEBUFFER, pointsFramebufferID);
		GLuint pointsTextureID = createTexture(GL_TEXTURE_2D, GL_LINEAR, GL_LINEAR);
		//glGenTextures(1, &pointsTextureID);
		//glBindTexture(GL_TEXTURE_2D, pointsTextureID);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8, config.frame_width, config.frame_height);
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, pointsTextureID, 0);
		checkGLError("Render-to-Texture Framebuffer 2 creation");
		//GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
		drawBuffer = GL_COLOR_ATTACHMENT0;
		glDrawBuffers(1, &drawBuffer); // "1" is the size of DrawBuffers
		// check either our framebuffer is ok
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		{
			throw std::exception("Framebuffer #3 is not OK!");
		}


		// Two triangles are always the same, so can be initialized just once in advance				
		GLuint elementbuffer, uvbuffer;
		GLuint index_buffer[] = {0, 1, 3, 1, 2, 3};
		GLint uv_buffer[] = {0, 0, config.frame_width, 0, config.frame_width, config.frame_height, 0, config.frame_height};
		glGenBuffers(1, &elementbuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(index_buffer), index_buffer, GL_STATIC_DRAW);

		glGenBuffers(1, &uvbuffer);
		glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(uv_buffer), uv_buffer, GL_STATIC_DRAW);
		
		// STL model is also static
		GLuint modelbuffer;
		glGenBuffers(1, &modelbuffer);
		glBindBuffer(GL_ARRAY_BUFFER, modelbuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*model_triangles*3, model_vertices.get(), GL_STATIC_DRAW);		
		
		GLuint texture1ID = createTexture(GL_TEXTURE_2D, GL_LINEAR, GL_LINEAR);
		GLuint texture2ID = createTexture(GL_TEXTURE_2D, GL_LINEAR, GL_LINEAR);

		//GLuint texture1ID, texture2ID;
		//glGenTextures(1, &texture1ID);
		//glBindTexture(GL_TEXTURE_2D, texture1ID);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		////glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

		//glGenTextures(1, &texture2ID);
		//glBindTexture(GL_TEXTURE_2D, texture2ID);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		////glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		checkGLError("Buffers and Textures init");
		
		CameraPtr camera1, camera2;		
		FeaturePtr feature1, feature2;

		checkStatus(system.OpenCameraByID(config.camera1_ip.c_str(), VmbAccessModeFull, camera1));
		checkStatus(system.OpenCameraByID(config.camera2_ip.c_str(), VmbAccessModeFull, camera2));		
		
		
		setFeature(camera1, feature1, "BinningHorizontal", config.camera_binning);
		setFeature(camera1, feature1, "BinningVertical", config.camera_binning);
		setFeature(camera1, feature1, "PixelFormat", "Mono8");
		setFeature(camera1, feature1, "Width", config.frame_width);
		setFeature(camera1, feature1, "Height", config.frame_height);
		setFeatureDouble(camera1, feature1, "ExposureTimeAbs", config.exposure_time);

		setFeature(camera2, feature2, "BinningHorizontal", config.camera_binning);
		setFeature(camera2, feature2, "BinningVertical", config.camera_binning);
		setFeature(camera2, feature2, "PixelFormat", "Mono8");
		setFeature(camera2, feature2, "Width", config.frame_width);
		setFeature(camera2, feature2, "Height", config.frame_height);
		setFeatureDouble(camera2, feature2, "ExposureTimeAbs", config.exposure_time);
		
		
		VmbInt64_t PayloadSize = readFeature(camera1, feature1, "PayloadSize");		
		PayloadSize = HW > PayloadSize ? HW : PayloadSize; 
		
		// these will be destroyed at the end of a app
		std::unique_ptr<float[]> lookup1 = config::prepare_lookup2(camera1_pair.first, cam1_k1, cam1_k2, config.frame_width, config.frame_height);
		std::unique_ptr<float[]> lookup2 = config::prepare_lookup2(camera2_pair.first, cam2_k1, cam2_k2, config.frame_width, config.frame_height);
		

		std::unique_ptr<VmbUchar_t[]> buffer1(new VmbUchar_t[PayloadSize*4]);
		std::unique_ptr<VmbUchar_t[]> buffer2(new VmbUchar_t[PayloadSize*4]);		
		IFrameObserverPtr pObserver1(new FrameObserver(camera1, buffer1.get(), lookup1.get(), config.frame_width, config.frame_height));
		IFrameObserverPtr pObserver2(new FrameObserver(camera2, buffer2.get(), lookup2.get(), config.frame_width, config.frame_height));

		std::unique_ptr<uint8_t[]> buffer3 = std::unique_ptr<uint8_t[]>(new uint8_t[PayloadSize*3+1]);
		
				
		setFeature(camera1, feature1, "AcquisitionMode", "Continuous");
		setFeature(camera1, feature1, "TriggerSource", "Software");
		setFeature(camera1, feature1, "SyncOutSource", "Imaging");
						
				
		setFeature(camera2, feature2, "AcquisitionMode", "Continuous");
		setFeature(camera2, feature2, "TriggerSource", "Line1");		
		

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
		while (!glfwWindowShouldClose(window) && glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS)
		{				
			checkGLError("Iteration Frame 1");
			std::cout << "GL Frame " << frameNum ++  << std::endl;

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
					FramePtr frame1 = ((FrameObserver*)pObserver1.get())->LastFrame;
					FramePtr frame2 = ((FrameObserver*)pObserver2.get())->LastFrame;
					camera1->QueueFrame(frame1);
					camera2->QueueFrame(frame2);
					((FrameObserver*)pObserver1.get())->frame_arrived = false;
					((FrameObserver*)pObserver2.get())->frame_arrived = false;
					runCommand(camera1, feature1, "TriggerSoftware");					
					triggered = time(NULL);
					std::cout << triggered << " Triggered Frames due to long delay" << std::endl;
				}
				std::this_thread::sleep_for(1ms);
			}
			
			FramePtr frame1 = ((FrameObserver*)pObserver1.get())->LastFrame;
			FramePtr frame2 = ((FrameObserver*)pObserver2.get())->LastFrame;
			

			VmbUchar_t * _buffer1;
			frame1->GetImage(_buffer1);

			VmbUchar_t * _buffer2;
			frame2->GetImage(_buffer2);

			//glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1ID);
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, 1920, 1080, 0, GL_RED, GL_UNSIGNED_BYTE, buffer1.get());
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, config.input_width, config.input_height, 0, GL_RED, GL_UNSIGNED_BYTE, buffer1.get());
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, config.input_width, config.input_height, 0, GL_RED, GL_UNSIGNED_BYTE, _buffer1);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, config.input_width, config.input_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer1.get());
			glGenerateMipmap(GL_TEXTURE_2D);
			

			//glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, texture2ID);
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, 1920, 1080, 0, GL_RED, GL_UNSIGNED_BYTE, buffer2.get());
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, config.input_width, config.input_height, 0, GL_RED, GL_UNSIGNED_BYTE, buffer2.get());
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, config.input_width, config.input_height, 0, GL_RED, GL_UNSIGNED_BYTE, _buffer2);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, config.input_width, config.input_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer2.get());
			glGenerateMipmap(GL_TEXTURE_2D);
			
			camera1->QueueFrame(frame1);
			camera2->QueueFrame(frame2);

			((FrameObserver*)pObserver1.get())->frame_arrived = false;
			((FrameObserver*)pObserver2.get())->frame_arrived = false;
						
			runCommand(camera1, feature1, "TriggerSoftware");
			triggered = time(NULL);
			std::cout << triggered << " Triggered Frames normally" << std::endl;			

			// turn on the indexing 
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
			/////////////////////////////////////// FIRST FRAMEBUFFER - COST VOLUME /////////////////////
			///// Camera 1 Cost Volume
			glBindFramebuffer(GL_FRAMEBUFFER, costvolFramebufferID);
			//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glUseProgram(costvolProgramID);
			glViewport(0, 0, config.frame_width, config.frame_height);

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1ID);

			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, texture2ID);

			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
			glVertexAttribIPointer(0, 2, GL_INT, 0, (void*)0);

			glUniform1i(glGetUniformLocation(costvolProgramID, "Texture1"), 0);
			glUniform1i(glGetUniformLocation(costvolProgramID, "Texture2"), 1);

			//glViewport(0, 0, config.frame_width, config.frame_height);
			glUniformMatrix4fv(glGetUniformLocation(costvolProgramID, "C2C1inv"), 1, GL_FALSE, &C2C1inv[0][0]);
			glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (void*)0, config.layers);

			////// Camera 2 Cost Volume
			glViewport(0, config.frame_height, config.frame_width, config.frame_height);

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture2ID);

			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, texture1ID);

			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
			glVertexAttribIPointer(0, 2, GL_INT, 0, (void*)0);

			glUniform1i(glGetUniformLocation(costvolProgramID, "Texture1"), 0);
			glUniform1i(glGetUniformLocation(costvolProgramID, "Texture2"), 1);

			//glViewport(0, 0, config.frame_width, config.frame_height);
			glUniformMatrix4fv(glGetUniformLocation(costvolProgramID, "C2C1inv"), 1, GL_FALSE, &C1C2inv[0][0]);
			glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (void*)0, config.layers);

			glDisableVertexAttribArray(0);

			///////////////////////////////  SECOND FRAMEBUFFER - WTA (DEPTH ESTIMATION) //////////////

			glBindFramebuffer(GL_FRAMEBUFFER, depthFramebufferID);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glUseProgram(wtaProgramID);			
			//glViewport(0, config.frame_height, config.frame_width, config.frame_height);
			// works for both cameras at once
			glViewport(0, 0, config.frame_width, config.frame_height*2);

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1ID);

			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, texture2ID);

			glActiveTexture(GL_TEXTURE2);
			glBindTexture(GL_TEXTURE_2D_ARRAY, costvolTextureID);
			glGenerateMipmap(GL_TEXTURE_2D_ARRAY);

			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
			glVertexAttribIPointer(0, 2, GL_INT, 0, (void*)0);

			glUniform1i(glGetUniformLocation(wtaProgramID, "Texture1"), 0);
			glUniform1i(glGetUniformLocation(wtaProgramID, "Texture2"), 1);
			glUniform1i(glGetUniformLocation(wtaProgramID, "TextureCost"), 2);

			
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (void*)0);
			glDisableVertexAttribArray(0);

			//////////////////////  THIRD FRAMEBUFFER = LEFT2RIGHT CHECK + REMOVING OUTLIERS (FILTERING) ////////////
			glBindFramebuffer(GL_FRAMEBUFFER, pointsFramebufferID);
			//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glUseProgram(l2rFilterProgramID);
			//glViewport(0, config.frame_height, config.frame_width, config.frame_height);
			glViewport(0, 0, config.screen_width, config.screen_height);

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1ID);

			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, texture2ID);

			glActiveTexture(GL_TEXTURE2);
			glBindTexture(GL_TEXTURE_2D, depthTextureID);
			//glGenerateMipmap(GL_TEXTURE);

			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
			glVertexAttribIPointer(0, 2, GL_INT, 0, (void*)0);

			glUniform1i(glGetUniformLocation(l2rFilterProgramID, "Texture1"), 0);
			glUniform1i(glGetUniformLocation(l2rFilterProgramID, "Texture2"), 1);
			glUniform1i(glGetUniformLocation(l2rFilterProgramID, "TextureDepth"), 2);
			
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (void*)0);
			glDisableVertexAttribArray(0);			
			checkGLError("Iteration Frame 3");			
			
			glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer3.get());
			checkGLError("glReadPixels Error");

			int length = 0;
			for (int i = 0; i < HW; i++)
			{				
				const float d = buffer3[i * 3];
				if (d > 0.f)
				{
					const float z = config.maxZ - (config.maxZ - config.minZ) * d / (config.layers - 1.f);
					const float u = float(i % width);
					const float v = float(i / width);
					glm::vec4 xyz  = C1inv * glm::vec4(u*z, v*z, z, 1.f);
					Points1[length].x = xyz.x;
					Points1[length].y = xyz.y;
					Points1[length].z = xyz.z;
					length++;
				}
			}

			std::cout << "major plane: (" << length << ") / ";
			vec3 abc = findMajourPlane(Points1.get(), length, config.plane_threshold);
			std::cout << " " << abc.x << " " << abc.y << " " << abc.z << " " << std::endl;

			int k = 0;
			for (int i = 0; i < length; i++)
			{
				const float error = abs(abc.x * Points1[i].x + abc.y * Points1[i].y + abc.z - Points1[i].z);
				if (error <= config.scene_threshold)
				{
					Points1[k].x = Points1[i].x;
					Points1[k].y = Points1[i].y;
					Points1[k].z = Points1[i].z;
					k++;
				}
			}
			goicp.Nd = k;			
			
			cout << "Registering..." << endl;
			clockBegin = clock();
			goicp.Register();
			clockEnd = clock();
			double time = (double)(clockEnd - clockBegin) / CLOCKS_PER_SEC;
			cout << "Optimal Rotation Matrix:" << endl;
			cout << goicp.optR << endl;
			cout << "Optimal Translation Vector:" << endl;
			cout << goicp.optT << endl;
			cout << "Finished in " << time << endl;

			//////////////////////  SCREEN FRAMEBUFFER DISPLAY ALIGNED MODEL
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			
			// Enable depth test		
			glEnable(GL_DEPTH_TEST);
			// Accept fragment if it closer to the camera than the former one
			glDepthFunc(GL_LESS);

			glUseProgram(displayProgramID);
			//glViewport(0, config.frame_height, config.frame_width, config.frame_height);
			glViewport(0, 0, config.screen_width, config.screen_height);

			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1ID);

			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, modelbuffer);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 0, (void*)0);
			glm::mat4 Transform(1,0,0,0,0,1,0,0,0,0,1,0,0,0,1000,1);
			glUniformMatrix4fv(glGetUniformLocation(l2rFilterProgramID, "Transform"), 1, GL_FALSE, &Transform[0][0]);
			glUniform1i(glGetUniformLocation(displayProgramID, "Texture1"), 0);
			// turn off the indexing  (it's easier for STL rendering)
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (void*)0);

			glDisableVertexAttribArray(0);

			
			glfwSwapBuffers(window);
			glfwPollEvents();			
		}

		glDisableVertexAttribArray(0);
		glDeleteBuffers(1, &uvbuffer);
		glDeleteBuffers(1, &elementbuffer);
		glDeleteVertexArrays(1, &vao);
		glDeleteTextures(1, &texture1ID);
		glDeleteTextures(1, &texture2ID);
		glDeleteTextures(1, &costvolTextureID);		
		glDeleteProgram(costvolProgramID);
		glDeleteProgram(wtaProgramID);
		glDeleteFramebuffers(1, &costvolFramebufferID);
		
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


//std::unique_ptr<float[]>  prepare_lookup2(const glm::mat3 K1, const float cam1_k1, const float cam1_k2, const int width, const int height);
//
//
//// frame observer that reacts on new frames
//class FrameObserver : public IFrameObserver
//{
//private:
//	VmbUchar_t * const buffer; // last frame buffer
//	const float * const lookup;		
//
//public:
//			
//	// last-frame buffer and mutex are transmitted here
//	FrameObserver(CameraPtr pCamera, VmbUchar_t * const _buffer, const float * const _lookup) : IFrameObserver(pCamera), buffer(_buffer), lookup(_lookup)
//	{		
//		
//	}
//
//	std::mutex buffer_mutex;
//	FramePtr LastFrame;
//	std::atomic<bool> frame_arrived;
//
//
//	void FrameReceived(const FramePtr pFrame)
//	{
//		std::string camid;
//		VmbUint64_t frameid;
//		m_pCamera->GetID(camid);
//		pFrame->GetFrameID(frameid);
//		std::cout << time(NULL) << " "<< camid << ": " << frameid << std::endl;
//		VmbUint32_t width, height;
//		VmbUchar_t * _buffer1;
//		pFrame->GetWidth(width);
//		pFrame->GetHeight(height);		
//		pFrame->GetImage(_buffer1);
//		#pragma omp parallel for
//		for (int y = 0; y < height; y++)
//		{
//			const long row = y * width * 3;
//			for (int x = 0; x < width; x++)
//			{
//				buffer[row + x * 3] = _buffer1[y * width + x];
//				const int xl = std::max(0, x - 1);
//				const int xr = std::min<int>(width-1, x + 1);
//				buffer[row + x * 3 + 1] = uint8_t(int(_buffer1[y * width + xr]) - int(_buffer1[y * width + xl]) + 127);
//				const int yu = std::max(0, y - 1);
//				const int yd = std::min<int>(height - 1, y + 1);
//				buffer[row + x * 3 + 2] = uint8_t(int(_buffer1[yd * width + x]) - int(_buffer1[yu * width + x]) + 127);
//			}
//		}
//		LastFrame = pFrame;
//		frame_arrived = true;
//		//VmbUchar_t * _buffer;
//		//pFrame->GetImage(_buffer); 
//		//VmbUint32_t imsize;
//		//pFrame->GetImageSize(imsize);
//		//buffer_mutex.lock();
//		//memcpy(buffer, _buffer, imsize);
//		//buffer_mutex.unlock();
//		//captured = true;
//		//VmbImage sourceImage;
//		//VmbImage destinationImage;
//		//VmbTransformInfo info;
//		//// set size member for verification inside API
//		//sourceImage.Size = sizeof(sourceImage);
//		//destinationImage.Size = sizeof(destinationImage);
//
//		//// attach the data buffers
//		//sourceImage.Data = _buffer;
//		//destinationImage.Data = buffer;
//
//		//VmbSetImageInfoFromPixelFormat(VmbPixelFormatBayerGR8, 1920, 1080, &sourceImage);
//		////VmbSetImageInfoFromPixelFormat(VmbPixelFormatBayerRG8, 1920, 1080, &sourceImage);
//		////VmbSetImageInfoFromPixelFormat(VmbPixelFormatBayerBG8, 1920, 1080, &sourceImage);		
//		//VmbSetImageInfoFromPixelFormat(VmbPixelFormatRgb8, 1920, 1080, &destinationImage);		
//		////VmbSetImageInfoFromInputImage(&sourceImage, VmbPixelLayoutRGB, 8, &destinationImage);
//
//		//// set the debayering algorithm to 3 by 3
//		////VmbSetDebayerMode(VmbDebayerMode3x3, &info);
//		//VmbSetDebayerMode(VmbDebayerMode2x2, &info);
//
//		//// perform the transformation
//		//VmbImageTransform(&sourceImage, &destinationImage, &info, 1);
//
//		// When you are finished copying the frame , re-queue it
//		//m_pCamera->QueueFrame(pFrame);
//	}
//};

//GLuint prepare_lookup2(const glm::mat3 K1, const float cam1_k1, const float cam1_k2, const int width, const int height);
//GLuint prepare_lookup_inverse(const glm::mat3 K1, const glm::mat3 K2, float cam1_k1, const float cam1_k2, const float cam2_k1, const float cam2_k2, const int width, const int height);



glm::vec3 findMajourPlane(const POINT3D* const Points1, const int length, const float z_threshold = 20.f)
{
	vec3 bestABC = vec3(0, 0, 0);
	int bestNumber = 0;

	for (int i = 0; i < 1000; i++)
	{
		
		int i1 = rand() % length;
		int i2 = i1;
		while (i2 == i1)
		{
			i2 = rand() % length;
		}
		int i3 = i1;
		while (i3 == i1 || i3 == i2)
		{
			i3 = rand() % length;
		}

		vec3 xy1a = vec3(Points1[i1].x, Points1[i1].y, 1);
		vec3 xy1b = vec3(Points1[i2].x, Points1[i2].y, 1);
		vec3 xy1c = vec3(Points1[i3].x, Points1[i3].y, 1);

		mat3 XY1 = glm::transpose(mat3(xy1a, xy1b, xy1c));
		vec3 abc = glm::inverse(XY1)*vec3(Points1[i1].z, Points1[i2].z, Points1[i3].z);
		
		int number = 0;
		#pragma omp parallel for reduction(+:number)
		for (int j = 0; j < length; j++)
		{
			float zj = glm::dot(abc, vec3(Points1[j].x, Points1[j].y, 1));
			float err = abs(Points1[j].z - zj);
			if (err < z_threshold)
			{
				number++;
			}
		}

		if (number > bestNumber)
		{
			bestNumber = number;
			bestABC = abc;
		}
	}
	cout << " (" << bestNumber << ") ";
	return bestABC;
}

float parse_float(std::ifstream& s) {
	char f_buf[sizeof(float)];
	s.read(f_buf, 4);
	float* fptr = (float*)f_buf;
	return *fptr;
}

std::pair<float*, uint32_t> read_stl(const char * const filename)
{
	ifstream ifs(filename, ios::in | ios::binary);
	char header_info[80];
	ifs.read(header_info, 80);
	uint32_t triangles;
	ifs.read(reinterpret_cast<char *>(&triangles), sizeof(uint32_t));
	float * points = new float[triangles * 3 * 3];
	for (uint32_t i = 0; i < triangles; i++)
	{
		float p[3];
		ifs.read(reinterpret_cast<char *>(&p), 3 * sizeof(float));

		ifs.read(reinterpret_cast<char *>(&points[i * 9]), 9 * sizeof(float));
	}
	return std::make_pair<float*, uint32_t>(std::move(points), std::move(triangles));
}

std::pair<POINT3D*, uint32_t> read_points(const char * const filename)
{
	ifstream ifs(filename, ios::in | ios::binary);
	uint32_t points;
	ifs.read(reinterpret_cast<char *>(&points), sizeof(uint32_t));
	POINT3D * pointXYZs = new POINT3D[points];
	for (uint32_t i = 0; i < points; i++)
	{
		//ifs.read(reinterpret_cast<char *>(&pointXYZs[i * 3]), 3 * sizeof(float));
		ifs.read(reinterpret_cast<char *>(&(pointXYZs[i].x)), sizeof(float));
		ifs.read(reinterpret_cast<char *>(&(pointXYZs[i].y)), sizeof(float));
		ifs.read(reinterpret_cast<char *>(&(pointXYZs[i].z)), sizeof(float));
	}
	return std::make_pair<POINT3D*, uint32_t>(std::move(pointXYZs), std::move(points));
}
