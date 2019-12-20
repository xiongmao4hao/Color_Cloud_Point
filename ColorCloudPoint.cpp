//#include "vtkAutoInit.h" 
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//#include <k4a/k4a.hpp>
//#include <iostream>
//#include <vector>
//#include <array>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/common/common_headers.h>
//#include <pcl/console/parse.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//#include "Pixel.h"
//#include "DepthPixelColorizer.h"
//#include "StaticImageProperties.h"
//#include "k4a_grabber.h"
//
//using namespace std;
//using namespace cv;
//using namespace boost;
//using namespace pcl;
//
//pcl::ModelCoefficients sphere_coeff;
//
//typedef pcl::PointXYZRGBA PointType;
//
//#define VERIFY(result, error)                                                                            \
//    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
//    {                                                                                                    \
//        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
//        exit(1);                                                                                         \
//    }  
//
//int main(int argc, char **argv)
//{
//	const uint32_t deviceCount = k4a::device::get_installed_count();
//	if (deviceCount == 0)
//	{
//		cout << "No Azure Kinect DK devices detected!" << endl;
//	}
//
//	// PCL Visualizer
//	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
//	//	new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
//	pcl::visualization::PCLVisualizer viewer("cloud viewer");
//	viewer.setBackgroundColor(0, 0, 0);
//	viewer.addCoordinateSystem(200.0);
//	viewer.setCameraPosition(0.0, 0.0, -2500.0, 1.0, -1.0, 1.0);
//
//	// Point Cloud
//	pcl::PointCloud<PointType>::ConstPtr cloud;
//
//	pcl::PointXYZRGBA point1;
//	point1.x = 20.0;
//	point1.y = 1000.0;
//	point1.z = 1000.0;
//
//	point1.b = 255;
//	point1.g = 0;
//	point1.r = 255;
//	point1.a = 255;
//
//	// Retrieved Point Cloud Callback Function
//	boost::mutex mutex;
//	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function = 
//		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr)
//	{
//		boost::mutex::scoped_lock lock(mutex);
//		
//		// Point Cloud Processing
//		cloud = ptr->makeShared();
//
//	};
//	
//	// KinectAzureDKGrabber
//	boost::shared_ptr<pcl::Grabber> grabber =
//		boost::make_shared<pcl::KinectAzureDKGrabber>(0, K4A_DEPTH_MODE_NFOV_UNBINNED,
//			K4A_IMAGE_FORMAT_COLOR_BGRA32, K4A_COLOR_RESOLUTION_720P);
//
//	// Register Callback Function
//	boost::signals2::connection connection = grabber->registerCallback(function);
//
//	sphere_coeff.values.resize(4);
//
//	// Start Grabber
//	grabber->start();
//
//	sphere_coeff.values[3] = 20.0;
//	//sphere_coeff.values[0] = 0.0;
//	//sphere_coeff.values[1] = 0.0;
//	//sphere_coeff.values[2] = 0.0;
//
//	while (!viewer.wasStopped())
//	{
//		// Update Viewer
//		viewer.spinOnce();
//		boost::mutex::scoped_try_lock lock(mutex);
//		if (lock.owns_lock() && cloud)
//		{
//			// Update Point Cloud
//			if (!viewer.updatePointCloud<pcl::PointXYZRGBA>(cloud, "cloud"))
//			{
//				viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, "cloud");
//				//viewer.updateSphere(point1, 20.0, 255, 255, 255, "sphere");
//				//viewer.addSphere(sphere_coeff, "sphere");
//				viewer.addSphere(point1, 20.0, 255, 0, 0, "sphere");
//				//viewer.addCube(0, 5, 0, 5, 0, 5, 1, 0, 0, "cube");
//			}
//		}
//	}
//
//	// Stop Grabber
//	grabber->stop();
//
//	// Disconnect Callback Function
//	if (connection.connected())
//	{
//		connection.disconnect();
//	}
//	return 0;
//}


//#include <iostream>
//#include <string>
//#include <vector>
//#include <array>
//#include <fstream>
//#include <algorithm>
//#include <sstream>
//#include <thread>
//#include <mutex>
//
//#include <windows.h>
//#include <GL/glew.h>
//#include <GLFW/glfw3.h>
//
//#include <Eigen/Dense>
//
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//
//#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp>
//#include <glm/gtc/type_ptr.hpp>
//
//#include <k4a/k4a.hpp>
//
//#include <aruco/src/cameraparameters.h>
//
//#include "Pixel.h"
//#include "Util.h"
//#include "DepthPixelColorizer.h"
//#include "StaticImageProperties.h"
//
//#include "Shader.h"
//#include "Texture.h"
//#include "PointCloudRenderer.h"
//#include "VideoRenderer.h"
//
//using namespace std;
//using namespace cv;
//using namespace glm;
//using namespace Eigen;
//
//GLFWwindow* window;
//
//int bufferIndex = 0;
//k4a::image initTexture;
//k4a::image depthImageBuffer;
//std::array<k4a::image, 30> depthBuffer;
//k4a::image outDepthFrame;
//
//k4a::image colorImageBuffer;
//std::array<k4a::image, 30> colorFrameBuffer;
//k4a::image outColorFrame;
//
//std::thread capturing_thread;
//std::mutex mtx;
//
//k4a::device device;
//k4a::capture capture;
//
//VideoRenderer videoRenderer;
//PointCloudRenderer cloudRenderer;
//
//k4a_device_configuration_t k4aSetup()
//{
//	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
//	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
//	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
//	config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
//	config.synchronized_images_only = true;
//	return config;
//}
//void frameRetriever()
//{
//	while (1)
//	{
//		if (device.get_capture(&capture, std::chrono::milliseconds(0)))
//		{
//			colorImageBuffer = capture.get_color_image();
//			depthImageBuffer = capture.get_depth_image();
//
//			if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))
//			{
//				break;
//			}
//
//			mtx.lock();
//			colorFrameBuffer[(bufferIndex) % 30] = colorImageBuffer;
//			depthBuffer[(bufferIndex) % 30] = depthImageBuffer;
//			bufferIndex++;
//			mtx.unlock();
//		}
//	}
//}
//void reshape(GLFWwindow *window, int w, int h)
//{
//	glViewport(0, 0, w, h);
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	glMatrixMode(GL_MODELVIEW);
//	glLoadIdentity();
//}
//void drawBackground()
//{
//	if (bufferIndex > 0)
//	{
//		mtx.lock();
//		outColorFrame = colorFrameBuffer[(bufferIndex - 1) % 30];
//		outDepthFrame = depthBuffer[(bufferIndex - 1) % 30];
//		mtx.unlock();
//
//		videoRenderer.render(outColorFrame);
//		cloudRenderer.render(window, outColorFrame, outDepthFrame);
//	}
//}
//void display()
//{
//	drawBackground();
//}
//
//int main(int argc, char **argv)
//{
//	const uint32_t deviceCount = k4a::device::get_installed_count();
//	if (deviceCount == 0)
//	{
//		cout << "no azure kinect devices detected!" << endl;
//	}
//
//	k4a_device_configuration_t config = k4aSetup();
//
//	cout << "Started opening K4A device..." << endl;
//	device = k4a::device::open(0);
//	device.start_cameras(&config);
//	cout << "Finished opening K4A device." << endl;
//
//	while (1)
//	{
//		if (device.get_capture(&capture, std::chrono::milliseconds(0)))
//		{
//			initTexture = capture.get_color_image();
//			break;
//		}
//	}
//
//	int texture_width = initTexture.get_width_pixels();
//	int texture_height = initTexture.get_height_pixels();
//
//	k4a::calibration calibration = device.get_calibration(config.depth_mode, config.color_resolution);
//	//***********************************************************************************************************
//
//	if (!glfwInit())
//	{
//		fprintf(stderr, "Failed to initialize GLFW\n");
//		return -1;
//	}
//
//	glfwWindowHint(GLFW_SAMPLES, 4);
//	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//
//	window = glfwCreateWindow(texture_width, texture_height, "VideoRenderer&PointCloudRenderer", NULL, NULL);
//	if (window == NULL)
//	{
//		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible.\n");
//		glfwTerminate();
//		return -1;
//	}
//	glfwMakeContextCurrent(window);
//
//	glewExperimental = true;
//	if (glewInit() != GLEW_OK)
//	{
//		fprintf(stderr, "Failed to initialize GLEW\n");
//		return -1;
//	}
//	glfwSetWindowPos(window, 1000, 500);
//	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
//	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
//	glfwSetFramebufferSizeCallback(window, reshape);
//
//	glfwPollEvents();
//	glfwSetCursorPos(window, texture_width / 2, texture_height / 2);
//	//***********************************************************************************************************
//
//	glViewport(0, 0, texture_width, texture_height);
//	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
//
//	GLuint VertexArrayID;
//	glGenVertexArrays(1, &VertexArrayID);
//	glBindVertexArray(VertexArrayID);
//
//	videoRenderer.setup("F:/科研项目/步态分析/代码/Color_Cloud_Point/TransformVertexShader.vertexshader", "F:/科研项目/步态分析/代码/TextureFragmentShader.fragmentshader");
//	videoRenderer.initTexture(initTexture);
//
//	cloudRenderer.setup("F:/科研项目/步态分析/代码/Color_Cloud_Point/PointCloud.vertexshader", "F:/科研项目/步态分析/代码/PointCloud.fragmentshader", texture_width, texture_height, calibration);
//
//	capturing_thread = std::thread(frameRetriever);
//
//	do
//	{
//		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//		display();
//
//		glfwSwapBuffers(window);
//		glfwPollEvents();
//
//	} while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && !glfwWindowShouldClose(window));
//
//	glDeleteVertexArrays(1, &VertexArrayID);
//
//	videoRenderer.deleteBuffer();
//	cloudRenderer.deleteBuffer();
//
//	glfwTerminate();
//
//	capturing_thread.join();
//	device.close();
//
//	return 0;
//}