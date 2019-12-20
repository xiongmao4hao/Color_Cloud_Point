//#include <k4a/k4a.hpp>
//#include <k4abt.hpp>
//#include <thread>
//#include <iostream>
//#include <windows.h>
//#include <k4abt.h>
//#include <vector>
//#include <array>
//#include <stdlib.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/imgproc/imgproc.hpp"
//#include "Pixel.h"
//#include "DepthPixelColorizer.h"
//#include "StaticImageProperties.h"
//
//using namespace cv;
//using namespace std;
//
//#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8000) ? 1:0)
//#define VERIFY(result, error)                                                                            \
//    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
//    {                                                                                                    \
//        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
//        exit(1);                                                                                         \
//    }  
//
//void cam0()  //普通的函数，用来执行线程
//{
//	int flag = -1;
//	int frame_count = 0;
//	//k4a::image depthImage;
//	k4a::image colorImage;
//	//k4a::image irImage;
//
//	//cv::Mat depthFrame;
//	cv::Mat colorFrame;
//	//cv::Mat irFrame;
//
//	//std::vector<Pixel> depthTextureBuffer;
//	//std::vector<Pixel> irTextureBuffer;
//	uint8_t* colorTextureBuffer;
//
//	k4a_float3_t tf_source_depth_3d;
//	k4a_float3_t tf_target_color_3d;
//	k4a_float2_t tf_target_color_2d;
//
//	Point joint_point[26];
//
//	const uint32_t deviceCount = k4a::device::get_installed_count();
//	if (deviceCount == 0)
//	{
//		cout << "No Azure Kinect DK devices detected!" << endl;
//	}
//
//	k4a::device dev = k4a::device::open(0);
//
//	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
//	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
//	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
//	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
//	config.synchronized_images_only = true;
//		
//	if (dev.is_sync_in_connected())
//	{
//		cout << "Subordinate device detected!\n" << endl;
//		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
//	}
//	else if (dev.is_sync_out_connected())
//	{
//		cout << "Master device detected!\n" << endl;
//		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
//	}
//	else
//	{
//		cout << "Standalone device detected!\n" << endl;
//		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
//	}
//
//	cout << "Started opening K4A device..." << endl;
//	dev.start_cameras(&config);
//	cout << "Finished opening K4A device!" << endl;
//
//	k4a::calibration sensor_calibration = dev.get_calibration(config.depth_mode, config.color_resolution);
//
//	k4abt::tracker tracker = k4abt::tracker::create(sensor_calibration);
//
//	k4a::capture capture;
//
//	while (1)
//	{
//		if (dev.get_capture(&capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
//		{
//			//depthImage = capture.get_depth_image();
//			colorImage = capture.get_color_image();
//			//irImage = capture.get_ir_image();
//
//			//ColorizeDepthImage(depthImage, DepthPixelColorizer::ColorizeBlueToRed, GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
//			//ColorizeDepthImage(irImage, DepthPixelColorizer::ColorizeGreyscale, GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);
//
//			colorTextureBuffer = colorImage.get_buffer();
//
//			//depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
//			colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
//			//irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_8UC4, irTextureBuffer.data());
//
//			if (tracker.enqueue_capture(capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
//			{
//				k4abt::frame body_frame = tracker.pop_result();
//				if (body_frame != nullptr)
//				{
//					size_t num_bodies = body_frame.get_num_bodies();
//                    //cout << num_bodies << " bodies are detected!" << endl;
//					for (size_t i = 0; i < num_bodies; i++)
//					{
//						k4abt_skeleton_t skeleton = body_frame.get_body_skeleton(i);
//						for (int j = 0; j < 26; j++)
//						{
//							tf_source_depth_3d.xyz.x = skeleton.joints[j].position.xyz.x;
//							tf_source_depth_3d.xyz.y = skeleton.joints[j].position.xyz.y;
//							tf_source_depth_3d.xyz.z = skeleton.joints[j].position.xyz.z;
//							tf_target_color_3d = sensor_calibration.convert_3d_to_3d(tf_source_depth_3d, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR);
//							sensor_calibration.convert_3d_to_2d(tf_target_color_3d, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &tf_target_color_2d);
//							//cout << "x: " << tf_target_color_2d.xy.x << " y: " << tf_target_color_2d.xy.y << endl;
//							
//							joint_point[j].x = tf_target_color_2d.xy.x;
//							joint_point[j].y = tf_target_color_2d.xy.y;
//							circle(colorFrame, joint_point[j], 10, Scalar(0, 255, 0), -1);
//						}
//						line(colorFrame, joint_point[20], joint_point[3], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[3], joint_point[2], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[2], joint_point[8], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[8], joint_point[9], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[9], joint_point[10], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[10], joint_point[11], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[2], joint_point[4], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[4], joint_point[5], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[5], joint_point[6], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[6], joint_point[7], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[2], joint_point[1], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[1], joint_point[0], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[0], joint_point[12], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[12], joint_point[13], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[13], joint_point[14], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[14], joint_point[15], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[0], joint_point[16], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[16], joint_point[17], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[17], joint_point[18], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[18], joint_point[19], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[20], joint_point[21], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[21], joint_point[22], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[22], joint_point[23], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[21], joint_point[24], Scalar(0, 255, 0), 2);
//						line(colorFrame, joint_point[24], joint_point[25], Scalar(0, 255, 0), 2);
//					}
//				}
//			}
//
//			//imshow("Kinect depth map", depthFrame);
//			
//			imshow("Kinect color frame", colorFrame);
//			//imshow("Kinect ir frame", irFrame);
//
//			if (KEY_DOWN('S') && flag == -1)
//			{
//				frame_count++;
//				imwrite(".\\a\\alpha" + std::to_string(frame_count) + ".png", colorFrame);
//				printf("You can take another picture!\n");
//				flag = 0;
//			}
//			if (!KEY_DOWN('S'))
//			{
//				flag = -1;
//			}
//		}
//		if (waitKey(30) == 27 || waitKey(30) == 'q')
//		{
//			dev.close();
//			break;
//		}
//	}
//	
//}
//
//void cam1()
//{
//	int flag = -1;
//	int frame_count = 0;
//	//k4a::image depthImage;
//	k4a::image colorImage;
//	//k4a::image irImage;
//
//	//cv::Mat depthFrame;
//	cv::Mat colorFrame;
//	//cv::Mat irFrame;
//
//	//std::vector<Pixel> depthTextureBuffer;
//	//std::vector<Pixel> irTextureBuffer;
//	uint8_t* colorTextureBuffer;
//
//	k4a_float3_t tf_source_depth_3d;
//	k4a_float3_t tf_target_color_3d;
//	k4a_float2_t tf_target_color_2d;
//
//	Point joint_point[26];
//
//	const uint32_t deviceCount = k4a::device::get_installed_count();
//	if (deviceCount == 0)
//	{
//		cout << "No Azure Kinect DK devices detected!" << endl;
//	}
//
//	k4a::device dev = k4a::device::open(1);
//
//	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
//	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
//	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
//	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
//	config.synchronized_images_only = true;
//
//	if (dev.is_sync_in_connected())
//	{
//		cout << "Subordinate device detected!\n" << endl;
//		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
//	}
//	else if (dev.is_sync_out_connected())
//	{
//		cout << "Master device detected!\n" << endl;
//		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
//	}
//	else
//	{
//		cout << "Standalone device detected!\n" << endl;
//		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
//	}
//
//	cout << "Started opening K4A device..." << endl;
//	dev.start_cameras(&config);
//	cout << "Finished opening K4A device!" << endl;
//
//	k4a::calibration sensor_calibration = dev.get_calibration(config.depth_mode, config.color_resolution);
//
//	k4abt::tracker tracker = k4abt::tracker::create(sensor_calibration);
//
//	k4a::capture capture;
//
//	while (1)
//	{
//		if (dev.get_capture(&capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
//		{
//			//depthImage = capture.get_depth_image();
//			colorImage = capture.get_color_image();
//			//irImage = capture.get_ir_image();
//
//			//ColorizeDepthImage(depthImage, DepthPixelColorizer::ColorizeBlueToRed, GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
//			//ColorizeDepthImage(irImage, DepthPixelColorizer::ColorizeGreyscale, GetIrLevels(K4A_DEPTH_MODE_PASSIVE_IR), &irTextureBuffer);
//
//			colorTextureBuffer = colorImage.get_buffer();
//
//			//depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
//			colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
//			//irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_8UC4, irTextureBuffer.data());
//
//			if (tracker.enqueue_capture(capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
//			{
//				k4abt::frame body_frame = tracker.pop_result();
//				if (body_frame != nullptr)
//				{
//					size_t num_bodies = body_frame.get_num_bodies();
//					//cout << num_bodies << " bodies are detected!" << endl;
//					for (size_t i = 0; i < num_bodies; i++)
//					{
//						k4abt_skeleton_t skeleton = body_frame.get_body_skeleton(i);
//						for (int j = 0; j < 26; j++)
//						{
//							tf_source_depth_3d.xyz.x = skeleton.joints[j].position.xyz.x;
//							tf_source_depth_3d.xyz.y = skeleton.joints[j].position.xyz.y;
//							tf_source_depth_3d.xyz.z = skeleton.joints[j].position.xyz.z;
//							tf_target_color_3d = sensor_calibration.convert_3d_to_3d(tf_source_depth_3d, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR);
//							sensor_calibration.convert_3d_to_2d(tf_target_color_3d, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &tf_target_color_2d);
//							//cout << "x: " << tf_target_color_2d.xy.x << " y: " << tf_target_color_2d.xy.y << endl;
//
//							joint_point[j].x = tf_target_color_2d.xy.x;
//							joint_point[j].y = tf_target_color_2d.xy.y;
//							circle(colorFrame, joint_point[j], 10, Scalar(0, 255, 0), -1);
//						}
//					}
//				}
//			}
//
//			//imshow("Kinect depth map", depthFrame);
//
//			imshow("Kinect color frame", colorFrame);
//			//imshow("Kinect ir frame", irFrame);
//
//			if (KEY_DOWN('S') && flag == -1)
//			{
//				frame_count++;
//				imwrite(".\\a\\alpha" + std::to_string(frame_count) + ".png", colorFrame);
//				printf("You can take another picture!\n");
//				flag = 0;
//			}
//			if (!KEY_DOWN('S'))
//			{
//				flag = -1;
//			}
//		}
//		if (waitKey(30) == 27 || waitKey(30) == 'q')
//		{
//			dev.close();
//			break;
//		}
//	}
//	
//}
//
//
//int main()
//{
//	thread th1(cam0);  //实例化一个线程对象th1，使用函数t1构造，然后该线程就开始执行了（t1()）
//	//Sleep(1000);
//	//thread th2(cam1);
//
//	th1.join(); // 必须将线程join或者detach 等待子线程结束主进程才可以退出
//	//th2.join();
//
//	//or use detach
//	//th1.detach();
//	//th2.detach();
//
//	return 0;
//}
//
