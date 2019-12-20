#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <pthread.h>
#include <iostream>
#include <windows.h>
#include <k4abt.h>
#include <stdlib.h>
#include <OpenNI.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <GLFW/glfw3.h>
#include <GL/freeglut.h>
#include <gl/gl.h>
#include <gl/GLU.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"
#include "myjoints.h"

using namespace cv;
using namespace std;

int16_t point_cloud_data[576][640][3];

#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8000) ? 1:0)
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }  

MyJoints myjoints;

void* cam0(void *p)
{
	int frame_count = 0;
	//k4a::image colorImage;
	
	cv::Mat depthFrame;
	//cv::Mat colorFrame;
	
	int16_t *xyzData;

	std::vector<Pixel> depthTextureBuffer;
	//uint8_t* colorTextureBuffer;

	k4a_float3_t tf_target_color_3d;
	k4a_float2_t tf_target_color_2d;

	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "No Azure Kinect DK devices detected!" << endl;
	}

	k4a::device dev = k4a::device::open(0);

	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;

	if (dev.is_sync_in_connected())
	{
		cout << "Subordinate device detected!\n" << endl;
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
	}
	else if (dev.is_sync_out_connected())
	{
		cout << "Master device detected!\n" << endl;
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
	}
	else
	{
		cout << "Standalone device detected!\n" << endl;
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
	}

	cout << "Started opening K4A device..." << endl;
	dev.start_cameras(&config);
	cout << "Finished opening K4A device!" << endl;

	k4a::calibration sensor_calibration = dev.get_calibration(config.depth_mode, config.color_resolution);
	k4a::transformation transformation = k4a::transformation(sensor_calibration);
	k4abt::tracker tracker = k4abt::tracker::create(sensor_calibration);
	k4a::capture capture;

	while (1)
	{
		if (dev.get_capture(&capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
		{
			k4a::image depthImage = capture.get_depth_image();
			//colorImage = capture.get_color_image();
			k4a::image xyzImage = NULL;
			xyzImage = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
										  depthImage.get_width_pixels(),
										  depthImage.get_height_pixels(),
										  depthImage.get_width_pixels() * 3 * (int)sizeof(int16_t));
			ColorizeDepthImage(depthImage, DepthPixelColorizer::ColorizeBlueToRed, GetDepthModeRange(config.depth_mode), &depthTextureBuffer);
			transformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_DEPTH, &xyzImage);

			xyzData = (int16_t *)(void *)xyzImage.get_buffer();
			const int width = xyzImage.get_width_pixels();
			const int height = xyzImage.get_height_pixels();
			for (int i = 0; i < width * height; ++i)
			{
				point_cloud_data[(i % width) - 1][int(i/width)][0] = xyzData[3 * i + 0];
				point_cloud_data[(i % width)][int(i / width)][1] = xyzData[3 * i + 1];
				point_cloud_data[(i % width) + 1][int(i / width)][2] = xyzData[3 * i + 2];
			}
			//colorTextureBuffer = colorImage.get_buffer();
			//height = 576, width = 640
			//depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4, depthTextureBuffer.data());
			//colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
			//cout << "Height:" << depthImage.get_height_pixels() << " Width:" << depthImage.get_width_pixels() << endl;
			
			if (tracker.enqueue_capture(capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
			{
				myjoints.setFlag();
				k4abt::frame body_frame = tracker.pop_result();
				if (body_frame != nullptr)
				{
					size_t num_bodies = body_frame.get_num_bodies();
					//cout << num_bodies << " bodies are detected!" << endl;
					for (size_t i = 0; i < num_bodies; i++)
					{
						k4abt_skeleton_t skeleton = body_frame.get_body_skeleton(i);
						myjoints.getData(skeleton);
					}
				}
			}
			else {
				myjoints.resetFlag();
			}
			//imshow("Kinect depth map", depthFrame);
			//imshow("Kinect color frame", colorFrame);		
		}
		if (waitKey(30) == 27 || waitKey(30) == 'q')
		{
			transformation.destroy();
			dev.close();
			break;
		}
	}
	return NULL;
}

const GLfloat PI = 3.14;


void init() 
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1);
	glShadeModel(GL_SMOOTH);
	GLfloat _ambient[] = { 1.0,1.0,1.0,1.0 };
	GLfloat _diffuse[] = { 1.0,1.0,0.0,1.0 };
	GLfloat _specular[] = { 1.0,1.0,1.0,1.0 };
	GLfloat _position[] = { 200,200,200,0 };
	glLightfv(GL_LIGHT0, GL_AMBIENT, _ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, _diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, _specular);
	glLightfv(GL_LIGHT0, GL_POSITION, _position);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	
	//材质反光性设置
	//GLfloat mat_specular[] = { 1.0, 0.0, 0.0, 1.0 };  //镜面反射参数
	//GLfloat mat_shininess[] = { 50.0 };               //高光指数
	//GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	//GLfloat white_light[] = { 1.0, 1.0, 1.0, 1.0 };   //灯位置(1,1,1), 最后1-开关
	//GLfloat Light_Model_Ambient[] = { 0.2, 0.2, 0.2, 1.0 }; //环境光参数

	////glClearColor(0.0, 0.0, 0.0, 0.0);  //背景色
	////glShadeModel(GL_SMOOTH);           //多变性填充模式

	////材质属性
	//glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	//glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

	////灯光设置
	//glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);   //散射光属性
	//glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);  //镜面反射光
	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, Light_Model_Ambient);  //环境光参数

	//glEnable(GL_LIGHTING);   //开关:使用光
	//glEnable(GL_LIGHT0);     //打开0#灯
	//glEnable(GL_DEPTH_TEST); //打开深度测试

}

void display(void)
{

	glMatrixMode(GL_MODELVIEW);
	// clear screen and depth buffer  
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	//glEnable(GL_BLEND); // 打开混合
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	// Reset the coordinate system before modifying   
	glLoadIdentity();

	myjoints.joint_lookat();

	glPushMatrix();
	glRotatef(180.0, 0.0, 0.0, 1.0);
	glTranslatef(-350.0, -300.0, 5000.0);
	myjoints.joint_rotate();
	glTranslatef(350.0, 300.0, -5000.0);

	//float x, y, z;
	// 绘制图像点云
	glBegin(GL_POINTS);
	for (int i = 0; i < 576; i++) {
		for (int j = 0; j < 640; j++) {
	//		// color interpolation
	//		glColor3f(1.0, 1.0, 1.0);
	//		x = point_cloud_data[i][j][0];
	//		y = point_cloud_data[i][j][1];
	//		z = point_cloud_data[i][j][2];
	//		glVertex3f(x, y, z);
		}
	}
	glEnd();
	glPopMatrix();

	myjoints.drawskeleton();

	glutSwapBuffers();
}

void mouse(int button, int state, int x, int y)
{
	myjoints.joint_mouse(button, state, x, y);
}

void motion(int x, int y)
{
	myjoints.joint_motion(x, y);
}

void keyboard(unsigned char c, int x, int y)
{
	myjoints.joint_key(c, x, y);
}

void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)w / (GLfloat)h, 1.0, 10000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	myjoints.joint_lookat();
}

void idle()
{
	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_DEPTH);
	glutInitWindowSize(640, 576);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("3D_Cloud_Points");
	init();
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutReshapeFunc(reshape);

	pthread_t pid;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_create(&pid, &attr, cam0, NULL);
	//cam0(NULL);
	//std::thread record = std::thread(cam0, NULL);

	glutMainLoop();
	return 0;
}

