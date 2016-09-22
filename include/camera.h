#include "stdafx.h"
#include <iostream>
#include <sstream>
#include "FlyCapture2.h"

//Opencv
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace FlyCapture2;
using namespace std;

//函数原型
//显示相机信息
void PrintCameraInfo( CameraInfo* pCamInfo );
//显示图像格式信息
void PrintFormat7Capabilities( Format7Info fmt7Info );

//相机初始化
int CameraInitialize(Error &error, Camera &cam);
//获取图像
int QueryImages(Error & error, Camera & cam, Image & convertedImage);