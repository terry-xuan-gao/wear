#ifndef MYTHREAD_H
#define MYTHREAD_H
#include "QThread"
#include "CameraController.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>  
#include <string>  
#include <algorithm>  
#include <iostream>  
#include <iterator>  
#include <stdio.h>  
#include <stdlib.h>  
#include <ctype.h>  

using namespace std;
using namespace cv;

class MyThread :public QThread
{
	Q_OBJECT

public:
	MyThread();
	~MyThread();

	void run();
	void getCameraPtr(CameraController* camera);
	void getImagePtr(Mat* image);
	void getCameraIndex(int index);

signals:
	void mess();
	void Display(const Mat* image, int index);

private:
	CameraController* cameraPtr = NULL;
	cv::Mat* imagePtr = NULL;
	int cameraIndex = NULL;
	int TriggerMode;
};

#endif // MYTHREAD_H
#pragma once


