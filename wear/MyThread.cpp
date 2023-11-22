#include "mythread.h"

MyThread::MyThread()
{
}

MyThread::~MyThread()
{
	terminate();
	if (cameraPtr != NULL)
	{
		delete cameraPtr;
	}
	if (imagePtr != NULL)
	{
		delete imagePtr;
	}
}

void MyThread::getCameraPtr(CameraController* camera)
{
	cameraPtr = camera;
}

void MyThread::getImagePtr(Mat* image)
{
	imagePtr = image;
}

void MyThread::getCameraIndex(int index)
{
	cameraIndex = index;
}

//void MyThread::get_TriggerMode(int m_nTriggerMode)
//{
//	TriggerMode = m_nTriggerMode;
//}

void MyThread::run()
{
	if (cameraPtr == NULL) {
		return;
	}

	if (imagePtr == NULL) {
		return;
	}

	while (!isInterruptionRequested())
	{
		std::cout << "Thread_Trigger:" << cameraPtr->softTrigger() << std::endl;
		std::cout << "Thread_Readbuffer:" << cameraPtr->ReadBuffer(*imagePtr) << std::endl;
		/*emit mess();*/
		emit Display(imagePtr, cameraIndex);//发送信号 img_display_label接收并显示
		msleep(30);
	}
}

