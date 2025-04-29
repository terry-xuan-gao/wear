#pragma once
#include <qwidget.h>
#include <QtWidgets/QWidget>
#include <QPushButton>
#include <QObject>
#include <QDebug>
#include <QVBoxLayout>
#include <iostream>
#include <QMessageBox>
#include <QHBoxLayout>  
#include <QLabel>
#include <QThread>
#include <QString>

#include <stdio.h>
#include <Windows.h>
#include <process.h>
#include <conio.h>
#include <chrono>
#include <iostream>
#include <string>
#include "MvCameraControl.h"

#include "CameraController.h"
#include "DataManager.h"
#include "MyThread.h"

using namespace std;
using namespace cv;


class Capture :
    public QWidget
{
    Q_OBJECT

public:
    Capture(QWidget* parent = nullptr);
    ~Capture();


private:
    QVBoxLayout* layout = nullptr;
    QLabel* statusLabel = nullptr;
    
    QPushButton* enumButton = nullptr;
    QPushButton* openButton = nullptr;
    QPushButton* closeButton = nullptr;
    QPushButton* continueModeSetButton = nullptr;
    QPushButton* triggerModeSetButton = nullptr;
    QPushButton* startGrabbingButton = nullptr;
    QPushButton* stopGrabbingButton = nullptr;
    
    QPushButton* saveButton = nullptr;  
    QPushButton* scanButton = nullptr;
    QPushButton* testButton = nullptr;

    QLabel* imageDisplayLabel = nullptr;


private slots:
    void enumButtonClicked();
    void openButtonClicked();
    void closeButtonClicked();

    void continueModeButtonClicked();

    void startGrabbingButtonClicked();
    void stopGrabbingButtonClicked();

    void saveButtonClicked();
    void scanButtonClicked();

    void testButtonClicked();

public:
    //CameraController* cameraController = nullptr;
    CameraController* m_pcMyCamera[2];
    // 设备信息列表结构体变量，用来存储设备列表
    MV_CC_DEVICE_INFO_LIST* m_stDevList = new MV_CC_DEVICE_INFO_LIST(); 
    
    int devices_num = 0;
    int m_nTriggerMode = 0;
    int m_bContinueStarted = 0;

    cv::Mat* myImage = new cv::Mat(); //用于保存相机图像的图像指针对象
    MyThread* myThread = NULL;        //相机线程对象
    
    DataManager* dataManager = new DataManager();

private:
    void enumCamera();
    void openCamera();
    void closeCamera();
    void saveImage();
    void scanToolPin(string taskName);
    void scanToolPin(string taskName, const int images_num);
    void displayImage(QString displayPath);

    void testExposureTime();
    void testAcquisitionFrameRate();

    void logCameraError(int nRet);

    void initStatusLabel();
    void initButtons();
    void initDisplayLabel();
};

