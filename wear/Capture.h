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

#include <stdio.h>
#include <Windows.h>
#include <process.h>
#include <conio.h>
#include <iostream>
#include "MvCameraControl.h"

#include "CameraController.h"
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
    
    QPushButton* startButton = nullptr;
    
    QPushButton* enumButton = nullptr;
    QPushButton* openButton = nullptr;
    QPushButton* closeButton = nullptr;
    QPushButton* continueModeSetButton = nullptr;
    QPushButton* triggerModeSetButton = nullptr;
    QPushButton* startGrabbingButton = nullptr;
    QPushButton* stopGrabbingButton = nullptr;
    QPushButton* saveButton = nullptr;

    QPushButton* ansysButton = nullptr;

private slots:
    void startButtonClicked();

    void enumButtonClicked();
    void openButtonClicked();
    void closeButtonClicked();

    void continueModeButtonClicked();

    void startGrabbingButtonClicked();
    void stopGrabbingButtonClicked();

    void saveButtonClicked();

public:
    //CameraController* cameraController = nullptr;
    CameraController* m_pcMyCamera[2];
    // �豸��Ϣ�б�ṹ������������洢�豸�б�
    MV_CC_DEVICE_INFO_LIST* m_stDevList = new MV_CC_DEVICE_INFO_LIST(); 
    
    int devices_num = 0;
    int m_nTriggerMode = 0;
    int m_bContinueStarted = 0;

    cv::Mat* myImage = new cv::Mat(); //���ڱ������ͼ���ͼ��ָ�����
    MyThread* myThread = NULL;        //����̶߳���
    
private:
    void enumCamera();
    int  openCamera();
    void closeCamera();
    void saveImage();

    void logCameraError(int nRet);

    void initStatusLabel();
    void initButtons();

};

