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
    QPushButton* startGrabbingButton = nullptr;
    QPushButton* stopGrabbingButton = nullptr;
    QPushButton* saveButton = nullptr;


    QPushButton* ansysButton = nullptr;

    CameraController* cameraController = nullptr;

    void initStatusLabel();
    void initButtons();
    
    void captureTask();

    
    CameraController* m_pcMyCamera[2];
    MV_CC_DEVICE_INFO_LIST* m_stDevList = new MV_CC_DEVICE_INFO_LIST(); // 设备信息列表结构体变量，用来存储设备列表
    int devices_num = 0;
    //MV_SAVE_IAMGE_TYPE m_nSaveImageType;

    void logCameraError(int nRet);


private slots:
    void startButtonClicked();
    
    void enumCamera();
    int openCamera();
    void closeCamera();
    void saveImage();
};

