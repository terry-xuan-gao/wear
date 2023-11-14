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
    QLabel* statusLabel = nullptr;
    
    QPushButton* startButton = nullptr;
    QPushButton* ansysButton = nullptr;

    CameraController* cameraController = nullptr;

    void initStatusLabel();
    void initButtons();
    
    void captureTask();



private slots:
    void startButtonClicked();
    

};

