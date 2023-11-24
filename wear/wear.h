#pragma once

#include <QtWidgets/QWidget>
#include <QPushButton>
#include <QObject>
#include <QDebug>
#include <QVBoxLayout>
#include <iostream>
#include <QMessageBox>
#include <QHBoxLayout>  
#include "ui_wear.h"
#include "Capture.h"
#include "Ansys.h"

class wear : public QWidget
{
    Q_OBJECT

public:
    wear(QWidget *parent = nullptr);
    ~wear();

    void run();

private:
    Ui::wearClass ui;
    
    QPushButton* captureButton = nullptr;
    Capture* captureWindow = new Capture();

    QPushButton* ansysButton = nullptr;
    Ansys* ansysWindow = new Ansys();

    void initButtons();

private slots:
    void captureButtonClicked();
    void ansysButtonClicked();
};
