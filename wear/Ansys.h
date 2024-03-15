#pragma once
#include <QWidget>
#include <QVBoxLayout>
#include <QCoreApplication>
#include <QFile>
#include <QTextStream>
#include <QComboBox>
#include <QStringListModel>
#include <QLabel>
#include <QHBoxLayout>
#include <QPushButton>
#include <QColor>
#include <QLineEdit>

#include <vector>
#include <string>

#include "DataManager.h"
#include "PointCloudProducer.h"

using namespace std;

class Ansys :
    public QWidget
{
    Q_OBJECT

public:
    Ansys(QWidget* parent = nullptr);
    ~Ansys();

    void initTaskList();
    void initPushButtons();

    void refreshTaskList();

private slots:
    void generatePointCloudButtonClicked();
    void viewPointCloudButtonClicked();
    void savePointCloudButtonClicked();
    void tiltOptimize();
    void fitUpperEnvelopButtonClicked();
    void fitLowerEnvelopButtonClicked();

private:
    QVBoxLayout* layout = new QVBoxLayout();

    QComboBox* taskListBox = new QComboBox();

    QPushButton* generatePointCloudButton = new QPushButton("Generate Point Cloud");
    QPushButton* viewPointCloudButton = new QPushButton("View Point Cloud");
    QPushButton* savePointCloudButton = new QPushButton("Save Point Cloud");
    
    QPushButton* tiltOptimizeButton = new QPushButton("Tilt Optimize");
    QPushButton* fitUpperEnvelopButton = new QPushButton("Fit Upper Envelop-line");
    QPushButton* fitLowerEnvelopButton = new QPushButton("Fit Lower Envelop-line");

    QPushButton* refreshPinCenterButton = new QPushButton("Refresh Pin Center");
    QLineEdit* imgNumLineEdit = new QLineEdit();
    QLabel* pinCenterLabel = new QLabel("-1");

    DataManager* dataManager = new DataManager();
    PointCloudProducer* pcProducer = new PointCloudProducer();
};

