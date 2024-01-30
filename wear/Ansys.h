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

#include <vector>

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

private:
    QVBoxLayout* layout = new QVBoxLayout();

    QComboBox* taskListBox = new QComboBox();

    QPushButton* generatePointCloudButton = new QPushButton("Generate Point Cloud");
    QPushButton* viewPointCloudButton = new QPushButton("View Point Cloud");
    QPushButton* savePointCloudButton = new QPushButton("Save Point Cloud");

    DataManager* dataManager = new DataManager();
    PointCloudProducer* pcProducer = new PointCloudProducer();
};

