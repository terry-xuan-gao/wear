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
#include <QMessageBox>


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
    void initValueDisplay();

    void refreshTaskList();

private slots:
    void generatePointCloudButtonClicked();
    void viewPointCloudButtonClicked();
    void savePointCloudButtonClicked();

    void refreshValueButtonClicked();
    void setAsButtonClicked();

    void poissonReconstuctionButtonClicked();

private:
    QVBoxLayout* layout = new QVBoxLayout();

    QComboBox* taskListBox = new QComboBox();

    QPushButton* generatePointCloudButton = new QPushButton("Generate Point Cloud");
    QPushButton* viewPointCloudButton = new QPushButton("View Point Cloud");
    QPushButton* savePointCloudButton = new QPushButton("Save Point Cloud");

    QPushButton* poissonReconstuctionButton = new QPushButton("Reconstruct Surface (Poisson)");

    DataManager* dataManager = new DataManager();
    PointCloudProducer* pcProducer = new PointCloudProducer();

    QPushButton* refreshValueButton = new QPushButton("Refresh");
    QLabel* valueLabel = new QLabel("----");

    QPushButton* setAsButton = new QPushButton("Set As");
    QComboBox* StandardValueBox = new QComboBox();
};

