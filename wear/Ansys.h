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

#include <vector>

#include "DataManager.h"

using namespace std;

class Ansys :
    public QWidget
{
    Q_OBJECT

public:
    Ansys(QWidget* parent = nullptr);
    ~Ansys();

    void initTaskList();

private:
    QVBoxLayout* layout = new QVBoxLayout();

    QComboBox* taskListBox = new QComboBox();

    DataManager* dataManager = new DataManager();
};

