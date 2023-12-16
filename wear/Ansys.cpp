#include "Ansys.h"

Ansys::Ansys(QWidget* parent)
	: QWidget(parent)
{
	this->resize(300, 450);

	this->initTaskList();
	
	this->setLayout(layout);
}

Ansys::~Ansys()
{
	delete this->layout;
	delete this->taskListBox;
	delete this->dataManager;
}

void Ansys::initTaskList()
{
	taskListBox->setToolTip("ALL TASK");

	vector<vector<string>> taskList = dataManager->loadTaskList();

	for (auto task : taskList)
	{
		QString qStr = QString::fromStdString(task[0]);
		taskListBox->addItem(qStr);
	}

	QLabel* label = new QLabel("ALL TASK");

	QHBoxLayout* boxLayout = new QHBoxLayout();
	boxLayout->addWidget(label);
	boxLayout->addWidget(taskListBox);

	this->layout->addLayout(boxLayout);
}

void Ansys::refreshTaskList()
{
	taskListBox->clear();

	vector<vector<string>> taskList = dataManager->getTaskList();

	for (auto task : taskList)
	{
		QString qStr = QString::fromStdString(task[0]);
		taskListBox->addItem(qStr);
	}
}