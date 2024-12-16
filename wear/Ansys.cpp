#include "Ansys.h"

Ansys::Ansys(QWidget* parent)
	: QWidget(parent)
{
	this->resize(400, 300);

	this->initTaskList();
	this->initPushButtons();
	
	this->setLayout(layout);
}

Ansys::~Ansys()
{
	delete this->layout;
	delete this->taskListBox;
	delete this->dataManager;

	delete this->generatePointCloudButton;
	delete this->viewPointCloudButton;
	delete this->savePointCloudButton;
}

void Ansys::initPushButtons()
{
	QVBoxLayout* buttonLayout = new QVBoxLayout();
	
	QVBoxLayout* pointCloudLayout = new QVBoxLayout();
	pointCloudLayout->addWidget(this->generatePointCloudButton);
	pointCloudLayout->addWidget(this->viewPointCloudButton);
	pointCloudLayout->addWidget(this->savePointCloudButton);
	pointCloudLayout->addWidget(this->poissonReconstuctionButton);
	buttonLayout->addLayout(pointCloudLayout);

	this->viewPointCloudButton->setEnabled(true);
	this->savePointCloudButton->setEnabled(false);

	this->poissonReconstuctionButton->setVisible(false);

	connect(this->generatePointCloudButton, &QPushButton::clicked,
		this, &Ansys::generatePointCloudButtonClicked);
	connect(this->viewPointCloudButton, &QPushButton::clicked,
		this, &Ansys::viewPointCloudButtonClicked);
	connect(this->savePointCloudButton, &QPushButton::clicked,
		this, &Ansys::savePointCloudButtonClicked);
	
	connect(this->poissonReconstuctionButton, &QPushButton::clicked,
		this, &Ansys::poissonReconstuctionButtonClicked);
	
	this->layout->addLayout(buttonLayout);
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
	label->setAlignment(Qt::AlignCenter);

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

void Ansys::generatePointCloudButtonClicked()
{
	QString currentText = this->taskListBox->currentText();
	string chosenTaskName = currentText.toStdString();

	this->pcProducer->getTaskName(chosenTaskName);
	this->pcProducer->generatePointCloud();

	this->viewPointCloudButton->setEnabled(true);
	this->savePointCloudButton->setEnabled(true);
}

void Ansys::viewPointCloudButtonClicked()
{
	if (this->savePointCloudButton->isEnabled() == false) {
		QString fileName = QFileDialog::getOpenFileName(
			this,
			"open a file.",
			"D:/gaoxuan/wear0.1/wear/data/",
			tr("point cloud(*.pcd);;video files(*.avi *.mp4 *.wmv);;All files(*.*)"));
		if (fileName.isEmpty()) {
			QMessageBox::warning(this, "Warning!", "Failed to open the video!");
		}

		qDebug() << fileName;
		this->pcProducer->viewPointCloud(fileName.toStdString());
	}
	else {
		this->pcProducer->viewPointCloud();
	}
	
}

void Ansys::savePointCloudButtonClicked()
{
	this->pcProducer->savePointCloud();
}

void Ansys::poissonReconstuctionButtonClicked()
{
	this->pcProducer->reconstruction();
}

