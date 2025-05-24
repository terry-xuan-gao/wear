#include "Ansys.h"

Ansys::Ansys(QWidget* parent)
	: QWidget(parent)
{
	this->resize(400, 100);

	this->initTaskList();
	this->initPushButtons();
	this->initValueDisplay();
	
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
	
	buttonLayout->addWidget(this->generatePointCloudButton);
	buttonLayout->addWidget(this->viewPointCloudButton);
	buttonLayout->addWidget(this->savePointCloudButton);
	buttonLayout->addWidget(this->poissonReconstuctionButton);

	this->viewPointCloudButton->setEnabled(true);
	this->savePointCloudButton->setEnabled(false);

	this->poissonReconstuctionButton->setVisible(false);

	connect(this->generatePointCloudButton, &QPushButton::clicked,
		this, &Ansys::generatePointCloudButtonClicked);
	connect(this->viewPointCloudButton, &QPushButton::clicked,
		this, &Ansys::viewPointCloudButtonClicked);
	connect(this->savePointCloudButton, &QPushButton::clicked,
		this, &Ansys::savePointCloudButtonClicked);
	connect(this->refreshValueButton, &QPushButton::clicked,
		this, &Ansys::refreshValueButtonClicked);
	connect(this->setAsButton, &QPushButton::clicked,
		this, &Ansys::setAsButtonClicked);
	
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

	QLabel* label = new QLabel("Task");
	//label->setAlignment(Qt::AlignCenter);

	QHBoxLayout* boxLayout = new QHBoxLayout();
	boxLayout->addWidget(label);
	boxLayout->addWidget(taskListBox);

	this->layout->addLayout(boxLayout);
}

void Ansys::initValueDisplay()
{
	QLabel* label = new QLabel("Value");

	QHBoxLayout* valueLayout = new QHBoxLayout();
	valueLayout->addWidget(label);
	valueLayout->addWidget(this->valueLabel);
	valueLayout->addWidget(this->refreshValueButton);


	unordered_map<string, double> valueList = dataManager->loadValueList();

	for (auto kv : valueList) {
		QString qStr = QString::fromStdString(kv.first);
		StandardValueBox->addItem(qStr);
	}

	valueLayout->addWidget(this->setAsButton);
	valueLayout->addWidget(this->StandardValueBox);

	this->setAsButton->setEnabled(false);
	
	this->layout->addLayout(valueLayout);
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

void Ansys::refreshValueButtonClicked() {
	double value = this->pcProducer->getToolValue();

	QString text = QString::number(value, 'f', 2);
	this->valueLabel->setText(QString("<u>%1</u>").arg(value, 0, 'f', 2));

	this->setAsButton->setEnabled(true);
}

void Ansys::setAsButtonClicked() {
	QString currentText = this->StandardValueBox->currentText();
	string name = currentText.toStdString();

	bool ok;
	double value = this->pcProducer->getToolValue();

	dataManager->setValue(name, value);
}

void Ansys::poissonReconstuctionButtonClicked()
{
	this->pcProducer->reconstruction();
}

