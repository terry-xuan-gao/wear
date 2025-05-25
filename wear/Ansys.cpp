#include "Ansys.h"

Ansys::Ansys(QWidget* parent)
	: QWidget(parent)
{
	this->resize(400, 100);

	this->initTaskList();
	this->initPushButtons();
	this->initValueDisplay();
	this->initProgressBar();
	
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
	buttonLayout->addWidget(this->generateLabel);
	this->generateLabel->resize(generateLabel->width(), 20);
	buttonLayout->addWidget(this->viewPointCloudButton);
	buttonLayout->addWidget(this->savePointCloudButton);
	buttonLayout->addWidget(this->poissonReconstuctionButton);

	this->viewPointCloudButton->setEnabled(false);
	this->savePointCloudButton->setEnabled(false);

	this->generateLabel->setVisible(false);
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

	unordered_map<string, double> valueList = dataManager->loadValueList();

	for (auto kv : valueList) {
		QString qStr = QString::fromStdString(kv.first);
		StandardValueBox->addItem(qStr);
	}

	boxLayout->addWidget(this->setAsButton);
	boxLayout->addWidget(this->StandardValueBox);

	this->setAsButton->setEnabled(false);

	connect(taskListBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, &Ansys::taskListBoxIndexChanged);

	this->layout->addLayout(boxLayout);
}

void Ansys::initValueDisplay()
{
	QLabel* label = new QLabel("Value");

	QHBoxLayout* valueLayout = new QHBoxLayout();
	valueLayout->addWidget(label);
	valueLayout->addWidget(this->valueLabel);
	valueLayout->addWidget(this->percentLabel);
	valueLayout->addWidget(this->refreshValueButton);
	
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
	this->generateLabel->setVisible(true);
	
	progressBar->setValue(10);
	
	QString currentText = this->taskListBox->currentText();
	string chosenTaskName = currentText.toStdString();

	progressBar->setValue(15);

	this->pcProducer->getTaskName(chosenTaskName);
	this->pcProducer->generatePointCloud();

	progressBar->setValue(90);

	this->viewPointCloudButton->setEnabled(true);
	this->savePointCloudButton->setEnabled(true);

	this->refreshValueButtonClicked();
	progressBar->setValue(100);

	this->generateLabel->setVisible(false);
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

void Ansys::taskListBoxIndexChanged(int index)
{
	this->setAsButton->setEnabled(false);
	this->viewPointCloudButton->setEnabled(false);
	this->savePointCloudButton->setEnabled(false);
	this->valueLabel->setText(QString("----"));
	this->percentLabel->setText(QString("---%"));
}

void Ansys::refreshValueButtonClicked() {
	double value = this->pcProducer->getToolValue();

	QString text = QString::number(value, 'f', 2);
	this->valueLabel->setText(QString("<u>%1</u>").arg(value, 0, 'f', 2));

	QString currentText = this->StandardValueBox->currentText();
	string name = currentText.toStdString();
	double standeredValue = this->dataManager->getValue(name);

	qDebug() << "value = " << value << ", standard value = " << standeredValue;

	double percent = (value / standeredValue) * 100;
	this->percentLabel->setText(QString("<u>%1</u>%").arg(percent, 0, 'f', 2));


	this->setAsButton->setEnabled(true);
}

void Ansys::setAsButtonClicked() {
	QString currentText = this->StandardValueBox->currentText();
	string name = currentText.toStdString();

	double value = this->pcProducer->getToolValue();

	dataManager->setValue(name, value);
}

void Ansys::poissonReconstuctionButtonClicked()
{
	this->pcProducer->reconstruction();
}

void Ansys::initProgressBar()
{
	this->progressBar = new QProgressBar();
	this->layout->addWidget(this->progressBar);

	progressBar->setRange(0, 100);
	progressBar->setValue(0);
}