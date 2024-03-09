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
	
	QHBoxLayout* pinCenterLayout = new QHBoxLayout();
	pinCenterLayout->addWidget(this->refreshPinCenterButton);
	this->pinCenterLabel->setAlignment(Qt::AlignCenter);
	//pinCenterLayout->addWidget(this->tiltOptimizeButton);
	pinCenterLayout->addWidget(this->pinCenterLabel);
	pinCenterLayout->addWidget(this->imgNumLineEdit);
	buttonLayout->addLayout(pinCenterLayout);

	this->refreshPinCenterButton->setVisible(false);
	this->pinCenterLabel->setVisible(false);
	this->imgNumLineEdit->setVisible(false);

	QHBoxLayout* envelopLayout = new QHBoxLayout();
	envelopLayout->addWidget(this->fitUpperEnvelopButton);
	envelopLayout->addWidget(this->fitLowerEnvelopButton);
	buttonLayout->addLayout(envelopLayout);

	buttonLayout->addWidget(this->tiltOptimizeButton);
	this->fitLowerEnvelopButton->setEnabled(false);
	this->tiltOptimizeButton->setEnabled(false);

	QVBoxLayout* pointCloudLayout = new QVBoxLayout();
	pointCloudLayout->addWidget(this->generatePointCloudButton);
	pointCloudLayout->addWidget(this->savePointCloudButton);
	pointCloudLayout->addWidget(this->viewPointCloudButton);
	buttonLayout->addLayout(pointCloudLayout);

	this->viewPointCloudButton->setEnabled(false);
	this->savePointCloudButton->setEnabled(false);

	connect(this->generatePointCloudButton, &QPushButton::clicked,
		this, &Ansys::generatePointCloudButtonClicked);
	connect(this->viewPointCloudButton, &QPushButton::clicked,
		this, &Ansys::viewPointCloudButtonClicked);
	connect(this->savePointCloudButton, &QPushButton::clicked,
		this, &Ansys::savePointCloudButtonClicked);

	connect(this->refreshPinCenterButton, &QPushButton::clicked,
		this, &Ansys::refreshPinCenter);

	connect(this->fitUpperEnvelopButton, &QPushButton::clicked,
		this, &Ansys::fitUpperEnvelopButtonClicked);
	connect(this->fitLowerEnvelopButton, &QPushButton::clicked,
		this, &Ansys::fitLowerEnvelopButtonClicked);
	connect(this->tiltOptimizeButton, &QPushButton::clicked,
		this, &Ansys::tiltOptimize);
	
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
	this->pcProducer->viewPointCloud();
}

void Ansys::savePointCloudButtonClicked()
{
	this->pcProducer->savePointCloud();
	//this->viewPointCloudButton->setEnabled(true);
}

void Ansys::refreshPinCenter()
{
	QString currentText = this->taskListBox->currentText();
	string chosenTaskName = currentText.toStdString();

	this->pcProducer->getTaskName(chosenTaskName);

	QString imgNum = this->imgNumLineEdit->text();
	
	double pinCenter = this->pcProducer->getPinCenter(imgNum.toInt());
	this->pinCenterLabel->setText(QString::number(pinCenter));

	QPalette pal = this->pinCenterLabel->palette();
	pal.setColor(QPalette::Foreground, QColor(0, 0, 255));
	this->pinCenterLabel->setPalette(pal);
}

void Ansys::tiltOptimize()
{
	qDebug() << "Tilt Optimize ";
	this->pcProducer->tiltOptimize();
}

void Ansys::fitUpperEnvelopButtonClicked()
{
	QString currentText = this->taskListBox->currentText();
	string chosenTaskName = currentText.toStdString();
	this->pcProducer->getTaskName(chosenTaskName);
	
	qDebug() << "Fit Upper Envelop-line clicked";
	this->pcProducer->fitEnvelopOfPinEnvelop(0);

	this->fitLowerEnvelopButton->setEnabled(true);
}

void Ansys::fitLowerEnvelopButtonClicked()
{
	QString currentText = this->taskListBox->currentText();
	string chosenTaskName = currentText.toStdString();
	this->pcProducer->getTaskName(chosenTaskName);
	
	qDebug() << "Fit Lower Envelop-line clicked";
	this->pcProducer->fitEnvelopOfPinEnvelop(1);

	this->tiltOptimizeButton->setEnabled(true);
}
