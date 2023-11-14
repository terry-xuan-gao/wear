#include "Capture.h"

int height = 800;
int width = 600;

Capture::Capture(QWidget* parent)
    : QWidget(parent)
{
    this->resize(800, 600);

    this->initStatusLabel();
    this->initButtons();
}

Capture::~Capture()
{

}

void Capture::initButtons()
{
    this->startButton = new QPushButton("START");
    this->ansysButton = new QPushButton("ANSYS");

    connect(this->startButton, &QPushButton::clicked,
        this, &Capture::startButtonClicked);

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(startButton);
    layout->addWidget(ansysButton);
    this->setLayout(layout);
}

void Capture::initStatusLabel()
{
    this->statusLabel = new QLabel("STATUS: OK", this);
    this->statusLabel->setGeometry(0, this->height() - 30, this->width(), 30);
    this->statusLabel->setAlignment(Qt::AlignBottom | Qt::AlignLeft);
    this->statusLabel->show();
}

void Capture::captureTask()
{
    this->statusLabel->setText("STATUS: capturing ... please wait for a will.");

    


}

void Capture::startButtonClicked()
{
    this->statusLabel->setText("STATUS: ready for capture!");
    this->captureTask();
}