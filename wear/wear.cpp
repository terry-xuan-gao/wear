#include "wear.h"

wear::wear(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);

    this->resize(400, 300);
    this->initButtons();
}

wear::~wear()
{
    delete this->captureButton;
    delete this->captureWindow;
    delete this->ansysButton;
    delete this->ansysWindow;
}

void wear::run()
{
    this->show();

    connect(this->captureButton, &QPushButton::clicked, 
        this, &wear::captureButtonClicked);

    connect(this->ansysButton, &QPushButton::clicked,
        this, &wear::ansysButtonClicked);
}

void wear::initButtons()
{
    this->captureWindow->setWindowTitle("CAPTURE");
    this->captureButton = new QPushButton("CAPTURE");


    this->ansysWindow->setWindowTitle("ANSYS");
    this->ansysButton = new QPushButton("ANSYS");


    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(captureButton);
    layout->addWidget(ansysButton);
    this->setLayout(layout);
}

void wear::captureButtonClicked()
{
    this->captureWindow->show();
}

void wear::ansysButtonClicked()
{
    this->ansysWindow->show();
}
