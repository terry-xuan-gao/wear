#include "wear.h"

wear::wear(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);

    this->layout = new QVBoxLayout();

    this->resize(1000, 600);

    this->initDisplayLabel_changke();
    this->initDisplayLabel_name();
    
    this->initButtons();
    this->initDisplayLabel_hit();
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


    this->ansysWindow->setWindowTitle("ANALYSE");
    this->ansysButton = new QPushButton("ANALYSE");

    connect(this->ansysButton, &QPushButton::clicked,
        this, &wear::ansysButtonClicked);

    this->layout->addWidget(captureButton);
    this->layout->addWidget(ansysButton);
    this->setLayout(this->layout);
}

void wear::initDisplayLabel_hit()
{
    this->imageDisplayLabel_hit = new QLabel("image");

    imageDisplayLabel_hit->setScaledContents(true);
    //imageDisplayLabel_hit->resize(400, 55);

    

    QImage image("..\\ui\\tail2.jpg");

    //image = (image).scaled(400, 55, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaledPixMap = pixmap.scaledToWidth(this->width(), Qt::FastTransformation);

    imageDisplayLabel_hit->setPixmap(scaledPixMap);
    imageDisplayLabel_hit->setAlignment(Qt::AlignCenter);

    this->layout->addWidget(imageDisplayLabel_hit);
}

void wear::initDisplayLabel_name()
{
    this->imageDisplayLabel_name = new QLabel("name");

    imageDisplayLabel_name->setScaledContents(true);
    //imageDisplayLabel_hit->resize(400, 55);



    QImage image("..\\ui\\name3.jpg");

    //image = (image).scaled(400, 55, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaledPixMap = pixmap.scaledToWidth(this->width(), Qt::FastTransformation);

    imageDisplayLabel_name->setPixmap(scaledPixMap);
    imageDisplayLabel_name->setAlignment(Qt::AlignCenter);

    this->layout->addWidget(imageDisplayLabel_name);
}


void wear::initDisplayLabel_changke()
{
    this->imageDisplayLabel_changke = new QLabel("image");

    imageDisplayLabel_changke->setScaledContents(true);
    //imageDisplayLabel_changke->resize(20, 200);

    QImage image("..\\ui\\title4.jpg");

    //image = (image).scaled(20, 200, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaledPixMap = pixmap.scaledToWidth(this->width(), Qt::FastTransformation);

    imageDisplayLabel_changke->setPixmap(scaledPixMap);
    imageDisplayLabel_changke->setAlignment(Qt::AlignCenter);
    
    this->layout->addWidget(imageDisplayLabel_changke);
}


void wear::captureButtonClicked()
{
    this->captureWindow->show();
}

void wear::ansysButtonClicked()
{
    this->ansysWindow->refreshTaskList();
    this->ansysWindow->show();
}
