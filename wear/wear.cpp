#include "wear.h"

wear::wear(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);

    this->layout = new QVBoxLayout();

    this->setFixedSize(950, 700);

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
    int fixedWidth = 931;
    int fixedHeight = 126;
    imageDisplayLabel_hit->setMinimumSize(fixedWidth, fixedHeight);
    imageDisplayLabel_hit->setMaximumSize(fixedWidth, fixedHeight);    

    QImage image("..\\ui\\tail2.jpg");
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaledPixMap = pixmap.scaledToWidth(this->width(), Qt::FastTransformation);

    imageDisplayLabel_hit->setPixmap(scaledPixMap);
    imageDisplayLabel_hit->setAlignment(Qt::AlignCenter);

    this->layout->addWidget(imageDisplayLabel_hit, 0, Qt::AlignHCenter);
}

void wear::initDisplayLabel_name()
{
    this->imageDisplayLabel_name = new QLabel("name");

    imageDisplayLabel_name->setScaledContents(true);
    
    // 61 : 6
    int fixedWidth = 915;
    int fixedHeight = 90;
    imageDisplayLabel_name->setMinimumSize(fixedWidth, fixedHeight);
    imageDisplayLabel_name->setMaximumSize(fixedWidth, fixedHeight);

    QImage image("..\\ui\\name3.jpg");
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaledPixMap = pixmap.scaledToWidth(fixedWidth, Qt::FastTransformation);

    imageDisplayLabel_name->setPixmap(scaledPixMap);
    imageDisplayLabel_name->setAlignment(Qt::AlignCenter);

    this->layout->addWidget(imageDisplayLabel_name, 0, Qt::AlignHCenter);
}


void wear::initDisplayLabel_changke()
{
    this->imageDisplayLabel_changke = new QLabel("image");

    imageDisplayLabel_changke->setScaledContents(true);
    int fixedWidth = 760;
    int fixedHeight = 260;
    imageDisplayLabel_changke->setMinimumSize(fixedWidth, fixedHeight);
    imageDisplayLabel_changke->setMaximumSize(fixedWidth, fixedHeight);

    QImage image("..\\ui\\title4.jpg");
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaledPixMap = pixmap.scaledToWidth(this->width(), Qt::FastTransformation);

    imageDisplayLabel_changke->setPixmap(scaledPixMap);
    imageDisplayLabel_changke->setAlignment(Qt::AlignCenter);
    
    this->layout->addWidget(imageDisplayLabel_changke, 0, Qt::AlignHCenter);
}


void wear::captureButtonClicked()
{
    this->layout->addWidget(captureWindow);
    this->captureWindow->show();

    this->imageDisplayLabel_hit->setVisible(false);
    this->imageDisplayLabel_changke->setVisible(false);

    this->ansysWindow->setVisible(false);
    this->captureWindow->setVisible(true);
}

void wear::ansysButtonClicked()
{
    this->layout->addWidget(ansysWindow);
    this->ansysWindow->refreshTaskList();
    this->ansysWindow->show();

    this->imageDisplayLabel_hit->setVisible(false);
    this->imageDisplayLabel_changke->setVisible(false);

    this->ansysWindow->setVisible(true);
    this->captureWindow->setVisible(false);
    
    
}
