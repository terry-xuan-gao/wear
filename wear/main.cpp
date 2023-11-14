#include <QCoreApplication>
#include <QApplication>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <QLabel>

#include "wear.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    wear* wp = new wear();
    wp->run();

    return a.exec();
}
