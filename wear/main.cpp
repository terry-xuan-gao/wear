#include <QCoreApplication>
#include <QApplication>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <QLabel>
#include <QFontDatabase>
#include <QFont>

#include "wear.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    QFont font = QFontDatabase::systemFont(QFontDatabase::SystemFont::GeneralFont);
    QString fontFamily = font.family();
    font.setFamily("Helvetica");
    font.setPointSize(10); 
    a.setFont(font);

    wear* wp = new wear();
    wp->run();

    return a.exec();
}
