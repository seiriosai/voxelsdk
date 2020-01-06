#include "mainwindow.h"
#include <QApplication>
extern int qttofmain();
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    qttofmain();
    return a.exec();
}
