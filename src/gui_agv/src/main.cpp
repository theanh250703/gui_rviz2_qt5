#include "mainwindow.h"
#include <QApplication>


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    MainWindow w(&app);
    w.show();

    int result = app.exec();
    rclcpp::shutdown();

    return result;
}
