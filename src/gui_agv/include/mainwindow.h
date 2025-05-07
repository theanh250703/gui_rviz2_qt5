#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/window_manager_interface.hpp"

QT_BEGIN_NAMESPACE
namespace Ui 
{  
    class MainWindow; 
}

namespace rviz_common
{
    class Display;
    class RenderPanel;
    class VisualizationManager;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QApplication * app, QWidget *parent = nullptr);
    ~MainWindow();

    std::shared_ptr<rviz_common::RenderPanel> _render_panel;

    void initializeRViz();

    // QWidget * getParentWindow() override;
    // rviz_common::PanelDockWidget * addPane(const QString & name, QWidget * pane, Qt::DockWidgetArea area, bool floating) override;
    // void setStatus(const QString & message) override;

public slots:
    void updateRos();
    void Control_Robot();
    void SendCommand_Vel();
    
    // void closeEvent(QCloseEvent *event);

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node;
    bool control_enable = true;


    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_send_vel;
    geometry_msgs::msg::Twist currentTwist;

    
    QApplication * _app;

    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> _rvizRosNodeTmp;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr _rvizRosNode;
    std::shared_ptr<rviz_common::VisualizationManager> _manager;
    
    rviz_common::Display * _grid;
    rviz_common::Display * _scan;
};

#endif // MAINWINDOW_H
