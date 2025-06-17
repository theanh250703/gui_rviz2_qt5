#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rclcpp/rclcpp.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/window_manager_interface.hpp"


#include <nav2_map_server/map_server.hpp>
#include <rclcpp/executors.hpp>
#include <memory>
#include <QProcess>

#include "std_msgs/msg/string.hpp"



QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
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
    MainWindow(QApplication *app, QWidget *parent = nullptr);
    ~MainWindow();

    void initial();
    void Control_Robot_Manual();
    void SendCommand_Vel();
    void setupRobotModelDisplay();
    void setmap();
    void setView(const QString &view_mode);
    void set_nav();

    void select_table();
    void move_table();
    void remove_table();
    void move_finish();

    // SLAM
    void start_slam();
    void quit_slam();
    void save_map();

    // void delete_map();

private:
    Ui::MainWindow *ui;
    QApplication * _app;

    bool control_enable = true;
    geometry_msgs::msg::Twist currentTwist;

    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> _rvizRosNodeTmp;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr _rvizRosNode;
    
    std::shared_ptr<rviz_common::VisualizationManager> _manager;
    std::shared_ptr<rviz_common::RenderPanel> _render_panel;


    rviz_common::Display * _grid;
    rviz_common::Display * _pointcloud;
    rviz_common::Display * robot_model_display;
    rviz_common::Display * path;
    rviz_common::Display * map_display;

    QProcess *robot_process = nullptr;
    QProcess *localization_process = nullptr;
    QProcess *navigation_process = nullptr;
    QProcess *slam_process = nullptr;
    QProcess *save_map_process = nullptr;

    rviz_common::Tool *initial_pose_tool = nullptr;
    rviz_common::Tool *nav_goal_tool = nullptr;

    QStringList table_list;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr table_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr finish_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr confirm;


    void confirm_callback(const std_msgs::msg::String::SharedPtr msg);

};
#endif // MAINWINDOW_H