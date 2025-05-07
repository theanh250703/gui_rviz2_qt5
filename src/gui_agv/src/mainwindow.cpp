#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>

#include <rviz_rendering/render_window.hpp>
#include <QVector3D>
#include <QDebug>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>

MainWindow::MainWindow(QApplication *app, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , _app(app)
{
    ui->setupUi(this);
    // ui->tabWidget->addTab(new QWidget(), "Tab3");

    
    node = std::make_shared<rclcpp::Node>("qt_gui_node");

    pub_send_vel = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    connect(ui->pushButton_9, &QPushButton::clicked, this, &MainWindow::Control_Robot);

    initializeRViz();

    Control_Robot();
    SendCommand_Vel();

    // ROS spin some bằng QTimer
    QTimer *rosTimer = new QTimer(this);
    connect(rosTimer, SIGNAL(timeout()), this, SLOT(updateRos()));
    rosTimer->start(50);
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
    delete ui;
}

void MainWindow::initializeRViz()
{
    
    _rvizRosNodeTmp = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");
    _rvizRosNode = _rvizRosNodeTmp;

    QApplication::processEvents();
    _render_panel = std::make_shared<rviz_common::RenderPanel>();
    QApplication::processEvents();
    _render_panel->getRenderWindow()->initialize();
    rviz_common::WindowManagerInterface * wm = nullptr;
    auto clock = _rvizRosNode.lock()->get_raw_node()->get_clock();
    _manager = std::make_shared<rviz_common::VisualizationManager>(_render_panel.get(), _rvizRosNode, wm, clock);
    _render_panel->initialize(_manager.get());
    QApplication::processEvents();

    _manager->setFixedFrame("laser_frame");
    _manager->initialize();
    _manager->startUpdate();

    ui->scrollArea->setWidget(_render_panel.get());             //####

    // 添加网格显示
    _grid = _manager->createDisplay("rviz_default_plugins/Grid", "adjustable grid", true);
    if (_grid == NULL) {
        throw std::runtime_error("Error creating grid display");
    }

    // 配置网格样式
    _grid->subProp("Line Style")->setValue("Billboards");
    _grid->subProp("Line Style")->subProp("Line Width")->setValue(0.02f);
    _grid->subProp("Color")->setValue(QColor(Qt::white));
    _grid->subProp("Cell Size")->setValue(1.0f);

    // 添加点云显示
    _scan = _manager->createDisplay("rviz_default_plugins/LaserScan", "scan", true);
    if (_scan == NULL) {
        throw std::runtime_error("Error creating pointcloud display");
    }

    // 配置点云样式
    _scan->subProp("Topic")->setValue("/scan");
    _scan->subProp("Style")->setValue("Points");
    _scan->subProp("Size (Pixels)")->setValue(2);
    _scan->subProp("Color Transformer")->setValue("Intensity");
    _scan->subProp("Invert Rainbow")->setValue("true");
    _scan->subProp("Decay Time")->setValue("0.1");



    // ####################
    _render_panel->setMouseTracking(true);
    _render_panel->setFocusPolicy(Qt::StrongFocus);

    // Set the view controller to Orbit to allow for mouse interactions
    _manager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");

    // Retrieve the active view controller to set properties and confirm it's set up correctly
    auto orbit_view_controller = _manager->getViewManager()->getCurrent();
    if (!orbit_view_controller) {
        qDebug() << "Orbit view controller could not be set.";
        return;
    }

    qDebug() << "Orbit view controller initialized successfully.";

    // Set default distance and focal point for the camera
    orbit_view_controller->subProp("Distance")->setValue(10.0);
    orbit_view_controller->subProp("Focal Point")->setValue(QVariant::fromValue(QVector3D(0.0, 0.0, 0.0)));

    // Set initial orientation of the camera
    orbit_view_controller->subProp("Pitch")->setValue(1.5708);  // Example angle in radians
    orbit_view_controller->subProp("Yaw")->setValue(3.14);     // Example angle in radians

    // Set Interact tool as the active tool to enable mouse interactions
    auto tool_manager = _manager->getToolManager();
    tool_manager->setCurrentTool(tool_manager->addTool("rviz_default_plugins/Interact"));

}

void MainWindow::updateRos()
{
    rclcpp::spin_some(node);
}


void MainWindow::Control_Robot()
{
    control_enable = !control_enable;
    if(control_enable)
    {
        ui->pushButton_9->setText("Start");
        ui->pushButton_9->setStyleSheet("background-color: green; color: white;");

        ui->pushButton->setEnabled(true);
        ui->pushButton_2->setEnabled(true);
        ui->pushButton_3->setEnabled(true);
        ui->pushButton_4->setEnabled(true);
        
        // this: doi tuong nhan tin hieu(MainWindow), [this]: ham thuc thi khi co signal
        connect(ui->pushButton, &QPushButton::pressed, this, [this]() {                       // thang                   
            currentTwist.linear.x = ui->doubleSpinBox->value(); 
            currentTwist.angular.z = 0; 
            SendCommand();
        });

        connect(ui->pushButton_4, &QPushButton::pressed, this, [this]() {                     // lui       
            currentTwist.linear.x = -(ui->doubleSpinBox->value()); 
            currentTwist.angular.z = 0; 
            SendCommand();
        });

        connect(ui->pushButton_3, &QPushButton::pressed, this, [this]() {                     // phai         
            currentTwist.linear.x = 0; 
            currentTwist.angular.z = -(ui->doubleSpinBox_2->value()); 
            SendCommand();
        });

        connect(ui->pushButton_2, &QPushButton::pressed, this, [this]() {                     // trai     
            currentTwist.linear.x = 0; 
            currentTwist.angular.z = ui->doubleSpinBox_2->value(); 
            SendCommand();
        });

        connect(ui->pushButton, &QPushButton::released, this, [this]() {                      // nha                 
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand();
        });

        connect(ui->pushButton_2, &QPushButton::released, this, [this]() {                                    
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand();
        });

        connect(ui->pushButton_3, &QPushButton::released, this, [this]() {                                
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand();
        });

        connect(ui->pushButton_4, &QPushButton::released, this, [this]() {                                     
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand();
        });
    }
    else
    {
        ui->pushButton_9->setText("Stop");
        ui->pushButton_9->setStyleSheet("background-color: red; color: white;");
        
        // Tắt điều khiển robot
        ui->pushButton->setEnabled(false);
        ui->pushButton_2->setEnabled(false);
        ui->pushButton_3->setEnabled(false);
        ui->pushButton_4->setEnabled(false);
    }
}


void MainWindow::SendCommand_Vel()
{
    pub_send_vel->publish(currentTwist);
}