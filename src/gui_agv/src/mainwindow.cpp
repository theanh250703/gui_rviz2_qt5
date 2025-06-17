#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <rviz_rendering/render_window.hpp>
#include <QVector3D>
#include <QDebug>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>

#include <QProcess>
#include <QDir>

#include "rclcpp/rclcpp.hpp"

MainWindow::MainWindow(QApplication * app, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , _app(app)
{
    ui->setupUi(this);

    initial();
    setupRobotModelDisplay();
    setmap();
    set_nav();

    auto raw_node = _rvizRosNodeTmp->get_raw_node();                    // tra ve kieu std::shared_ptr<rclcpp::Node>

    // gui ten cua table sang waypoint
    table_publisher = raw_node->create_publisher<std_msgs::msg::String>("selected_table", 10);

    confirm = raw_node->create_subscription<std_msgs::msg::String>("gui_node", 10, std::bind(&MainWindow::confirm_callback, this, std::placeholders::_1));

    finish_publisher = raw_node->create_publisher<std_msgs::msg::String>("finish_node", 10);

    vel_publisher = raw_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    connect(ui->comboBox, &QComboBox::currentTextChanged, this, &MainWindow::setmap);
    
    connect(ui->pushButton_14, &QPushButton::clicked, this, [=](){
        setView("2D");
    });
    
    connect(ui->pushButton_15, &QPushButton::clicked, this, [=](){
        setView("3D");
    });

    connect(ui->pushButton_20, &QPushButton::clicked, this, [=]() {
        _manager->getToolManager()->setCurrentTool(initial_pose_tool);
    });

    connect(ui->pushButton_6, &QPushButton::clicked, this, [=]() {
        _manager->getToolManager()->setCurrentTool(nav_goal_tool);
    });

    // control robot manual
    connect(ui->pushButton_9, &QPushButton::clicked, this, &MainWindow::Control_Robot_Manual);

    // nhap ten map moi cho save
    ui->pushButton_25->setDisabled(ui->lineEdit->text().trimmed().isEmpty());
    connect(ui->lineEdit, &QLineEdit::textChanged, this, [=](const QString &text) {
    ui->pushButton_25->setDisabled(text.trimmed().isEmpty());
    });


    connect(ui->pushButton_19, &QPushButton::clicked, this, &MainWindow::select_table);
    connect(ui->pushButton_22, &QPushButton::clicked, this, &MainWindow::remove_table);
    connect(ui->pushButton_21, &QPushButton::clicked, this, &MainWindow::move_table);
    connect(ui->pushButton_18, &QPushButton::clicked, this, &MainWindow::move_finish);

    // connect button to slam
    connect(ui->pushButton_5, &QPushButton::clicked, this, &MainWindow::start_slam);
    connect(ui->pushButton_24,&QPushButton::clicked, this, &MainWindow::quit_slam);
    connect(ui->pushButton_25,&QPushButton::clicked, this, &MainWindow::save_map);
}

MainWindow::~MainWindow()
{
    quit_slam();
    delete ui;
    kill(0, SIGINT);                            // dung toan bo tien trinh
    // delete _manager;
    // delete _render_panel;
}


void MainWindow::initial()
{
    _rvizRosNodeTmp = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("gui_node");
    _rvizRosNode = _rvizRosNodeTmp;

    QApplication::processEvents();                                          // tranh treo UI trong luc khoi tao rviz
    _render_panel = std::make_shared<rviz_common::RenderPanel>();           // khoi tao rendel panel hien thi rviz
    QApplication::processEvents();
    _render_panel->getRenderWindow()->initialize();

    rviz_common::WindowManagerInterface * wm = nullptr;
    auto clock = _rvizRosNode.lock()->get_raw_node()->get_clock();
    _manager = std::make_shared<rviz_common::VisualizationManager>(_render_panel.get(), _rvizRosNode, wm, clock);
    _render_panel->initialize(_manager.get());
    QApplication::processEvents();

    _manager->setFixedFrame("/map");
    _manager->initialize();
    _manager->startUpdate();

    ui->scrollArea->setWidget(_render_panel.get());

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
    _pointcloud = _manager->createDisplay("rviz_default_plugins/LaserScan", "scan", true);
    if (_pointcloud == NULL) {
        throw std::runtime_error("Error creating pointcloud display");
    }

    // 配置点云样式
    _pointcloud->subProp("Topic")->setValue("/laser_controller/out");
    _pointcloud->subProp("Style")->setValue("Points");
    _pointcloud->subProp("Size (Pixels)")->setValue(2);
    _pointcloud->subProp("Color Transformer")->setValue("Intensity");
    _pointcloud->subProp("Invert Rainbow")->setValue("true");
    _pointcloud->subProp("Decay Time")->setValue("0.1");


    // Path
    path = _manager->createDisplay("rviz_default_plugins/Path", "plan", true);
    if (path == NULL) {
        throw std::runtime_error("Error creating path display");
    }
    path->subProp("Topic")->setValue("/plan");
    path->subProp("Line Style")->setValue("Billboards");
    path->subProp("Line Width")->setValue(0.1);
    path->subProp("Color")->setValue("0 255 0");
    // ##########################################
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
    orbit_view_controller->subProp("Yaw")->setValue(3.14);      // Example angle in radians

    // Set Interact tool as the active tool to enable mouse interactions
    auto tool_manager = _manager->getToolManager();
    tool_manager->setCurrentTool(tool_manager->addTool("rviz_default_plugins/Interact"));
}


void MainWindow::setupRobotModelDisplay()
{
    robot_model_display = _manager->createDisplay("rviz_default_plugins/RobotModel", "RobotModel Display", true);
    
    if(robot_model_display)
    {
        robot_model_display -> subProp("Description Topic")->setValue("/robot_description");
        qDebug() << "RobotModel display configured for /robot_description topic.";
    }

    else
    {
        qDebug() << "Failed to create RobotModel display.";
    }
}



void MainWindow::setmap()
{
    auto name_map = ui->comboBox->currentText();

    QString map_yaml;
    if (name_map == "map1")
        map_yaml = "/home/theanh/maps/test_map.yaml";
    else if (name_map == "map2")
        map_yaml = "/home/theanh/maps/map_test_wall.yaml";
    else if (name_map == "map3")
        map_yaml = "/home/theanh/maps/my_map.yaml";
    else
        return;
    
    ////////// localization process
    if(localization_process)
    {
        localization_process->kill();
        localization_process->waitForFinished(3000);
        delete localization_process;
        localization_process = nullptr;
    }

    localization_process = new QProcess(this);

    QStringList localization_args;
    localization_args << "launch"
                      << "articubot_one"
                      << "localization_launch.py"
                      << QString("map:=%1").arg(map_yaml);
    
    localization_process->setProgram("ros2");
    localization_process->setArguments(localization_args);
    localization_process->start();


    if (!localization_process->waitForStarted(3000)) {
        qDebug() << "Không thể khởi động localization_process";
        return;
        }
    qDebug() << "localization_process đã được khởi động";


    ////////// navigation process
    if(navigation_process)
    {
        navigation_process->kill();
        navigation_process->waitForFinished(3000);
        delete navigation_process;
        navigation_process = nullptr;
    }

    navigation_process = new QProcess(this);

    QStringList navigation_args;
    navigation_args << "launch"
                    << "articubot_one"
                    << "navigation_launch.py";
    
    navigation_process->setProgram("ros2");
    navigation_process->setArguments(navigation_args);
    navigation_process->start();


    ////////// robot process
    if(robot_process)
    {
        robot_process->kill();
        robot_process->waitForFinished(3000);
        delete robot_process;
        robot_process = nullptr;
    }

    robot_process = new QProcess(this);

    QStringList robot_args;
    robot_args << "launch"
               << "articubot_one"
               << "launch_sim.launch.py";
    
    robot_process->setProgram("ros2");
    robot_process->setArguments(robot_args);
    robot_process->start();

    // Config display map
    auto map_display = _manager->createDisplay("rviz_default_plugins/Map", "Map Display", true);
    if (map_display)
    {
        map_display->subProp("Topic")->setValue("/map");
        qDebug() << "RViz show map.";
        QApplication::processEvents();
    }
    else
    {
        qDebug() << "Can't create map.";
    }
}


void MainWindow::setView(const QString &view_mode)
{
    if(view_mode == "2D")
    {
        _manager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/TopDownOrtho");
        _manager->getViewManager()->getCurrent()->subProp("Scale")->setValue(40);
    }

    else if(view_mode == "3D")
    {
        _manager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");
    }
    else
    {
        qDebug() << "Fail to set View";
    }
}

void MainWindow::set_nav()
{
    auto tool_manager = _manager->getToolManager();
    initial_pose_tool = tool_manager->addTool("rviz_default_plugins/SetInitialPose");
    nav_goal_tool = tool_manager->addTool("rviz_default_plugins/SetGoal");

    initial_pose_tool->setName("InitialPose");
    nav_goal_tool->setName("NavGoal");
}

void MainWindow::select_table()
{
    auto name_table = ui->comboBox_2->currentText();
    if(!table_list.contains(name_table))
    {
        table_list.append(name_table);
        ui->listWidget->addItem(name_table);
        qDebug()<<"Add table: "<<name_table;
    }
    else
    {
        qDebug()<<"Table already in list";
    }
}


void MainWindow::remove_table()
{
    QList<QListWidgetItem*> selectedItems = ui->listWidget->selectedItems();
    for (QListWidgetItem* item : selectedItems)
    {
        delete ui->listWidget->takeItem(ui->listWidget->row(item));
    }
}


void MainWindow::move_table()
{
    if(table_list.isEmpty())
    {
        qDebug()<<"Emty list";                      // gui data ve vi tri home
        auto message = std_msgs::msg::String();
        message.data = "home";
        table_publisher->publish(message);
    }

    else
    {
        QString target = table_list.join(",");
        auto message = std_msgs::msg::String();
        message.data = target.toStdString();
        table_publisher->publish(message);
        table_list.clear();
    }
}

void MainWindow::confirm_callback(const std_msgs::msg::String::SharedPtr msg)
{
    QString data = QString::fromStdString(msg->data);
    ui->label_6->setText(data);
}

void MainWindow::move_finish()
{
    auto msg = std_msgs::msg::String();
    msg.data = "go_to_next_point";
    finish_publisher->publish(msg);
}

void MainWindow::start_slam()
{
    QProcess::execute("pkill", QStringList() << "-f" << "slam_toolbox");        // tat cac tien trinh cu
    slam_process = new QProcess(this);

    QStringList slam_args;
    slam_args << "setsid" << "ros2" << "launch"
              << "articubot_one"
              << "online_async_launch.py"
              << "use_sim_time:=True";

    // slam_process->setArguments(slam_args);
    slam_process->start("setsid", slam_args);

    // Config display map
    auto map_display = _manager->createDisplay("rviz_default_plugins/Map", "Map Display", true);
    if (map_display)
    {
        map_display->subProp("Topic")->setValue("/map");
        qDebug() << "RViz show map.";
        QApplication::processEvents();
    }
    else
    {
        qDebug() << "Can't create map.";
    }
}


void MainWindow::quit_slam()
{
    // Tạo tiến trình để lấy PID từ lệnh ps + grep + awk
    QProcess find_pid;
    // awk '{print $1}': in ra cot dau tien (process id cua slamtoolbox)
    find_pid.start("bash", QStringList() << "-c" << "ps -eo pid,command | grep slam_toolbox | grep -v grep | awk '{print $1}'");
    find_pid.waitForFinished();
    
    QString pid_str = QString(find_pid.readAllStandardOutput()).trimmed();

    if (!pid_str.isEmpty())
    {
        qDebug() << "Found PID:" << pid_str;
        QProcess::execute("kill", QStringList() << "-SIGINT" << pid_str);

        QProcess::execute("pkill", QStringList() << "-f" << "slam_toolbox"); 
    }
    else
    {
        qDebug() << "slam_toolbox not running or PID not found.";
    }
}

void MainWindow::save_map()
{
    save_map_process = new QProcess(this);
    QString map_name = ui->lineEdit->text().trimmed();                  // trimmed: loai bo khoang trang o dau va cuoi chuoi

    QDir().mkpath(QDir::homePath() + "/maps");
    QString full_path = QDir::homePath() + "/maps/" + map_name;

    QStringList save_map_args;
    save_map_args << "run"
                  << "nav2_map_server"
                  << "map_saver_cli"
                  << "-f"
                  << full_path;

    save_map_process->start("ros2", save_map_args);
}


void MainWindow::Control_Robot_Manual()
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
            SendCommand_Vel();
        });

        connect(ui->pushButton_4, &QPushButton::pressed, this, [this]() {                     // lui       
            currentTwist.linear.x = -(ui->doubleSpinBox->value()); 
            currentTwist.angular.z = 0; 
            SendCommand_Vel();
        });

        connect(ui->pushButton_3, &QPushButton::pressed, this, [this]() {                     // phai         
            currentTwist.linear.x = 0; 
            currentTwist.angular.z = -(ui->doubleSpinBox_2->value()); 
            SendCommand_Vel();
        });

        connect(ui->pushButton_2, &QPushButton::pressed, this, [this]() {                     // trai     
            currentTwist.linear.x = 0; 
            currentTwist.angular.z = ui->doubleSpinBox_2->value(); 
            SendCommand_Vel();
        });

        connect(ui->pushButton, &QPushButton::released, this, [this]() {                      // nha                 
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand_Vel();
        });

        connect(ui->pushButton_2, &QPushButton::released, this, [this]() {                                    
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand_Vel();
        });

        connect(ui->pushButton_3, &QPushButton::released, this, [this]() {                                
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand_Vel();
        });

        connect(ui->pushButton_4, &QPushButton::released, this, [this]() {                                     
            currentTwist.linear.x = 0.0; 
            currentTwist.angular.z = 0.0; 
            SendCommand_Vel();
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
    vel_publisher->publish(currentTwist);
}

// void MainWindow::delete_map()
// {
//     pass
// }

// void MainWindow::quit_slam()
// {
//     if (slam_process && slam_process->state() != QProcess::NotRunning)
//     {
//         qint64 pid = slam_process->processId();
//         QString pidStr = QString::number(pid);

//         // Gửi tín hiệu SIGINT để tắt sạch sẽ node
//         QProcess::execute("kill", QStringList() << "-SIGINT" << pidStr);

//         // Đợi node shutdown
//         slam_process->waitForFinished(3000);

//         delete slam_process;
//         slam_process = nullptr;
//     }
// }
