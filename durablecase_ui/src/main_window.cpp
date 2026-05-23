#include "durablecase_ui/main_window.hpp"
#include "durablecase_ui/map_visualization.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QGroupBox>
#include <QFormLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QHeaderView>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

using namespace std::chrono_literals;

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> ros_node, QWidget *parent)
    : QMainWindow(parent), ros_node_(ros_node) {
  setWindowTitle("DurableCase - Multi-Agent Control UI");
  setWindowIcon(QIcon(":/icons/app_icon.png"));
  setGeometry(100, 100, 1600, 900);

  // Initialize ROS2 connections
  setupRosConnections();

  // Setup UI
  setupUI();

  // Setup update timer
  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &MainWindow::onUpdateSystemStatus);
  update_timer_->start(500);  // Update every 500ms

  // Initialize robot state
  robot_state_.robot_id = "robot_1";
  robot_state_.x_pos = 0.0;
  robot_state_.y_pos = 0.0;
  robot_state_.angle = 0.0;
  robot_state_.system_health = 0.95;
  robot_state_.active_agents = 4;

  RCLCPP_INFO(ros_node_->get_logger(), "✓ DurableCase UI initialized");
}

MainWindow::~MainWindow() = default;

void MainWindow::setupRosConnections() {
  // System state subscriber
  system_state_sub_ = ros_node_->create_subscription<
      supervisor_msgs::msg::SystemState>(
      "supervisor/system_state", 10,
      [this](const supervisor_msgs::msg::SystemState::SharedPtr msg) {
        robot_state_.system_health = msg->system_health;
        robot_state_.active_agents = msg->num_active_agents;
        robot_state_.failed_agents = std::vector<std::string>(
            msg->failed_agents.begin(), msg->failed_agents.end());
        updateSystemHealth();
      });

  // Task allocation client
  task_client_ =
      ros_node_->create_client<supervisor_msgs::srv::AllocateTask>(
          "supervisor/allocate_task");

  // Command velocity publisher
  cmd_vel_pub_ =
      ros_node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void MainWindow::setupUI() {
  QWidget *central_widget = new QWidget(this);
  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);

  // Create main splitter
  QSplitter *splitter = new QSplitter(Qt::Horizontal);

  // Left panel - Controls
  QWidget *left_panel = new QWidget();
  createLeftPanel(left_panel);
  splitter->addWidget(left_panel);
  splitter->setStretchFactor(0, 1);

  // Right panel - Visualization
  QWidget *right_panel = new QWidget();
  createRightPanel(right_panel);
  splitter->addWidget(right_panel);
  splitter->setStretchFactor(1, 2);

  main_layout->addWidget(splitter);
  setCentralWidget(central_widget);

  setupConnections();
}

void MainWindow::createLeftPanel(QWidget *container) {
  QVBoxLayout *layout = new QVBoxLayout(container);

  // --- Robot Info Section ---
  QGroupBox *info_group = new QGroupBox("Robot Info", this);
  QFormLayout *info_layout = new QFormLayout(info_group);

  robot_id_edit_ = new QLineEdit(this);
  robot_id_edit_->setText("robot_1");
  robot_id_edit_->setReadOnly(true);
  info_layout->addRow("Robot ID:", robot_id_edit_);

  current_position_label_ = new QLabel("X: 0.0, Y: 0.0, θ: 0.0", this);
  info_layout->addRow("Current Position:", current_position_label_);

  current_route_id_label_ = new QLabel("14272", this);
  info_layout->addRow("Current Route ID:", current_route_id_label_);

  current_status_label_ = new QLabel("Idle", this);
  info_layout->addRow("Current Status:", current_status_label_);

  status_progress_ = new QProgressBar(this);
  status_progress_->setValue(50);
  info_layout->addRow("Progress:", status_progress_);

  start_time_label_ = new QLabel("1970-01-03 03:48:20", this);
  info_layout->addRow("Start Time:", start_time_label_);

  velocity_label_ = new QLabel("v_x: 0.0, v_y: 0.0, v_θ: 0.0 (m/s)", this);
  info_layout->addRow("Velocity:", velocity_label_);

  layout->addWidget(info_group);

  // --- Motion Control Section ---
  QGroupBox *motion_group = new QGroupBox("Motion Control", this);
  QVBoxLayout *motion_layout = new QVBoxLayout(motion_group);

  // Arrow button layout
  QHBoxLayout *arrow_top = new QHBoxLayout();
  forward_btn_ = new QPushButton("↑ Forward", this);
  forward_btn_->setMinimumWidth(80);
  arrow_top->addStretch();
  arrow_top->addWidget(forward_btn_);
  arrow_top->addStretch();
  motion_layout->addLayout(arrow_top);

  QHBoxLayout *arrow_mid = new QHBoxLayout();
  left_btn_ = new QPushButton("← Left", this);
  stop_btn_ = new QPushButton("STOP", this);
  stop_btn_->setStyleSheet("background-color: red; color: white; font-weight: bold;");
  right_btn_ = new QPushButton("Right →", this);
  arrow_mid->addWidget(left_btn_);
  arrow_mid->addWidget(stop_btn_);
  arrow_mid->addWidget(right_btn_);
  motion_layout->addLayout(arrow_mid);

  QHBoxLayout *arrow_bot = new QHBoxLayout();
  backward_btn_ = new QPushButton("↓ Backward", this);
  backward_btn_->setMinimumWidth(80);
  arrow_bot->addStretch();
  arrow_bot->addWidget(backward_btn_);
  arrow_bot->addStretch();
  motion_layout->addLayout(arrow_bot);

  layout->addWidget(motion_group);

  // --- Mission Control Section ---
  QGroupBox *mission_group = new QGroupBox("Mission Control", this);
  QVBoxLayout *mission_layout = new QVBoxLayout(mission_group);

  QHBoxLayout *path_layout = new QHBoxLayout();
  set_path_btn_ = new QPushButton("Set path", this);
  stop_path_btn_ = new QPushButton("Stop path", this);
  path_layout->addWidget(set_path_btn_);
  path_layout->addWidget(stop_path_btn_);
  mission_layout->addLayout(path_layout);

  QHBoxLayout *pause_layout = new QHBoxLayout();
  pause_btn_ = new QPushButton("Pause", this);
  resume_btn_ = new QPushButton("Resume", this);
  pause_layout->addWidget(pause_btn_);
  pause_layout->addWidget(resume_btn_);
  mission_layout->addLayout(pause_layout);

  layout->addWidget(mission_group);

  // --- Task Management Section ---
  QGroupBox *task_group = new QGroupBox("Task Management", this);
  QVBoxLayout *task_layout = new QVBoxLayout(task_group);

  QFormLayout *task_form = new QFormLayout();
  task_uid_edit_ = new QLineEdit(this);
  task_uid_edit_->setPlaceholderText("Enter task UUID...");
  task_form->addRow("Task UID:", task_uid_edit_);
  task_layout->addLayout(task_form);

  allocate_task_btn_ = new QPushButton("Allocate Task", this);
  task_layout->addWidget(allocate_task_btn_);

  task_list_table_ = new QTableWidget(this);
  task_list_table_->setColumnCount(3);
  task_list_table_->setHorizontalHeaderLabels({"Task ID", "Status", "Agent"});
  task_list_table_->horizontalHeader()->setStretchLastSection(true);
  task_list_table_->setMaximumHeight(150);
  task_layout->addWidget(task_list_table_);

  layout->addWidget(task_group);

  // Add stretch at bottom
  layout->addStretch();
}

void MainWindow::createRightPanel(QWidget *container) {
  QVBoxLayout *layout = new QVBoxLayout(container);

  // Create map visualization
  map_widget_ = new MapVisualization(this);
  layout->addWidget(map_widget_, 1);

  // Bottom info bar
  QHBoxLayout *info_bar = new QHBoxLayout();
  QLabel *system_health_label = new QLabel("System Health: ", this);
  QProgressBar *system_health_bar = new QProgressBar(this);
  system_health_bar->setValue(95);
  info_bar->addWidget(system_health_label);
  info_bar->addWidget(system_health_bar, 1);
  layout->addLayout(info_bar);
}

void MainWindow::setupConnections() {
  // Motion buttons
  connect(forward_btn_, &QPushButton::clicked, this, &MainWindow::onForwardButtonClicked);
  connect(backward_btn_, &QPushButton::clicked, this, &MainWindow::onBackwardButtonClicked);
  connect(left_btn_, &QPushButton::clicked, this, &MainWindow::onLeftButtonClicked);
  connect(right_btn_, &QPushButton::clicked, this, &MainWindow::onRightButtonClicked);
  connect(stop_btn_, &QPushButton::clicked, this, &MainWindow::onStopButtonClicked);

  // Mission buttons
  connect(set_path_btn_, &QPushButton::clicked, this, &MainWindow::onSetPathClicked);
  connect(stop_path_btn_, &QPushButton::clicked, this, &MainWindow::onStopPathClicked);

  // Task management
  connect(allocate_task_btn_, &QPushButton::clicked, this, &MainWindow::onAllocateTaskClicked);

  // Pause/Resume
  connect(pause_btn_, &QPushButton::clicked, this, &MainWindow::onPauseClicked);
  connect(resume_btn_, &QPushButton::clicked, this, &MainWindow::onResumeClicked);
}

void MainWindow::onForwardButtonClicked() {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = 0.5;
  cmd_vel_pub_->publish(msg);
  RCLCPP_INFO(ros_node_->get_logger(), "Moving forward");
}

void MainWindow::onBackwardButtonClicked() {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = -0.5;
  cmd_vel_pub_->publish(msg);
  RCLCPP_INFO(ros_node_->get_logger(), "Moving backward");
}

void MainWindow::onLeftButtonClicked() {
  auto msg = geometry_msgs::msg::Twist();
  msg.angular.z = 0.5;
  cmd_vel_pub_->publish(msg);
  RCLCPP_INFO(ros_node_->get_logger(), "Turning left");
}

void MainWindow::onRightButtonClicked() {
  auto msg = geometry_msgs::msg::Twist();
  msg.angular.z = -0.5;
  cmd_vel_pub_->publish(msg);
  RCLCPP_INFO(ros_node_->get_logger(), "Turning right");
}

void MainWindow::onStopButtonClicked() {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = 0.0;
  msg.angular.z = 0.0;
  cmd_vel_pub_->publish(msg);
  RCLCPP_INFO(ros_node_->get_logger(), "STOP");
}

void MainWindow::onSetPathClicked() {
  RCLCPP_INFO(ros_node_->get_logger(), "Set path clicked");
}

void MainWindow::onStopPathClicked() {
  RCLCPP_INFO(ros_node_->get_logger(), "Stop path clicked");
}

void MainWindow::onAllocateTaskClicked() {
  QString task_id = task_uid_edit_->text();
  if (task_id.isEmpty()) {
    RCLCPP_WARN(ros_node_->get_logger(), "Task ID is empty");
    return;
  }

  auto request = std::make_shared<supervisor_msgs::srv::AllocateTask::Request>();
  request->task_id = task_id.toStdString();
  request->agent_id = robot_state_.robot_id.toStdString();
  request->task_type = "navigation";
  request->priority = "high";

  RCLCPP_INFO(ros_node_->get_logger(), "Allocating task: %s", task_id.toStdString().c_str());
}

void MainWindow::onPauseClicked() {
  onStopButtonClicked();
  RCLCPP_INFO(ros_node_->get_logger(), "Mission paused");
}

void MainWindow::onResumeClicked() {
  RCLCPP_INFO(ros_node_->get_logger(), "Mission resumed");
}

void MainWindow::onUpdateSystemStatus() {
  updateRobotInfo();
  updateSystemHealth();

  // Spin spin ROS
  rclcpp::spin_some(ros_node_);
}

void MainWindow::updateRobotInfo() {
  QString pos_str = QString("X: %1, Y: %2, θ: %3")
                        .arg(robot_state_.x_pos, 0, 'f', 2)
                        .arg(robot_state_.y_pos, 0, 'f', 2)
                        .arg(robot_state_.angle, 0, 'f', 2);
  current_position_label_->setText(pos_str);

  // Update map visualization
  map_widget_->setRobotPose(robot_state_.x_pos, robot_state_.y_pos,
                            robot_state_.angle);
}

void MainWindow::updateSystemHealth() {
  int health_percent = static_cast<int>(robot_state_.system_health * 100);
  status_progress_->setValue(health_percent);

  if (robot_state_.system_health < 0.3) {
    status_progress_->setStyleSheet("QProgressBar::chunk { background-color: red; }");
  } else if (robot_state_.system_health < 0.6) {
    status_progress_->setStyleSheet("QProgressBar::chunk { background-color: orange; }");
  } else {
    status_progress_->setStyleSheet("QProgressBar::chunk { background-color: green; }");
  }
}

void MainWindow::onUpdateAgentStatus() {
  // Placeholder for agent status updates
  // This will be called periodically to update agent displays
}
