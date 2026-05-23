#ifndef DURABLECASE_UI_MAIN_WINDOW_HPP_
#define DURABLECASE_UI_MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QComboBox>
#include <QTableWidget>
#include <QProgressBar>
#include <QTimer>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "supervisor_msgs/msg/agent_status.hpp"
#include "supervisor_msgs/msg/system_state.hpp"
#include "supervisor_msgs/srv/allocate_task.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MapVisualization;
class ControlPanel;
class StatusPanel;

/**
 * Main window for DurableCase UI
 * Integrates ROS2 supervisory control layer with Qt-based dashboard
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(std::shared_ptr<rclcpp::Node> ros_node, QWidget *parent = nullptr);
  ~MainWindow();

 private slots:
  void onForwardButtonClicked();
  void onBackwardButtonClicked();
  void onLeftButtonClicked();
  void onRightButtonClicked();
  void onStopButtonClicked();
  void onSetPathClicked();
  void onStopPathClicked();
  void onAllocateTaskClicked();
  void onPauseClicked();
  void onResumeClicked();
  void onUpdateSystemStatus();
  void onUpdateAgentStatus();

 private:
  void setupUI();
  void setupConnections();
  void setupRosConnections();
  void createLeftPanel(QWidget *container);
  void createRightPanel(QWidget *container);
  void updateRobotInfo();
  void updateSystemHealth();

  // ROS2 Members
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<supervisor_msgs::msg::SystemState>::SharedPtr
      system_state_sub_;
  rclcpp::Subscription<supervisor_msgs::msg::AgentStatus>::SharedPtr
      agent_status_sub_;
  rclcpp::Client<supervisor_msgs::srv::AllocateTask>::SharedPtr task_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // UI Components - Left Panel
  QLineEdit *robot_id_edit_;
  QLabel *current_position_label_;
  QLabel *current_route_id_label_;
  QLabel *current_status_label_;
  QProgressBar *status_progress_;
  QPushButton *forward_btn_;
  QPushButton *backward_btn_;
  QPushButton *left_btn_;
  QPushButton *right_btn_;
  QPushButton *stop_btn_;
  QPushButton *set_path_btn_;
  QPushButton *stop_path_btn_;
  QPushButton *pause_btn_;
  QPushButton *resume_btn_;
  QLineEdit *task_uid_edit_;
  QPushButton *allocate_task_btn_;
  QLabel *start_time_label_;
  QLabel *velocity_label_;
  QTableWidget *task_list_table_;

  // UI Components - Right Panel
  MapVisualization *map_widget_;

  // System State
  struct RobotState {
    QString robot_id;
    double x_pos;
    double y_pos;
    double angle;
    std::string current_route_id;
    std::string status;
    double system_health;
    int active_agents;
    std::vector<std::string> failed_agents;
  } robot_state_;

  QTimer *update_timer_;
};

#endif  // DURABLECASE_UI_MAIN_WINDOW_HPP_
