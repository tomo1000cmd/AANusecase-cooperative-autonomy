#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "durablecase_ui/main_window.hpp"

int main(int argc, char *argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("durablecase_ui");

  // Create Qt application
  QApplication app(argc, argv);

  // Create and show main window
  MainWindow window(node);
  window.show();

  // Run Qt event loop
  int result = app.exec();

  // Cleanup
  rclcpp::shutdown();
  return result;
}
