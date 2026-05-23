#ifndef DURABLECASE_UI_MAP_VISUALIZATION_HPP_
#define DURABLECASE_UI_MAP_VISUALIZATION_HPP_

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <vector>
#include <memory>
#include <cmath>

/**
 * OpenGL-based map visualization widget
 * Displays field/map with obstacles, paths, and robot poses
 */
class MapVisualization : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

 public:
  MapVisualization(QWidget *parent = nullptr);
  ~MapVisualization();

  void setRobotPose(double x, double y, double angle);
  void setGoalPose(double x, double y);
  void setObstacles(const std::vector<std::pair<double, double>> &obstacles);
  void setPath(const std::vector<std::pair<double, double>> &path);
  void clearPath();

 protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

 private:
  void drawGrid();
  void drawRobot();
  void drawObstacles();
  void drawPath();
  void drawGoal();

  // Camera control
  float camera_x_;
  float camera_y_;
  float camera_z_;
  float zoom_;
  float pan_x_;
  float pan_y_;

  // Robot state
  double robot_x_;
  double robot_y_;
  double robot_angle_;
  double goal_x_;
  double goal_y_;
  bool has_goal_;

  // Map elements
  std::vector<std::pair<double, double>> obstacles_;
  std::vector<std::pair<double, double>> path_;

  // OpenGL program ID
  GLuint shader_program_;
};

#endif  // DURABLECASE_UI_MAP_VISUALIZATION_HPP_
