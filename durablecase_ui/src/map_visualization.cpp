#include "durablecase_ui/map_visualization.hpp"
#include <QMouseEvent>
#include <QWheelEvent>

MapVisualization::MapVisualization(QWidget *parent)
    : QOpenGLWidget(parent),
      camera_x_(0.0f),
      camera_y_(0.0f),
      camera_z_(25.0f),
      zoom_(1.0f),
      pan_x_(0.0f),
      pan_y_(0.0f),
      robot_x_(0.0),
      robot_y_(0.0),
      robot_angle_(0.0),
      goal_x_(0.0),
      goal_y_(0.0),
      has_goal_(false),
      shader_program_(0) {}

MapVisualization::~MapVisualization() {
  makeCurrent();
  glDeleteProgram(shader_program_);
  doneCurrent();
}

void MapVisualization::initializeGL() {
  initializeOpenGLFunctions();
  glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void MapVisualization::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
}

void MapVisualization::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawGrid();
  drawObstacles();
  drawPath();
  drawGoal();
  drawRobot();
}

void MapVisualization::drawGrid() {
  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(-50, 50, -50, 50, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glColor4f(0.4f, 0.4f, 0.4f, 0.5f);
  glBegin(GL_LINES);
  for (int i = -50; i <= 50; i += 5) {
    glVertex2f(static_cast<float>(i), -50.0f);
    glVertex2f(static_cast<float>(i), 50.0f);
    glVertex2f(-50.0f, static_cast<float>(i));
    glVertex2f(50.0f, static_cast<float>(i));
  }
  glEnd();

  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glEnable(GL_DEPTH_TEST);
}

void MapVisualization::drawRobot() {
  glPushMatrix();
  glTranslatef(static_cast<float>(robot_x_), static_cast<float>(robot_y_), 0.0f);
  glRotatef(static_cast<float>(robot_angle_ * 180.0 / 3.14159265), 0.0f, 0.0f, 1.0f);

  // Robot body (rectangle)
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_QUADS);
  glVertex2f(-0.5f, -0.3f);
  glVertex2f(0.5f, -0.3f);
  glVertex2f(0.5f, 0.3f);
  glVertex2f(-0.5f, 0.3f);
  glEnd();

  // Robot direction indicator
  glColor3f(1.0f, 0.0f, 0.0f);
  glBegin(GL_TRIANGLES);
  glVertex2f(0.5f, 0.0f);
  glVertex2f(0.3f, -0.2f);
  glVertex2f(0.3f, 0.2f);
  glEnd();

  glPopMatrix();
}

void MapVisualization::drawObstacles() {
  glColor3f(1.0f, 0.0f, 1.0f);  // Magenta

  for (const auto &obs : obstacles_) {
    glPushMatrix();
    glTranslatef(static_cast<float>(obs.first), static_cast<float>(obs.second), 0.0f);

    glBegin(GL_QUADS);
    glVertex2f(-2.0f, -2.0f);
    glVertex2f(2.0f, -2.0f);
    glVertex2f(2.0f, 2.0f);
    glVertex2f(-2.0f, 2.0f);
    glEnd();

    glPopMatrix();
  }
}

void MapVisualization::drawPath() {
  if (path_.empty()) return;

  glColor3f(0.0f, 1.0f, 1.0f);  // Cyan
  glBegin(GL_LINE_STRIP);
  for (const auto &point : path_) {
    glVertex2f(static_cast<float>(point.first), static_cast<float>(point.second));
  }
  glEnd();
}

void MapVisualization::drawGoal() {
  if (!has_goal_) return;

  glColor3f(1.0f, 1.0f, 0.0f);  // Yellow
  glPushMatrix();
  glTranslatef(static_cast<float>(goal_x_), static_cast<float>(goal_y_), 0.0f);

  glBegin(GL_TRIANGLES);
  glVertex2f(0.0f, 0.5f);
  glVertex2f(-0.5f, -0.5f);
  glVertex2f(0.5f, -0.5f);
  glEnd();

  glPopMatrix();
}

void MapVisualization::setRobotPose(double x, double y, double angle) {
  robot_x_ = x;
  robot_y_ = y;
  robot_angle_ = angle;
  update();
}

void MapVisualization::setGoalPose(double x, double y) {
  goal_x_ = x;
  goal_y_ = y;
  has_goal_ = true;
  update();
}

void MapVisualization::setObstacles(
    const std::vector<std::pair<double, double>> &obstacles) {
  obstacles_ = obstacles;
  update();
}

void MapVisualization::setPath(
    const std::vector<std::pair<double, double>> &path) {
  path_ = path;
  update();
}

void MapVisualization::clearPath() {
  path_.clear();
  has_goal_ = false;
  update();
}

void MapVisualization::mousePressEvent(QMouseEvent *event) {
  QOpenGLWidget::mousePressEvent(event);
  update();
}

void MapVisualization::mouseMoveEvent(QMouseEvent *event) {
  QOpenGLWidget::mouseMoveEvent(event);
  update();
}

void MapVisualization::wheelEvent(QWheelEvent *event) {
  zoom_ += event->angleDelta().y() / 120.0f * 0.1f;
  zoom_ = std::max(0.1f, zoom_);
  update();
  event->accept();
}
