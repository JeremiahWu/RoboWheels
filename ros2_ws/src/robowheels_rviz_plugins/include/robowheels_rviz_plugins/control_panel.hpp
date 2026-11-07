#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QNetworkAccessManager>
#include <QJsonObject>
#include <array>
#include <vector>

namespace robowheels_rviz_plugins {
class ControlPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit ControlPanel(QWidget *parent=nullptr);
  ~ControlPanel() override;
protected:
  bool eventFilter(QObject *object, QEvent *event) override;
private:
  void tick();
  void pollTrajectory();
  void post(const QString &path, const QJsonObject &data);
  void stopBase();
  void sendTrajectory(bool demo, bool home);
  void setBusy(bool busy);
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  QNetworkAccessManager network_;
  QTimer timer_;
  QLabel *status_, *base_status_;
  QDoubleSpinBox *speed_, *duration_;
  QPushButton *demo_, *home_, *execute_, *cancel_;
  std::array<QDoubleSpinBox*,10> target_{};
  std::array<QLabel*,10> feedback_{};
  std::array<double,10> actual_{};
  std::vector<std::string> names_;
  double linear_=0, angular_=0;
  bool busy_=false, feedback_ready_=false, controller_ready_=false, polling_=false, posting_=false;
  int poll_ticks_=0;
};
}
