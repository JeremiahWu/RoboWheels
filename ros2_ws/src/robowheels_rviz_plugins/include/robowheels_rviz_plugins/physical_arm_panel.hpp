#pragma once
#include <rviz_common/panel.hpp>
#include <QNetworkAccessManager>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <array>
namespace robowheels_rviz_plugins {
class PhysicalArmPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit PhysicalArmPanel(QWidget *parent=nullptr);
private:
  void poll();
  void command(const QByteArray &payload);
  QNetworkAccessManager network_;
  QTimer timer_;
  QLabel *status_;
  QPushButton *arm_, *stop_;
  std::array<QLabel*,8> values_{};
  std::array<QPushButton*,8> minus_{}, plus_{};
  bool polling_=false, posting_=false;
};
}
