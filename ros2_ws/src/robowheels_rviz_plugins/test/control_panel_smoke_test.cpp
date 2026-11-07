// Opt-in integration check against the running mock controllers. Widgets are
// instantiated on the container's hidden Xvfb, not the user's XQuartz display.
#include "robowheels_rviz_plugins/control_panel.hpp"
#include <QApplication>
#include <QTest>
#include <QElapsedTimer>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <cmath>

int main(int argc,char **argv){
  rclcpp::init(argc,argv);QApplication app(argc,argv);
  auto observer=std::make_shared<rclcpp::Node>("native_panel_test_observer");
  double elbow=0,x=0,y=0,linear=0,angular=0;int joint_messages=0,odom_messages=0;
  auto states=observer->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,[&](sensor_msgs::msg::JointState::ConstSharedPtr msg){
    for(size_t i=0;i<msg->name.size();++i)if(msg->name[i]=="right_elbow_pitch"&&i<msg->position.size())elbow=msg->position[i];++joint_messages;
  });
  auto odom=observer->create_subscription<nav_msgs::msg::Odometry>("/odom",10,[&](nav_msgs::msg::Odometry::ConstSharedPtr msg){x=msg->pose.pose.position.x;y=msg->pose.pose.position.y;linear=msg->twist.twist.linear.x;angular=msg->twist.twist.angular.z;++odom_messages;});
  int code=0;
  try{
    robowheels_rviz_plugins::ControlPanel panel;panel.resize(360,850);panel.show();
    auto pump=[&](int ms){QElapsedTimer timer;timer.start();while(timer.elapsed()<ms){QApplication::processEvents();rclcpp::spin_some(observer);QTest::qWait(10);}};
    auto wait=[&](std::function<bool()> predicate,int timeout,const char *stage){std::cerr<<"Checking "<<stage<<std::endl;QElapsedTimer timer;timer.start();while(!predicate()&&timer.elapsed()<timeout)pump(20);if(!predicate()){auto label=panel.findChild<QLabel*>("trajectory_status");throw std::runtime_error(std::string("Timed out: ")+stage+"; status="+(label?label->text().toStdString():"missing")+"; joint messages="+std::to_string(joint_messages)+"; odom messages="+std::to_string(odom_messages));}};
    auto button=[&](const QString &text){for(auto b:panel.findChildren<QPushButton*>())if(b->text()==text)return b;throw std::runtime_error("Button missing: "+text.toStdString());};
    auto status=panel.findChild<QLabel*>("trajectory_status");auto target=panel.findChild<QDoubleSpinBox*>("right_elbow_pitch");
    wait([&]{return joint_messages>0&&odom_messages>0&&button("Execute targets")->isEnabled();},10000,"controller readiness");
    target->setValue(.14);button("Execute targets")->click();
    wait([&]{return status->text()=="Trajectory succeeded";},7000,"execute target");
    if(std::abs(elbow-.14)>.002)throw std::runtime_error("Native target did not reach joint feedback");
    button("Demo trajectory")->click();pump(700);button("Cancel / hold")->click();
    wait([&]{return status->text()=="Canceled; holding position";},3000,"cancel demo");
    double held=elbow;pump(400);if(std::abs(elbow-held)>.002)throw std::runtime_error("Canceled arm did not hold");
    button("Home (CAD zero)")->click();wait([&]{return status->text()=="Trajectory succeeded";},7000,"home");
    if(std::abs(elbow)>.002)throw std::runtime_error("Native home did not reach zero");
    double start_x=x,start_y=y;
    QMetaObject::invokeMethod(button("Forward"),"pressed",Qt::DirectConnection);pump(1300);
    QMetaObject::invokeMethod(button("Forward"),"released",Qt::DirectConnection);pump(700);
    double distance=std::hypot(x-start_x,y-start_y);
    if(distance<.08||std::abs(linear)>.005)throw std::runtime_error("Native base forward/release failed");
    QMetaObject::invokeMethod(button("Left"),"pressed",Qt::DirectConnection);pump(900);
    if(angular<.2)throw std::runtime_error("Native left turn failed");
    QEvent deactivate(QEvent::WindowDeactivate);QApplication::sendEvent(&panel,&deactivate);pump(700);
    if(std::abs(angular)>.005||std::abs(linear)>.005)throw std::runtime_error("Native focus-loss stop failed");
    std::cout<<"{\"passed\":true,\"native_execute_feedback_rad\":0.14,\"native_demo_cancel\":true,\"native_home\":true,\"native_forward_distance_m\":"<<distance<<",\"native_release_stop\":true,\"native_turn\":true,\"native_focus_loss_stop\":true}"<<std::endl;
  }catch(const std::exception &error){std::cerr<<error.what()<<std::endl;code=1;}
  rclcpp::shutdown();return code;
}
