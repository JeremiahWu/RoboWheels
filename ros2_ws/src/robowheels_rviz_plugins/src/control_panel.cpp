#include "robowheels_rviz_plugins/control_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QApplication>
#include <QEvent>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QJsonDocument>
#include <QJsonArray>

namespace robowheels_rviz_plugins {
ControlPanel::ControlPanel(QWidget *parent):rviz_common::Panel(parent) {
  auto wrapper=new QVBoxLayout(this);auto scroll=new QScrollArea();scroll->setWidgetResizable(true);
  auto content=new QWidget();auto layout=new QVBoxLayout(content);scroll->setWidget(content);wrapper->addWidget(scroll);setMinimumWidth(310);
  layout->addWidget(new QLabel("RoboWheels | Virtual hardware"));
  auto base=new QGroupBox("Differential drive (hold to move)");
  auto base_layout=new QVBoxLayout(base);
  base_status_=new QLabel("Waiting for odometry...");base_status_->setWordWrap(true);base_layout->addWidget(base_status_);
  auto speed_row=new QFormLayout();speed_=new QDoubleSpinBox();speed_->setRange(.05,.25);speed_->setSingleStep(.01);speed_->setValue(.12);speed_->setSuffix(" m/s");
  speed_row->addRow("Speed",speed_);base_layout->addLayout(speed_row);
  auto pad=new QGridLayout();
  auto button=[this,pad](const QString &label,int row,int col,double linear,double angular) {
    auto b=new QPushButton(label);b->setMinimumHeight(32);pad->addWidget(b,row,col);
    connect(b,&QPushButton::pressed,this,[this,linear,angular]{linear_=linear*speed_->value();angular_=angular;});
    connect(b,&QPushButton::released,this,[this]{stopBase();});return b;
  };
  button("Forward",0,1,1,0);button("Left",1,0,0,.6);button("Right",1,2,0,-.6);button("Reverse",2,1,-1,0);
  auto stop=new QPushButton("STOP");pad->addWidget(stop,1,1);connect(stop,&QPushButton::clicked,this,[this]{stopBase();});
  base_layout->addLayout(pad);layout->addWidget(base);
  auto arms=new QGroupBox("Dual-arm trajectory (rad)");auto arm_layout=new QVBoxLayout(arms);
  auto grid=new QGridLayout();grid->addWidget(new QLabel("Joint / target"),0,0);grid->addWidget(new QLabel("Feedback"),0,2);
  const std::array<std::string,5> suffix={"shoulder_roll","shoulder_pitch","elbow_pitch","wrist_pitch","wrist_roll"};
  const std::array<QString,5> labels={"Shoulder roll","Shoulder pitch","Elbow pitch","Wrist pitch","Wrist roll"};
  for(int side=0;side<2;++side)for(int j=0;j<5;++j){
    const int i=side*5+j;names_.push_back(std::string(side==0?"right_":"left_")+suffix[j]);
    grid->addWidget(new QLabel((side==0?"R ":"L ")+labels[j]),i+1,0);
    target_[i]=new QDoubleSpinBox();target_[i]->setRange(-.65,.65);target_[i]->setDecimals(2);target_[i]->setSingleStep(.01);target_[i]->setObjectName(QString::fromStdString(names_[i]));grid->addWidget(target_[i],i+1,1);
    feedback_[i]=new QLabel("0.000");grid->addWidget(feedback_[i],i+1,2);
  }
  arm_layout->addLayout(grid);
  auto duration_row=new QFormLayout();duration_=new QDoubleSpinBox();duration_->setRange(4,15);duration_->setValue(4);duration_->setSuffix(" s");duration_row->addRow("Duration",duration_);arm_layout->addLayout(duration_row);
  auto buttons=new QGridLayout();demo_=new QPushButton("Demo trajectory");home_=new QPushButton("Home (CAD zero)");execute_=new QPushButton("Execute targets");cancel_=new QPushButton("Cancel / hold");
  buttons->addWidget(demo_,0,0);buttons->addWidget(home_,0,1);buttons->addWidget(execute_,1,0);buttons->addWidget(cancel_,1,1);arm_layout->addLayout(buttons);
  connect(demo_,&QPushButton::clicked,this,[this]{sendTrajectory(true,false);});
  connect(home_,&QPushButton::clicked,this,[this]{sendTrajectory(false,true);});
  connect(execute_,&QPushButton::clicked,this,[this]{sendTrajectory(false,false);});
  connect(cancel_,&QPushButton::clicked,this,[this]{post("/api/cancel",QJsonObject{});});
  status_=new QLabel("Waiting for controllers...");status_->setObjectName("trajectory_status");status_->setWordWrap(true);arm_layout->addWidget(status_);layout->addWidget(arms);
  auto note=new QLabel("CAD zero and demo limits are provisional.\nKinematic mock hardware; no physical robot connection.");note->setWordWrap(true);layout->addWidget(note);layout->addStretch();
  node_=std::make_shared<rclcpp::Node>("robowheels_rviz_control",rclcpp::NodeOptions().use_global_arguments(false));executor_.add_node(node_);
  drive_=node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
  joints_=node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,[this](sensor_msgs::msg::JointState::ConstSharedPtr msg){
    int found=0;
    for(size_t i=0;i<names_.size();++i){auto it=std::find(msg->name.begin(),msg->name.end(),names_[i]);if(it!=msg->name.end()){
      auto index=std::distance(msg->name.begin(),it);if(static_cast<size_t>(index)<msg->position.size()){actual_[i]=msg->position[index];feedback_[i]->setText(QString::number(actual_[i],'f',3));++found;}}}
    feedback_ready_=found==10;
  });
  odom_=node_->create_subscription<nav_msgs::msg::Odometry>("/odom",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
    const auto &p=msg->pose.pose.position;const auto &q=msg->pose.pose.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    base_status_->setText(QString("x %1 m  y %2 m  yaw %3 deg\nv %4 m/s  w %5 rad/s").arg(p.x,0,'f',3).arg(p.y,0,'f',3).arg(yaw*180/3.141592653589793,0,'f',1).arg(msg->twist.twist.linear.x,0,'f',2).arg(msg->twist.twist.angular.z,0,'f',2));
  });
  connect(&timer_,&QTimer::timeout,this,[this]{tick();});timer_.start(50);setBusy(false);qApp->installEventFilter(this);
}
ControlPanel::~ControlPanel(){qApp->removeEventFilter(this);timer_.stop();stopBase();executor_.remove_node(node_);}
bool ControlPanel::eventFilter(QObject *object,QEvent *event){
  if(event->type()==QEvent::ApplicationDeactivate||event->type()==QEvent::WindowDeactivate||event->type()==QEvent::Close)stopBase();
  return rviz_common::Panel::eventFilter(object,event);
}
void ControlPanel::stopBase(){linear_=angular_=0;if(drive_)drive_->publish(geometry_msgs::msg::Twist());}
void ControlPanel::tick(){
  executor_.spin_some();
  if(linear_!=0||angular_!=0){geometry_msgs::msg::Twist msg;msg.linear.x=linear_;msg.angular.z=angular_;drive_->publish(msg);}
  if(++poll_ticks_%4==0)pollTrajectory();
  const bool ready=feedback_ready_&&controller_ready_;
  demo_->setEnabled(ready&&!busy_&&!posting_);home_->setEnabled(ready&&!busy_&&!posting_);execute_->setEnabled(ready&&!busy_&&!posting_);
}
void ControlPanel::pollTrajectory(){
  if(polling_)return;polling_=true;
  auto reply=network_.get(QNetworkRequest(QUrl("http://127.0.0.1:8081/api/state")));
  connect(reply,&QNetworkReply::finished,this,[this,reply]{
    polling_=false;
    if(reply->error()!=QNetworkReply::NoError){controller_ready_=false;status_->setText("Waiting for local trajectory bridge...");reply->deleteLater();return;}
    const auto root=QJsonDocument::fromJson(reply->readAll()).object();reply->deleteLater();
    controller_ready_=root.value("ready").toBool();if(posting_)return;
    const auto execution=root.value("execution").toObject();const auto phase=execution.value("status").toString();
    setBusy(phase=="queued"||phase=="running"||phase=="canceling");
    if(phase=="succeeded")status_->setText("Trajectory succeeded");
    else if(phase=="canceled")status_->setText("Canceled; holding position");
    else if(phase=="running"){
      double error=0;for(const auto value:execution.value("error").toArray())error=std::max(error,std::abs(value.toDouble()));
      status_->setText(QString("Running | max error %1 rad").arg(error,0,'f',5));
    }else if(phase=="canceling")status_->setText("Cancel requested; holding position");
    else if(phase=="queued")status_->setText("Submitting trajectory...");
    else if(phase=="idle")status_->setText(controller_ready_?"Controllers ready | CAD assembly zero":"Waiting for controllers...");
    else status_->setText("Trajectory failed: "+execution.value("message").toString());
  });
}
void ControlPanel::post(const QString &path,const QJsonObject &data){
  if(posting_)return;posting_=true;
  QNetworkRequest request(QUrl("http://127.0.0.1:8081"+path));request.setHeader(QNetworkRequest::ContentTypeHeader,"application/json");
  auto reply=network_.post(request,QJsonDocument(data).toJson(QJsonDocument::Compact));
  connect(reply,&QNetworkReply::finished,this,[this,reply]{
    posting_=false;const auto data=QJsonDocument::fromJson(reply->readAll()).object();
    if(reply->error()!=QNetworkReply::NoError){setBusy(false);status_->setText("Request failed: "+data.value("error").toString(reply->errorString()));}
    reply->deleteLater();pollTrajectory();
  });
}
void ControlPanel::setBusy(bool busy){busy_=busy;demo_->setEnabled(!busy);home_->setEnabled(!busy);execute_->setEnabled(!busy);cancel_->setEnabled(busy);}
void ControlPanel::sendTrajectory(bool demo,bool home){
  if(busy_||posting_||!controller_ready_)return;
  QJsonObject command;
  if(demo)command.insert("preset","demo");
  else{
    QJsonArray target;double distance=0;
    for(size_t i=0;i<10;++i){double value=home?0:target_[i]->value();target.append(value);distance=std::max(distance,std::abs(value-actual_[i]));}
    if(duration_->value()<distance*6){status_->setText(QString("Increase duration to at least %1 s").arg(distance*6,0,'f',1));return;}
    command.insert("positions",target);command.insert("duration",duration_->value());
  }
  setBusy(true);status_->setText("Submitting trajectory...");post("/api/trajectory",command);
}
}
PLUGINLIB_EXPORT_CLASS(robowheels_rviz_plugins::ControlPanel,rviz_common::Panel)
