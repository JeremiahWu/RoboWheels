#include "robowheels_rviz_plugins/physical_arm_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QNetworkReply>
#include <QNetworkRequest>
namespace robowheels_rviz_plugins {
PhysicalArmPanel::PhysicalArmPanel(QWidget *parent):rviz_common::Panel(parent) {
  auto layout=new QVBoxLayout(this);setMinimumWidth(360);
  auto title=new QLabel("Physical arms | measured feedback");layout->addWidget(title);
  auto note=new QLabel("One click = 20 raw units / 2.5 s. No auto home.\nAll joints use the same step limits. Angles provisional.\nModel follows measured feedback, not target commands.");note->setWordWrap(true);layout->addWidget(note);
  auto grid=new QGridLayout();
  const std::array<QString,4> names={"Shoulder pitch","Shoulder roll","Elbow","Wrist pitch"};
  for(int i=0;i<8;++i){
    grid->addWidget(new QLabel(QString("%1 %2 | ID%3").arg(i<4?"R":"L").arg(names[i%4]).arg(i+1)),i,0);
    minus_[i]=new QPushButton("-20");plus_[i]=new QPushButton("+20");values_[i]=new QLabel("Waiting...");
    minus_[i]->setEnabled(false);plus_[i]->setEnabled(false);
    grid->addWidget(minus_[i],i,1);grid->addWidget(plus_[i],i,2);grid->addWidget(values_[i],i,3);
    connect(minus_[i],&QPushButton::clicked,this,[this,i]{command(QString("{\"action\":\"jog\",\"id\":%1,\"delta_raw\":-20}").arg(i+1).toUtf8());});
    connect(plus_[i],&QPushButton::clicked,this,[this,i]{command(QString("{\"action\":\"jog\",\"id\":%1,\"delta_raw\":20}").arg(i+1).toUtf8());});
  }
  layout->addLayout(grid);
  arm_=new QPushButton("Enable physical outputs");stop_=new QPushButton("Stop / disarm (hold active joint)");
  layout->addWidget(arm_);layout->addWidget(stop_);
  connect(arm_,&QPushButton::clicked,this,[this]{command("{\"action\":\"arm\"}");});
  connect(stop_,&QPushButton::clicked,this,[this]{command("{\"action\":\"disarm\"}");});
  status_=new QLabel("Connecting...");status_->setWordWrap(true);layout->addWidget(status_);layout->addStretch();
  connect(&timer_,&QTimer::timeout,this,[this]{poll();});timer_.start(250);poll();
}
void PhysicalArmPanel::command(const QByteArray &payload){
  if(posting_)return;posting_=true;
  for(int i=0;i<8;++i){minus_[i]->setEnabled(false);plus_[i]->setEnabled(false);}
  QNetworkRequest req(QUrl("http://127.0.0.1:8082/command"));req.setHeader(QNetworkRequest::ContentTypeHeader,"application/json");
  req.setTransferTimeout(3500);
  auto reply=network_.post(req,payload);
  connect(reply,&QNetworkReply::finished,this,[this,reply]{
    posting_=false;
    const auto obj=QJsonDocument::fromJson(reply->readAll()).object();
    if(reply->error()!=QNetworkReply::NoError)status_->setText("Rejected: "+obj.value("error").toString(reply->errorString()));
    reply->deleteLater();poll();
  });
}
void PhysicalArmPanel::poll(){
  if(polling_)return;polling_=true;
  QNetworkRequest req(QUrl("http://127.0.0.1:8082/state"));req.setTransferTimeout(1500);
  auto reply=network_.get(req);
  connect(reply,&QNetworkReply::finished,this,[this,reply]{
    polling_=false;
    const auto obj=QJsonDocument::fromJson(reply->readAll()).object();
    const bool fresh=reply->error()==QNetworkReply::NoError&&obj.value("bridge_fresh").toBool();reply->deleteLater();
    const bool armed=obj.value("armed").toBool(),busy=obj.value("busy").toBool();
    for(int i=0;i<8;++i){minus_[i]->setEnabled(false);plus_[i]->setEnabled(false);}
    for(const auto value:obj.value("joints").toArray()){
      const auto j=value.toObject();const int i=j.value("id").toInt()-1;if(i<0||i>7)continue;
      const bool valid=fresh&&j.value("fresh").toBool();
      const QString fault=j.value("fault").toString();
      values_[i]->setText(valid?QString("raw %1 | delta %2%3").arg(j.value("raw").toInt()).arg(j.value("relative_raw").toInt()).arg(fault.isEmpty()?"":" | FAULT"):"STALE");
      const bool enabled=valid&&armed&&!busy&&!posting_&&j.value("enabled").toBool()&&fault.isEmpty();
      minus_[i]->setEnabled(enabled);plus_[i]->setEnabled(enabled);
      values_[i]->setToolTip(fault);
    }
    arm_->setEnabled(fresh&&!armed&&!posting_&&!busy);stop_->setEnabled(fresh&&!posting_);
    if(!fresh)status_->setText("Disconnected / stale feedback; movement disabled");
    else if(!posting_)status_->setText(busy?"Physical joint moving; wait or Stop":armed?"Outputs enabled | step controls active":"Outputs disarmed | feedback only");
  });
}
}
PLUGINLIB_EXPORT_CLASS(robowheels_rviz_plugins::PhysicalArmPanel,rviz_common::Panel)
