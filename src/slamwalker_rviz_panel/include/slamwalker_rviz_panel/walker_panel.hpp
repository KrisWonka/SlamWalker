#pragma once

#include <memory>
#include <string>

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace slamwalker_rviz_panel
{

class WalkerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WalkerPanel(QWidget * parent = nullptr);
  ~WalkerPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onStartFrontier();
  void onFinishToPhase2();
  void onLoadMap();
  void onResetPose();
  void onBrowseMap();
  void onEmergencyStop();

private:
  void callTrigger(const std::string & service_name, const std::string & friendly);
  void setStatus(const QString & text, bool ok);

  rclcpp::Node::SharedPtr node_;
  rclcpp::AsyncParametersClient::SharedPtr session_param_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_start_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_finish_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_load_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_reset_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_stop_;

  QPushButton * btn_start_;
  QPushButton * btn_finish_;
  QPushButton * btn_load_;
  QPushButton * btn_reset_;
  QPushButton * btn_browse_;
  QPushButton * btn_estop_;
  QLineEdit * map_path_edit_;
  QLabel * status_;
};

}  // namespace slamwalker_rviz_panel
