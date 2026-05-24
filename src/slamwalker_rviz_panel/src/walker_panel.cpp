#include "slamwalker_rviz_panel/walker_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QFont>

#include <rviz_common/display_context.hpp>
#include <rclcpp/parameter_client.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace slamwalker_rviz_panel
{

WalkerPanel::WalkerPanel(QWidget * parent)
: rviz_common::Panel(parent),
  btn_start_(nullptr), btn_finish_(nullptr), btn_load_(nullptr),
  btn_reset_(nullptr), btn_browse_(nullptr), btn_estop_(nullptr),
  map_path_edit_(nullptr), status_(nullptr)
{
  auto layout = new QVBoxLayout;

  auto title = new QLabel("<b>SlamWalker Control</b>");
  title->setAlignment(Qt::AlignCenter);
  layout->addWidget(title);

  btn_estop_ = new QPushButton("■  EMERGENCY STOP");
  btn_estop_->setStyleSheet(
    "QPushButton { background-color: #c0392b; color: white; "
    "font-weight: bold; font-size: 14pt; padding: 10px; }"
    "QPushButton:hover { background-color: #e74c3c; }");
  btn_estop_->setMinimumHeight(50);
  layout->addWidget(btn_estop_);

  // Phase 1 group
  auto phase1_box = new QGroupBox("Phase 1: Frontier Exploration");
  auto phase1_layout = new QVBoxLayout;
  btn_start_ = new QPushButton("▶ Start Frontier");
  btn_finish_ = new QPushButton("✓ Done → switch to Phase 2");
  phase1_layout->addWidget(btn_start_);
  phase1_layout->addWidget(btn_finish_);
  phase1_box->setLayout(phase1_layout);
  layout->addWidget(phase1_box);

  // Phase 2 group
  auto phase2_box = new QGroupBox("Phase 2: Navigation on saved map");
  auto phase2_layout = new QVBoxLayout;
  auto path_row = new QHBoxLayout;
  map_path_edit_ = new QLineEdit("/home/kris_nano/walker_ws/maps/auto_map.yaml");
  btn_browse_ = new QPushButton("…");
  btn_browse_->setMaximumWidth(36);
  path_row->addWidget(map_path_edit_);
  path_row->addWidget(btn_browse_);
  phase2_layout->addLayout(path_row);
  btn_load_ = new QPushButton("📂 Load Saved Map");
  btn_reset_ = new QPushButton("⟳ Reset Initial Pose (0,0,0)");
  phase2_layout->addWidget(btn_load_);
  phase2_layout->addWidget(btn_reset_);
  phase2_box->setLayout(phase2_layout);
  layout->addWidget(phase2_box);

  status_ = new QLabel("status: idle");
  status_->setWordWrap(true);
  status_->setStyleSheet("QLabel { color: gray; }");
  layout->addWidget(status_);

  layout->addStretch();
  setLayout(layout);

  connect(btn_start_, &QPushButton::clicked, this, &WalkerPanel::onStartFrontier);
  connect(btn_finish_, &QPushButton::clicked, this, &WalkerPanel::onFinishToPhase2);
  connect(btn_load_, &QPushButton::clicked, this, &WalkerPanel::onLoadMap);
  connect(btn_reset_, &QPushButton::clicked, this, &WalkerPanel::onResetPose);
  connect(btn_browse_, &QPushButton::clicked, this, &WalkerPanel::onBrowseMap);
  connect(btn_estop_, &QPushButton::clicked, this, &WalkerPanel::onEmergencyStop);
}

WalkerPanel::~WalkerPanel() = default;

void WalkerPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  cli_start_ = node_->create_client<std_srvs::srv::Trigger>("/walker/start_frontier");
  cli_finish_ = node_->create_client<std_srvs::srv::Trigger>("/walker/finish_to_phase2");
  cli_load_ = node_->create_client<std_srvs::srv::Trigger>("/walker/load_map");
  cli_reset_ = node_->create_client<std_srvs::srv::Trigger>("/walker/reset_pose");
  cli_stop_ = node_->create_client<std_srvs::srv::Trigger>("/walker/stop");

  session_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, "/session_manager");

  setStatus("ready", true);
}

void WalkerPanel::setStatus(const QString & text, bool ok)
{
  status_->setText("status: " + text);
  status_->setStyleSheet(
    ok ? "QLabel { color: #2c8b3d; }" : "QLabel { color: #b04040; }");
}

void WalkerPanel::callTrigger(const std::string & service_name,
                              const std::string & friendly)
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli;
  if (service_name == "/walker/start_frontier") cli = cli_start_;
  else if (service_name == "/walker/finish_to_phase2") cli = cli_finish_;
  else if (service_name == "/walker/load_map") cli = cli_load_;
  else if (service_name == "/walker/reset_pose") cli = cli_reset_;
  else if (service_name == "/walker/stop") cli = cli_stop_;
  else { setStatus(QString::fromStdString("unknown service: " + service_name), false); return; }

  if (!cli->wait_for_service(std::chrono::seconds(1))) {
    setStatus(QString::fromStdString(friendly + " — session_manager not available"), false);
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = cli->async_send_request(req);
  setStatus(QString::fromStdString(friendly + " — sent"), true);
  // we don't block; assume manager logs result
}

void WalkerPanel::onStartFrontier()
{
  callTrigger("/walker/start_frontier", "start_frontier");
}

void WalkerPanel::onFinishToPhase2()
{
  // Make sure map_path param matches current edit text
  if (session_param_client_->service_is_ready()) {
    session_param_client_->set_parameters(
      {rclcpp::Parameter("map_path", map_path_edit_->text().toStdString())});
  }
  callTrigger("/walker/finish_to_phase2", "finish_to_phase2");
}

void WalkerPanel::onLoadMap()
{
  if (session_param_client_->service_is_ready()) {
    session_param_client_->set_parameters(
      {rclcpp::Parameter("map_path", map_path_edit_->text().toStdString())});
  }
  callTrigger("/walker/load_map", "load_map");
}

void WalkerPanel::onResetPose()
{
  callTrigger("/walker/reset_pose", "reset_pose");
}

void WalkerPanel::onEmergencyStop()
{
  callTrigger("/walker/stop", "EMERGENCY STOP");
}

void WalkerPanel::onBrowseMap()
{
  QString fn = QFileDialog::getOpenFileName(
    this, "Select map YAML", map_path_edit_->text(), "Map files (*.yaml)");
  if (!fn.isEmpty()) {
    map_path_edit_->setText(fn);
  }
}

}  // namespace slamwalker_rviz_panel

PLUGINLIB_EXPORT_CLASS(slamwalker_rviz_panel::WalkerPanel, rviz_common::Panel)
