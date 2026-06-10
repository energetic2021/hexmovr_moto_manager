#include "hexmovr_moto_panel/hexmovr_panel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonValue>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace
{

constexpr int kMotorTableColumns = 6;
constexpr int kFaultTableColumns = 4;
constexpr int kPlotHistoryLimit = 300;

QString timestampString(double stamp_seconds)
{
  const qint64 ms = static_cast<qint64>(stamp_seconds * 1000.0);
  return QDateTime::fromMSecsSinceEpoch(ms).toString("yyyy-MM-dd hh:mm:ss");
}

std::vector<QString> controlParamNames()
{
  return {
    "position_max_speed",
    "max_q_current",
    "current_slope",
    "velocity_acceleration",
    "position_kp",
    "position_ki",
    "velocity_kp",
    "velocity_ki",
  };
}

std::vector<QString> advancedParamNames()
{
  return {
    "trapezoid_acceleration",
    "trapezoid_deceleration",
    "position_filter_bandwidth",
    "position_filter_inertia",
    "position_filter_feedforward_current",
  };
}

}  // namespace

namespace hexmovr_moto_panel
{

PlotWidget::PlotWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(220);
  empty_text_ = "Waiting for telemetry";
}

void PlotWidget::setSeries(const QVector<QPointF> & points, const QString & title)
{
  points_ = points;
  title_ = title;
  update();
}

void PlotWidget::setEmptyText(const QString & text)
{
  empty_text_ = text;
  update();
}

void PlotWidget::paintEvent(QPaintEvent * event)
{
  QWidget::paintEvent(event);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(24, 28, 35));

  const QRectF canvas = rect().adjusted(12.0, 24.0, -12.0, -16.0);
  painter.setPen(QPen(QColor(62, 72, 88), 1.0));
  for (int i = 0; i < 5; ++i) {
    const qreal y = canvas.top() + canvas.height() * i / 4.0;
    painter.drawLine(QPointF(canvas.left(), y), QPointF(canvas.right(), y));
  }
  for (int i = 0; i < 6; ++i) {
    const qreal x = canvas.left() + canvas.width() * i / 5.0;
    painter.drawLine(QPointF(x, canvas.top()), QPointF(x, canvas.bottom()));
  }

  painter.setPen(QColor(240, 240, 240));
  painter.drawText(QRectF(12.0, 4.0, width() - 24.0, 18.0), title_);

  if (points_.size() < 2) {
    painter.setPen(QColor(180, 180, 180));
    painter.drawText(canvas, Qt::AlignCenter, empty_text_);
    return;
  }

  qreal min_x = points_.first().x();
  qreal max_x = points_.last().x();
  qreal min_y = std::numeric_limits<qreal>::max();
  qreal max_y = std::numeric_limits<qreal>::lowest();
  for (const auto & point : points_) {
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }
  if (std::abs(max_y - min_y) < 1e-6) {
    max_y += 1.0;
    min_y -= 1.0;
  }
  if (std::abs(max_x - min_x) < 1e-6) {
    max_x += 1.0;
  }

  QPainterPath path;
  for (int i = 0; i < points_.size(); ++i) {
    const auto & point = points_.at(i);
    const qreal norm_x = (point.x() - min_x) / (max_x - min_x);
    const qreal norm_y = (point.y() - min_y) / (max_y - min_y);
    const QPointF mapped(
      canvas.left() + norm_x * canvas.width(),
      canvas.bottom() - norm_y * canvas.height());
    if (i == 0) {
      path.moveTo(mapped);
    } else {
      path.lineTo(mapped);
    }
  }

  painter.setPen(QPen(QColor(72, 190, 255), 2.0));
  painter.drawPath(path);

  painter.setPen(QColor(200, 200, 200));
  painter.drawText(
    QRectF(canvas.left(), canvas.bottom() - 2.0, canvas.width(), 16.0),
    Qt::AlignRight,
    QString("min=%1  max=%2").arg(min_y, 0, 'f', 3).arg(max_y, 0, 'f', 3));
}

HexmovrPanel::HexmovrPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
}

void HexmovrPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();
  setupRos();
}

void HexmovrPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("selected_motor", motor_select_->currentText());
  config.mapSetValue("plot_metric", plot_metric_combo_->currentData().toString());
  config.mapSetValue(
    "language", language_ == DisplayLanguage::Chinese ? "zh_CN" : "en");
}

void HexmovrPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString selected_motor;
  QString plot_metric;
  QString language_code;
  if (config.mapGetString("language", &language_code)) {
    language_ = language_code == "zh_CN" ? DisplayLanguage::Chinese : DisplayLanguage::English;
    if (language_combo_) {
      language_combo_->setCurrentIndex(
        language_ == DisplayLanguage::Chinese ? 1 : 0);
    }
    applyLanguage();
  }
  if (config.mapGetString("selected_motor", &selected_motor)) {
    const int index = motor_select_->findText(selected_motor);
    if (index >= 0) {
      motor_select_->setCurrentIndex(index);
    }
  }
  if (config.mapGetString("plot_metric", &plot_metric)) {
    int index = plot_metric_combo_->findData(plot_metric);
    if (index < 0) {
      index = plot_metric_combo_->findText(plot_metric);
    }
    if (index >= 0) {
      plot_metric_combo_->setCurrentIndex(index);
    }
  }
}

void HexmovrPanel::buildUi()
{
  const auto locale_name = QLocale::system().name();
  language_ = locale_name.startsWith("zh") ? DisplayLanguage::Chinese : DisplayLanguage::English;

  auto bind_text_key = [](QObject * object, const char * key) {
      object->setProperty("ui_key", key);
    };

  auto * root_layout = new QVBoxLayout(this);

  connection_label_ = new QLabel("Hexmovr panel is starting...", this);
  latest_event_label_ = new QLabel("No events yet", this);
  latest_event_label_->setWordWrap(true);
  language_label_ = new QLabel(this);
  language_combo_ = new QComboBox(this);
  bind_text_key(language_label_, "language");
  language_combo_->addItem("English", "en");
  language_combo_->addItem(QString::fromUtf8("中文"), "zh_CN");
  language_combo_->setCurrentIndex(language_ == DisplayLanguage::Chinese ? 1 : 0);

  auto * header_layout = new QHBoxLayout();
  auto * status_layout = new QVBoxLayout();
  status_layout->addWidget(connection_label_);
  status_layout->addWidget(latest_event_label_);
  header_layout->addLayout(status_layout, 1);
  auto * language_layout = new QHBoxLayout();
  language_layout->addWidget(language_label_);
  language_layout->addWidget(language_combo_);
  header_layout->addLayout(language_layout);
  root_layout->addLayout(header_layout);

  auto * top_actions = new QHBoxLayout();
  auto * scan_button = new QPushButton("Scan", this);
  auto * refresh_all_button = new QPushButton("Refresh All", this);
  auto * clear_backend_history_button = new QPushButton("Clear History", this);
  bind_text_key(scan_button, "scan");
  bind_text_key(refresh_all_button, "refresh_all");
  bind_text_key(clear_backend_history_button, "clear_history");
  top_actions->addWidget(scan_button);
  top_actions->addWidget(refresh_all_button);
  top_actions->addWidget(clear_backend_history_button);
  top_actions->addStretch(1);
  root_layout->addLayout(top_actions);

  tabs_ = new QTabWidget(this);
  root_layout->addWidget(tabs_, 1);

  auto * motors_tab = new QWidget(this);
  bind_text_key(motors_tab, "tab_motors");
  auto * motors_layout = new QHBoxLayout(motors_tab);
  auto * motors_splitter = new QSplitter(Qt::Horizontal, motors_tab);
  motors_layout->addWidget(motors_splitter);

  auto * table_container = new QWidget(motors_splitter);
  auto * table_layout = new QVBoxLayout(table_container);
  auto * discovered_motors_label = new QLabel(table_container);
  bind_text_key(discovered_motors_label, "discovered_motors");
  motor_table_ = new QTableWidget(0, kMotorTableColumns, table_container);
  motor_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  motor_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  motor_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  motor_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_layout->addWidget(discovered_motors_label);
  table_layout->addWidget(motor_table_);
  motors_splitter->addWidget(table_container);

  auto * detail_container = new QWidget(motors_splitter);
  auto * detail_layout = new QVBoxLayout(detail_container);

  motor_select_ = new QComboBox(detail_container);
  auto * selected_motor_label = new QLabel(detail_container);
  bind_text_key(selected_motor_label, "selected_motor");
  detail_layout->addWidget(selected_motor_label);
  detail_layout->addWidget(motor_select_);

  auto * action_box = new QGroupBox("Motor Actions", detail_container);
  bind_text_key(action_box, "motor_actions");
  auto * action_layout = new QGridLayout(action_box);
  auto * refresh_motor_button = new QPushButton("Refresh", action_box);
  auto * clear_error_button = new QPushButton("Clear Error", action_box);
  auto * set_zero_button = new QPushButton("Set Zero", action_box);
  auto * free_motor_button = new QPushButton("Free Motor", action_box);
  auto * return_zero_button = new QPushButton("Return To Zero", action_box);
  auto * brake_open_button = new QPushButton("Brake Open", action_box);
  auto * brake_close_button = new QPushButton("Brake Close", action_box);
  bind_text_key(refresh_motor_button, "refresh");
  bind_text_key(clear_error_button, "clear_error");
  bind_text_key(set_zero_button, "set_zero");
  bind_text_key(free_motor_button, "free_motor");
  bind_text_key(return_zero_button, "return_to_zero");
  bind_text_key(brake_open_button, "brake_open");
  bind_text_key(brake_close_button, "brake_close");
  action_layout->addWidget(refresh_motor_button, 0, 0);
  action_layout->addWidget(clear_error_button, 0, 1);
  action_layout->addWidget(set_zero_button, 1, 0);
  action_layout->addWidget(free_motor_button, 1, 1);
  action_layout->addWidget(return_zero_button, 2, 0);
  action_layout->addWidget(brake_open_button, 2, 1);
  action_layout->addWidget(brake_close_button, 3, 0, 1, 2);
  detail_layout->addWidget(action_box);

  auto * control_box = new QGroupBox("Control Forms", detail_container);
  bind_text_key(control_box, "control_forms");
  auto * control_layout = new QFormLayout(control_box);
  absolute_position_box_ = new QDoubleSpinBox(control_box);
  absolute_position_box_->setRange(-1000.0, 1000.0);
  absolute_position_box_->setDecimals(4);
  auto * absolute_button = new QPushButton("Send Absolute Position", control_box);
  relative_position_box_ = new QDoubleSpinBox(control_box);
  relative_position_box_->setRange(-1000.0, 1000.0);
  relative_position_box_->setDecimals(4);
  auto * relative_button = new QPushButton("Send Relative Position", control_box);
  velocity_box_ = new QDoubleSpinBox(control_box);
  velocity_box_->setRange(-500.0, 500.0);
  velocity_box_->setDecimals(4);
  auto * velocity_button = new QPushButton("Send Velocity", control_box);
  current_box_ = new QDoubleSpinBox(control_box);
  current_box_->setRange(-200.0, 200.0);
  current_box_->setDecimals(4);
  auto * current_button = new QPushButton("Send Current", control_box);
  mit_position_box_ = new QDoubleSpinBox(control_box);
  mit_position_box_->setRange(-1000.0, 1000.0);
  mit_position_box_->setDecimals(4);
  mit_velocity_box_ = new QDoubleSpinBox(control_box);
  mit_velocity_box_->setRange(-500.0, 500.0);
  mit_velocity_box_->setDecimals(4);
  mit_stiffness_box_ = new QDoubleSpinBox(control_box);
  mit_stiffness_box_->setRange(0.0, 500.0);
  mit_stiffness_box_->setValue(30.0);
  mit_damping_box_ = new QDoubleSpinBox(control_box);
  mit_damping_box_->setRange(0.0, 10.0);
  mit_damping_box_->setValue(1.0);
  mit_torque_box_ = new QDoubleSpinBox(control_box);
  mit_torque_box_->setRange(-100.0, 100.0);
  mit_torque_box_->setDecimals(4);
  auto * mit_button = new QPushButton("Send MIT Control", control_box);
  bind_text_key(absolute_button, "send_absolute_position");
  bind_text_key(relative_button, "send_relative_position");
  bind_text_key(velocity_button, "send_velocity");
  bind_text_key(current_button, "send_current");
  bind_text_key(mit_button, "send_mit_control");

  control_layout->addRow("Absolute Position (rad)", absolute_position_box_);
  control_layout->addRow(absolute_button);
  control_layout->addRow("Relative Position (rad)", relative_position_box_);
  control_layout->addRow(relative_button);
  control_layout->addRow("Velocity (rad/s)", velocity_box_);
  control_layout->addRow(velocity_button);
  control_layout->addRow("Current (A)", current_box_);
  control_layout->addRow(current_button);
  control_layout->addRow("MIT Position (rad)", mit_position_box_);
  control_layout->addRow("MIT Velocity (rad/s)", mit_velocity_box_);
  control_layout->addRow("MIT Stiffness", mit_stiffness_box_);
  control_layout->addRow("MIT Damping", mit_damping_box_);
  control_layout->addRow("MIT Torque (Nm)", mit_torque_box_);
  control_layout->addRow(mit_button);
  detail_layout->addWidget(control_box);

  auto * param_box = new QGroupBox("Parameter Form", detail_container);
  bind_text_key(param_box, "parameter_form");
  auto * param_layout = new QFormLayout(param_box);
  param_group_combo_ = new QComboBox(param_box);
  param_name_combo_ = new QComboBox(param_box);
  param_value_box_ = new QDoubleSpinBox(param_box);
  param_value_box_->setRange(-1000000.0, 1000000.0);
  param_value_box_->setDecimals(6);
  auto * apply_param_button = new QPushButton("Apply Parameter", param_box);
  bind_text_key(apply_param_button, "apply_parameter");
  param_layout->addRow("Group", param_group_combo_);
  param_layout->addRow("Name", param_name_combo_);
  param_layout->addRow("Value", param_value_box_);
  param_layout->addRow(apply_param_button);
  detail_layout->addWidget(param_box);

  motor_snapshot_view_ = new QPlainTextEdit(detail_container);
  motor_snapshot_view_->setReadOnly(true);
  auto * telemetry_snapshot_label = new QLabel(detail_container);
  bind_text_key(telemetry_snapshot_label, "telemetry_snapshot");
  detail_layout->addWidget(telemetry_snapshot_label);
  detail_layout->addWidget(motor_snapshot_view_, 1);

  motors_splitter->addWidget(detail_container);
  tabs_->addTab(motors_tab, "Motors");

  auto * batch_tab = new QWidget(this);
  bind_text_key(batch_tab, "tab_batch");
  auto * batch_layout = new QVBoxLayout(batch_tab);
  auto * batch_target_box = new QGroupBox("Batch Targets", batch_tab);
  bind_text_key(batch_target_box, "batch_targets");
  auto * batch_target_layout = new QFormLayout(batch_target_box);
  batch_all_checkbox_ = new QCheckBox("Use all discovered motors", batch_target_box);
  bind_text_key(batch_all_checkbox_, "use_all_discovered_motors");
  batch_all_checkbox_->setChecked(true);
  batch_ids_edit_ = new QLineEdit(batch_target_box);
  batch_ids_edit_->setPlaceholderText("Example: 1,2,5");
  batch_target_layout->addRow(batch_all_checkbox_);
  auto * manual_ids_label = new QLabel(batch_target_box);
  bind_text_key(manual_ids_label, "manual_ids");
  batch_target_layout->addRow(manual_ids_label, batch_ids_edit_);
  batch_layout->addWidget(batch_target_box);

  auto * batch_action_box = new QGroupBox("Batch Actions", batch_tab);
  bind_text_key(batch_action_box, "batch_actions");
  auto * batch_action_layout = new QGridLayout(batch_action_box);
  auto * batch_scan_button = new QPushButton("Scan", batch_action_box);
  auto * batch_refresh_button = new QPushButton("Refresh All", batch_action_box);
  auto * batch_clear_button = new QPushButton("Clear Errors", batch_action_box);
  auto * batch_zero_button = new QPushButton("Set Zero", batch_action_box);
  auto * batch_free_button = new QPushButton("Free Motors", batch_action_box);
  auto * batch_brake_open_button = new QPushButton("Brake Open", batch_action_box);
  auto * batch_brake_close_button = new QPushButton("Brake Close", batch_action_box);
  bind_text_key(batch_scan_button, "scan");
  bind_text_key(batch_refresh_button, "refresh_all");
  bind_text_key(batch_clear_button, "clear_errors");
  bind_text_key(batch_zero_button, "set_zero");
  bind_text_key(batch_free_button, "free_motors");
  bind_text_key(batch_brake_open_button, "brake_open");
  bind_text_key(batch_brake_close_button, "brake_close");
  batch_action_layout->addWidget(batch_scan_button, 0, 0);
  batch_action_layout->addWidget(batch_refresh_button, 0, 1);
  batch_action_layout->addWidget(batch_clear_button, 1, 0);
  batch_action_layout->addWidget(batch_zero_button, 1, 1);
  batch_action_layout->addWidget(batch_free_button, 2, 0);
  batch_action_layout->addWidget(batch_brake_open_button, 2, 1);
  batch_action_layout->addWidget(batch_brake_close_button, 3, 0, 1, 2);
  batch_layout->addWidget(batch_action_box);

  auto * batch_control_box = new QGroupBox("Batch Control", batch_tab);
  bind_text_key(batch_control_box, "batch_control");
  auto * batch_control_layout = new QFormLayout(batch_control_box);
  batch_mode_combo_ = new QComboBox(batch_control_box);
  batch_mode_combo_->addItems(
    {"absolute_position", "relative_position", "velocity", "current"});
  batch_value_box_ = new QDoubleSpinBox(batch_control_box);
  batch_value_box_->setRange(-1000.0, 1000.0);
  batch_value_box_->setDecimals(4);
  auto * batch_control_button = new QPushButton("Apply Batch Control", batch_control_box);
  bind_text_key(batch_control_button, "apply_batch_control");
  batch_control_layout->addRow("Mode", batch_mode_combo_);
  batch_control_layout->addRow("Value", batch_value_box_);
  batch_control_layout->addRow(batch_control_button);
  batch_layout->addWidget(batch_control_box);

  auto * batch_param_box = new QGroupBox("Batch Parameter Form", batch_tab);
  bind_text_key(batch_param_box, "batch_parameter_form");
  auto * batch_param_layout = new QFormLayout(batch_param_box);
  batch_param_group_combo_ = new QComboBox(batch_param_box);
  batch_param_name_combo_ = new QComboBox(batch_param_box);
  batch_param_value_box_ = new QDoubleSpinBox(batch_param_box);
  batch_param_value_box_->setRange(-1000000.0, 1000000.0);
  batch_param_value_box_->setDecimals(6);
  auto * batch_param_button = new QPushButton("Apply Batch Parameter", batch_param_box);
  bind_text_key(batch_param_button, "apply_batch_parameter");
  batch_param_layout->addRow("Group", batch_param_group_combo_);
  batch_param_layout->addRow("Name", batch_param_name_combo_);
  batch_param_layout->addRow("Value", batch_param_value_box_);
  batch_param_layout->addRow(batch_param_button);
  batch_layout->addWidget(batch_param_box);
  batch_layout->addStretch(1);
  tabs_->addTab(batch_tab, "Batch");

  auto * fault_tab = new QWidget(this);
  bind_text_key(fault_tab, "tab_faults");
  auto * fault_layout = new QVBoxLayout(fault_tab);
  fault_table_ = new QTableWidget(0, kFaultTableColumns, fault_tab);
  fault_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  fault_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  auto * clear_local_fault_button = new QPushButton("Clear Visible History", fault_tab);
  bind_text_key(clear_local_fault_button, "clear_visible_history");
  fault_layout->addWidget(fault_table_);
  fault_layout->addWidget(clear_local_fault_button);
  tabs_->addTab(fault_tab, "Faults");

  auto * plot_tab = new QWidget(this);
  bind_text_key(plot_tab, "tab_plots");
  auto * plot_layout = new QVBoxLayout(plot_tab);
  auto * plot_controls = new QHBoxLayout();
  auto * plot_motor_label = new QLabel(plot_tab);
  plot_motor_combo_ = new QComboBox(plot_tab);
  auto * plot_metric_label = new QLabel(plot_tab);
  plot_metric_combo_ = new QComboBox(plot_tab);
  bind_text_key(plot_motor_label, "motor");
  bind_text_key(plot_metric_label, "metric");
  plot_controls->addWidget(plot_motor_label);
  plot_controls->addWidget(plot_motor_combo_);
  plot_controls->addWidget(plot_metric_label);
  plot_controls->addWidget(plot_metric_combo_);
  plot_controls->addStretch(1);
  plot_layout->addLayout(plot_controls);
  plot_widget_ = new PlotWidget(plot_tab);
  plot_layout->addWidget(plot_widget_, 1);
  tabs_->addTab(plot_tab, "Plots");

  updateParamNameOptions();
  updateBatchParamNameOptions();
  applyLanguage();

  connect(language_combo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
    language_ =
      language_combo_->currentData().toString() == "zh_CN" ?
      DisplayLanguage::Chinese : DisplayLanguage::English;
    applyLanguage();
    updateMotorDetails();
    refreshPlot();
    handleHistoryMessage(std::make_shared<std_msgs::msg::String>(std_msgs::msg::String{}));
    updateConnectionBanner(
      connected_,
      connected_ ?
      connectionDetailText(
        true,
        QString("%1|%2|%3").arg(can_interface_).arg(motors_.size()).arg(manager_history_size_)) :
      connectionDetailText(false, transport_error_));
  });

  connect(scan_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "scan"}});
  });
  connect(refresh_all_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "refresh_all"}, {"deep", true}});
  });
  connect(clear_backend_history_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "clear_history"}});
  });

  connect(motor_select_, &QComboBox::currentTextChanged, this, [this](const QString &) {
    updateMotorDetails();
    refreshPlot();
  });
  connect(motor_table_, &QTableWidget::cellClicked, this, [this](int row, int) {
    if (row < 0 || row >= motor_table_->rowCount()) {
      return;
    }
    const auto * item = motor_table_->item(row, 0);
    if (!item) {
      return;
    }
    setSelectedMotorId(item->text().toInt());
  });
  connect(param_group_combo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
    updateParamNameOptions();
  });
  connect(batch_param_group_combo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
    updateBatchParamNameOptions();
  });
  connect(plot_motor_combo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
    refreshPlot();
  });
  connect(plot_metric_combo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
    refreshPlot();
  });

  connect(refresh_motor_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "refresh"}, {"motor_id", selectedMotorId()}, {"deep", true}});
  });
  connect(clear_error_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "clear_error"}, {"motor_id", selectedMotorId()}});
  });
  connect(set_zero_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "set_zero"}, {"motor_id", selectedMotorId()}});
  });
  connect(free_motor_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "free_motor"}, {"motor_id", selectedMotorId()}});
  });
  connect(return_zero_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "return_to_zero"}, {"motor_id", selectedMotorId()}});
  });
  connect(brake_open_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "brake"}, {"motor_id", selectedMotorId()}, {"closed", false}});
  });
  connect(brake_close_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "brake"}, {"motor_id", selectedMotorId()}, {"closed", true}});
  });

  connect(absolute_button, &QPushButton::clicked, this, [this]() {
    publishCommand(
      QJsonObject{
        {"op", "control"},
        {"motor_id", selectedMotorId()},
        {"mode", "absolute_position"},
        {"position_rad", absolute_position_box_->value()}
      });
  });
  connect(relative_button, &QPushButton::clicked, this, [this]() {
    publishCommand(
      QJsonObject{
        {"op", "control"},
        {"motor_id", selectedMotorId()},
        {"mode", "relative_position"},
        {"position_rad", relative_position_box_->value()}
      });
  });
  connect(velocity_button, &QPushButton::clicked, this, [this]() {
    publishCommand(
      QJsonObject{
        {"op", "control"},
        {"motor_id", selectedMotorId()},
        {"mode", "velocity"},
        {"velocity_rad_s", velocity_box_->value()}
      });
  });
  connect(current_button, &QPushButton::clicked, this, [this]() {
    publishCommand(
      QJsonObject{
        {"op", "control"},
        {"motor_id", selectedMotorId()},
        {"mode", "current"},
        {"current_a", current_box_->value()}
      });
  });
  connect(mit_button, &QPushButton::clicked, this, [this]() {
    publishCommand(
      QJsonObject{
        {"op", "control"},
        {"motor_id", selectedMotorId()},
        {"mode", "mit"},
        {"position_rad", mit_position_box_->value()},
        {"velocity_rad_s", mit_velocity_box_->value()},
        {"stiffness", mit_stiffness_box_->value()},
        {"damping", mit_damping_box_->value()},
        {"torque_nm", mit_torque_box_->value()}
      });
  });
  connect(apply_param_button, &QPushButton::clicked, this, [this]() {
    publishCommand(
      QJsonObject{
        {"op", "set_param"},
        {"motor_id", selectedMotorId()},
        {"group", currentParamGroup()},
        {"name", param_name_combo_->currentData().toString()},
        {"value", param_value_box_->value()}
      });
  });

  connect(batch_scan_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "scan"}});
  });
  connect(batch_refresh_button, &QPushButton::clicked, this, [this]() {
    publishCommand(QJsonObject{{"op", "refresh_all"}, {"deep", true}});
  });
  connect(batch_clear_button, &QPushButton::clicked, this, [this]() {
    QJsonArray ids;
    for (int motor_id : currentBatchTargets()) {
      ids.append(motor_id);
    }
    publishCommand(
      QJsonObject{
        {"op", "batch"},
        {"all", batch_all_checkbox_->isChecked()},
        {"motor_ids", ids},
        {"command", QJsonObject{{"op", "clear_error"}}}
      });
  });
  connect(batch_zero_button, &QPushButton::clicked, this, [this]() {
    QJsonArray ids;
    for (int motor_id : currentBatchTargets()) {
      ids.append(motor_id);
    }
    publishCommand(
      QJsonObject{
        {"op", "batch"},
        {"all", batch_all_checkbox_->isChecked()},
        {"motor_ids", ids},
        {"command", QJsonObject{{"op", "set_zero"}}}
      });
  });
  connect(batch_free_button, &QPushButton::clicked, this, [this]() {
    QJsonArray ids;
    for (int motor_id : currentBatchTargets()) {
      ids.append(motor_id);
    }
    publishCommand(
      QJsonObject{
        {"op", "batch"},
        {"all", batch_all_checkbox_->isChecked()},
        {"motor_ids", ids},
        {"command", QJsonObject{{"op", "free_motor"}}}
      });
  });
  connect(batch_brake_open_button, &QPushButton::clicked, this, [this]() {
    QJsonArray ids;
    for (int motor_id : currentBatchTargets()) {
      ids.append(motor_id);
    }
    publishCommand(
      QJsonObject{
        {"op", "batch"},
        {"all", batch_all_checkbox_->isChecked()},
        {"motor_ids", ids},
        {"command", QJsonObject{{"op", "brake"}, {"closed", false}}}
      });
  });
  connect(batch_brake_close_button, &QPushButton::clicked, this, [this]() {
    QJsonArray ids;
    for (int motor_id : currentBatchTargets()) {
      ids.append(motor_id);
    }
    publishCommand(
      QJsonObject{
        {"op", "batch"},
        {"all", batch_all_checkbox_->isChecked()},
        {"motor_ids", ids},
        {"command", QJsonObject{{"op", "brake"}, {"closed", true}}}
      });
  });
  connect(batch_control_button, &QPushButton::clicked, this, [this]() {
    QJsonObject nested{
      {"op", "control"},
      {"mode", currentBatchMode()}
    };
    const auto mode = currentBatchMode();
    if (mode == "current") {
      nested["current_a"] = batch_value_box_->value();
    } else if (mode == "velocity") {
      nested["velocity_rad_s"] = batch_value_box_->value();
    } else {
      nested["position_rad"] = batch_value_box_->value();
    }
    QJsonArray ids;
    for (int motor_id : currentBatchTargets()) {
      ids.append(motor_id);
    }
    publishCommand(
      QJsonObject{
        {"op", "batch"},
        {"all", batch_all_checkbox_->isChecked()},
        {"motor_ids", ids},
        {"command", nested}
      });
  });
  connect(batch_param_button, &QPushButton::clicked, this, [this]() {
    QJsonArray ids;
    for (int motor_id : currentBatchTargets()) {
      ids.append(motor_id);
    }
    publishCommand(
      QJsonObject{
        {"op", "batch"},
        {"all", batch_all_checkbox_->isChecked()},
        {"motor_ids", ids},
        {"command",
          QJsonObject{
            {"op", "set_param"},
            {"group", currentBatchParamGroup()},
            {"name", batch_param_name_combo_->currentData().toString()},
            {"value", batch_param_value_box_->value()}
          }}
      });
  });

  connect(clear_local_fault_button, &QPushButton::clicked, this, [this]() {
    fault_history_ = QJsonArray();
    handleHistoryMessage(std::make_shared<std_msgs::msg::String>(std_msgs::msg::String{}));
  });
}

void HexmovrPanel::setupRos()
{
  if (!rclcpp::ok()) {
    return;
  }
  node_ = std::make_shared<rclcpp::Node>("hexmovr_motor_panel");
  command_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/hexmovr_moto_manager/command", 10);
  state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/hexmovr_moto_manager/state",
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) { handleStateMessage(msg); });
  event_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/hexmovr_moto_manager/event",
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) { handleEventMessage(msg); });
  history_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/hexmovr_moto_manager/history",
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) { handleHistoryMessage(msg); });

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, [this]() {
    if (node_) {
      rclcpp::spin_some(node_);
    }
  });
  spin_timer_->start(50);
  updateConnectionBanner(false, uiText("waiting_manager_state"));
}

void HexmovrPanel::publishCommand(const QJsonObject & command)
{
  if (!command_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = QJsonDocument(command).toJson(QJsonDocument::Compact).toStdString();
  command_pub_->publish(msg);
}

void HexmovrPanel::handleStateMessage(const std_msgs::msg::String::SharedPtr msg)
{
  const auto json = QByteArray::fromStdString(msg->data);
  const auto doc = QJsonDocument::fromJson(json);
  if (!doc.isObject()) {
    return;
  }
  const auto root = doc.object();
  connected_ = root.value("connected").toBool(false);
  can_interface_ = root.value("can_interface").toString();
  manager_history_size_ = root.value("history_size").toInt();
  transport_error_ = root.value("transport_error").toString();
  updateConnectionBanner(
    connected_,
    connected_ ?
    connectionDetailText(
      true,
      QString("%1|%2|%3")
      .arg(can_interface_)
      .arg(root.value("motor_count").toInt())
      .arg(manager_history_size_)) :
    connectionDetailText(false, transport_error_));

  motors_.clear();
  const auto motors = root.value("motors").toArray();
  for (const auto & motor_value : motors) {
    const auto motor = motor_value.toObject();
    MotorRecord record;
    record.motor_id = motor.value("motor_id").toInt();
    record.last_seen = motor.value("last_seen").toDouble();
    record.last_error = motor.value("last_error").toString();
    record.snapshot = motor.value("snapshot").toObject();
    motors_[record.motor_id] = record;
    appendPlotSamples(record.motor_id, record.snapshot);
  }
  updateMotorWidgets();
  updateMotorDetails();
  refreshPlot();
}

void HexmovrPanel::handleEventMessage(const std_msgs::msg::String::SharedPtr msg)
{
  const auto json = QByteArray::fromStdString(msg->data);
  const auto doc = QJsonDocument::fromJson(json);
  if (!doc.isObject()) {
    latest_event_ = QString::fromStdString(msg->data);
  } else {
    const auto root = doc.object();
    latest_event_ = QString("%1 | %2")
      .arg(root.value("event").toString())
      .arg(QString::fromUtf8(QJsonDocument(root.value("payload").toObject()).toJson(
        QJsonDocument::Compact)));
  }
  latest_event_label_->setText(latest_event_);
}

void HexmovrPanel::handleHistoryMessage(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg->data.empty()) {
    const auto json = QByteArray::fromStdString(msg->data);
    const auto doc = QJsonDocument::fromJson(json);
    if (doc.isObject()) {
      fault_history_ = doc.object().value("fault_history").toArray();
    }
  }

  fault_table_->setRowCount(fault_history_.size());
  for (int row = 0; row < fault_history_.size(); ++row) {
    const auto entry = fault_history_.at(row).toObject();
    fault_table_->setItem(
      row, 0, new QTableWidgetItem(timestampString(entry.value("timestamp").toDouble())));
    fault_table_->setItem(
      row, 1, new QTableWidgetItem(
        entry.value("motor_id").isNull() ? "-" : QString::number(entry.value("motor_id").toInt())));
    fault_table_->setItem(
      row, 2, new QTableWidgetItem(historyKindDisplayName(entry.value("kind").toString())));
    fault_table_->setItem(row, 3, new QTableWidgetItem(entry.value("message").toString()));
  }
}

void HexmovrPanel::updateConnectionBanner(bool connected, const QString & detail)
{
  if (connected) {
    connection_label_->setText(detail);
    connection_label_->setStyleSheet("QLabel { color: #1f8f4a; font-weight: 600; }");
  } else {
    connection_label_->setText(detail);
    connection_label_->setStyleSheet("QLabel { color: #c25a1c; font-weight: 600; }");
  }
}

void HexmovrPanel::updateMotorWidgets()
{
  const int previous_motor = selectedMotorId();
  motor_table_->setRowCount(motors_.size());
  int row = 0;
  for (auto it = motors_.cbegin(); it != motors_.cend(); ++it, ++row) {
    const auto & record = it.value();
    motor_table_->setItem(row, 0, new QTableWidgetItem(QString::number(record.motor_id)));
    motor_table_->setItem(
      row, 1,
      new QTableWidgetItem(QString("0x%1").arg(
          record.snapshot.value("fault_code").toInt(), 2, 16, QLatin1Char('0'))));
    motor_table_->setItem(
      row, 2,
      new QTableWidgetItem(QString::number(
          readSnapshotNumber(record.snapshot, "temperature_c"), 'f', 1)));
    motor_table_->setItem(
      row, 3,
      new QTableWidgetItem(QString::number(
          readSnapshotNumber(record.snapshot, "position_rad"), 'f', 3)));
    motor_table_->setItem(
      row, 4,
      new QTableWidgetItem(QString::number(
          readSnapshotNumber(record.snapshot, "velocity_rad_s"), 'f', 3)));
    motor_table_->setItem(
      row, 5,
      new QTableWidgetItem(record.last_error.isEmpty() ? uiText("status_ok") : record.last_error));
  }

  motor_select_->blockSignals(true);
  plot_motor_combo_->blockSignals(true);
  motor_select_->clear();
  plot_motor_combo_->clear();
  for (auto it = motors_.cbegin(); it != motors_.cend(); ++it) {
    const QString label = QString::number(it.key());
    motor_select_->addItem(label, it.key());
    plot_motor_combo_->addItem(label, it.key());
  }
  motor_select_->blockSignals(false);
  plot_motor_combo_->blockSignals(false);

  if (previous_motor > 0) {
    setSelectedMotorId(previous_motor);
  } else if (!motors_.isEmpty()) {
    setSelectedMotorId(motors_.firstKey());
  }
}

void HexmovrPanel::updateMotorDetails()
{
  const int motor_id = selectedMotorId();
  if (!motors_.contains(motor_id)) {
    motor_snapshot_view_->setPlainText(uiText("no_motor_selected"));
    return;
  }
  const auto & record = motors_.value(motor_id);
  QJsonObject detail_object = record.snapshot;
  detail_object.insert("last_seen", record.last_seen);
  detail_object.insert("last_error", record.last_error);
  motor_snapshot_view_->setPlainText(prettyJson(detail_object));
}

void HexmovrPanel::updateParamNameOptions()
{
  const auto group = currentParamGroup();
  const auto current_value = param_name_combo_->currentData().toString();
  param_name_combo_->blockSignals(true);
  param_name_combo_->clear();
  const auto names = group == "advanced" ? advancedParamNames() : controlParamNames();
  for (const auto & name : names) {
    param_name_combo_->addItem(paramDisplayName(name), name);
  }
  int index = param_name_combo_->findData(current_value);
  if (index < 0) {
    index = 0;
  }
  param_name_combo_->setCurrentIndex(index);
  param_name_combo_->blockSignals(false);
}

void HexmovrPanel::updateBatchParamNameOptions()
{
  const auto group = currentBatchParamGroup();
  const auto current_value = batch_param_name_combo_->currentData().toString();
  batch_param_name_combo_->blockSignals(true);
  batch_param_name_combo_->clear();
  const auto names = group == "advanced" ? advancedParamNames() : controlParamNames();
  for (const auto & name : names) {
    batch_param_name_combo_->addItem(paramDisplayName(name), name);
  }
  int index = batch_param_name_combo_->findData(current_value);
  if (index < 0) {
    index = 0;
  }
  batch_param_name_combo_->setCurrentIndex(index);
  batch_param_name_combo_->blockSignals(false);
}

void HexmovrPanel::refreshPlot()
{
  const int motor_id = plot_motor_combo_->currentData().toInt();
  const QString metric = currentPlotMetric();
  const QVector<QPointF> series = plot_history_[motor_id][metric];
  plot_widget_->setSeries(
    series, QString("%1 %2 | %3").arg(uiText("motor")).arg(motor_id).arg(metricDisplayName(metric)));
}

void HexmovrPanel::appendPlotSamples(int motor_id, const QJsonObject & snapshot)
{
  const double stamp = QDateTime::currentMSecsSinceEpoch() / 1000.0;
  const QStringList metrics = {
    "position_rad", "velocity_rad_s", "q_current_a",
    "temperature_c", "torque_nm", "bus_voltage_v"
  };
  for (const auto & metric : metrics) {
    auto & buffer = plot_history_[motor_id][metric];
    buffer.append(QPointF(stamp, readSnapshotNumber(snapshot, metric)));
    if (buffer.size() > kPlotHistoryLimit) {
      buffer.remove(0, buffer.size() - kPlotHistoryLimit);
    }
  }
}

std::vector<int> HexmovrPanel::currentBatchTargets() const
{
  if (batch_all_checkbox_->isChecked()) {
    std::vector<int> ids;
    ids.reserve(motors_.size());
    for (auto it = motors_.cbegin(); it != motors_.cend(); ++it) {
      ids.push_back(it.key());
    }
    return ids;
  }

  std::vector<int> ids;
  const auto parts = batch_ids_edit_->text().split(',', Qt::SkipEmptyParts);
  for (const auto & part : parts) {
    bool ok = false;
    const int motor_id = part.trimmed().toInt(&ok);
    if (ok) {
      ids.push_back(motor_id);
    }
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  return ids;
}

int HexmovrPanel::selectedMotorId() const
{
  return motor_select_->currentData().toInt();
}

void HexmovrPanel::setSelectedMotorId(int motor_id)
{
  const int combo_index = motor_select_->findData(motor_id);
  if (combo_index >= 0) {
    motor_select_->setCurrentIndex(combo_index);
  }
  for (int row = 0; row < motor_table_->rowCount(); ++row) {
    const auto * item = motor_table_->item(row, 0);
    if (item && item->text().toInt() == motor_id) {
      motor_table_->selectRow(row);
      break;
    }
  }
  const int plot_index = plot_motor_combo_->findData(motor_id);
  if (plot_index >= 0) {
    plot_motor_combo_->setCurrentIndex(plot_index);
  }
}

QString HexmovrPanel::currentPlotMetric() const
{
  return plot_metric_combo_->currentData().toString();
}

QString HexmovrPanel::currentBatchMode() const
{
  return batch_mode_combo_->currentData().toString();
}

QString HexmovrPanel::currentParamGroup() const
{
  return param_group_combo_->currentData().toString();
}

QString HexmovrPanel::currentBatchParamGroup() const
{
  return batch_param_group_combo_->currentData().toString();
}

double HexmovrPanel::readSnapshotNumber(const QJsonObject & snapshot, const QString & key) const
{
  return snapshot.value(key).toDouble();
}

QString HexmovrPanel::prettyJson(const QJsonObject & object) const
{
  return QString::fromUtf8(QJsonDocument(object).toJson(QJsonDocument::Indented));
}

void HexmovrPanel::applyLanguage()
{
  const auto set_widget_text = [this](QObject * object) {
      const auto key = object->property("ui_key").toString();
      if (key.isEmpty()) {
        return;
      }
      const auto value = uiText(key);
      if (auto * label = qobject_cast<QLabel *>(object)) {
        label->setText(value);
      } else if (auto * button = qobject_cast<QPushButton *>(object)) {
        button->setText(value);
      } else if (auto * box = qobject_cast<QGroupBox *>(object)) {
        box->setTitle(value);
      } else if (auto * check = qobject_cast<QCheckBox *>(object)) {
        check->setText(value);
      }
    };
  const auto set_form_label = [](QFormLayout * layout, QWidget * field, const QString & text) {
      if (!layout) {
        return;
      }
      if (auto * label = qobject_cast<QLabel *>(layout->labelForField(field))) {
        label->setText(text);
      }
    };

  for (QObject * object : findChildren<QObject *>()) {
    set_widget_text(object);
  }

  if (latest_event_.isEmpty()) {
    latest_event_label_->setText(uiText("no_events_yet"));
  }
  plot_widget_->setEmptyText(uiText("waiting_for_telemetry"));

  if (tabs_) {
    for (int i = 0; i < tabs_->count(); ++i) {
      const auto key = tabs_->widget(i)->property("ui_key").toString();
      if (!key.isEmpty()) {
        tabs_->setTabText(i, uiText(key));
      }
    }
  }

  batch_ids_edit_->setPlaceholderText(uiText("batch_ids_placeholder"));
  motor_table_->setHorizontalHeaderLabels(
    {uiText("id"), uiText("fault"), uiText("temp_c"), uiText("pos"), uiText("vel"), uiText("status")});
  fault_table_->setHorizontalHeaderLabels(
    {uiText("time"), uiText("motor"), uiText("kind"), uiText("message")});

  const auto reset_combo = [this](QComboBox * combo, const std::vector<std::pair<QString, QString>> & values) {
      const auto current_value = combo->currentData().toString();
      combo->blockSignals(true);
      combo->clear();
      for (const auto & pair : values) {
        combo->addItem(uiText(pair.first), pair.second);
      }
      const int index = combo->findData(current_value);
      combo->setCurrentIndex(index >= 0 ? index : 0);
      combo->blockSignals(false);
    };

  auto * control_layout = qobject_cast<QFormLayout *>(absolute_position_box_->parentWidget()->layout());
  set_form_label(control_layout, absolute_position_box_, uiText("absolute_position_rad"));
  set_form_label(control_layout, relative_position_box_, uiText("relative_position_rad"));
  set_form_label(control_layout, velocity_box_, uiText("velocity_rad_s"));
  set_form_label(control_layout, current_box_, uiText("current_a"));
  set_form_label(control_layout, mit_position_box_, uiText("mit_position_rad"));
  set_form_label(control_layout, mit_velocity_box_, uiText("mit_velocity_rad_s"));
  set_form_label(control_layout, mit_stiffness_box_, uiText("mit_stiffness"));
  set_form_label(control_layout, mit_damping_box_, uiText("mit_damping"));
  set_form_label(control_layout, mit_torque_box_, uiText("mit_torque_nm"));

  auto * param_layout = qobject_cast<QFormLayout *>(param_group_combo_->parentWidget()->layout());
  set_form_label(param_layout, param_group_combo_, uiText("group"));
  set_form_label(param_layout, param_name_combo_, uiText("name"));
  set_form_label(param_layout, param_value_box_, uiText("value"));

  auto * batch_target_layout = qobject_cast<QFormLayout *>(batch_ids_edit_->parentWidget()->layout());
  set_form_label(batch_target_layout, batch_ids_edit_, uiText("manual_ids"));

  auto * batch_control_layout = qobject_cast<QFormLayout *>(batch_mode_combo_->parentWidget()->layout());
  set_form_label(batch_control_layout, batch_mode_combo_, uiText("mode"));
  set_form_label(batch_control_layout, batch_value_box_, uiText("value"));

  auto * batch_param_layout = qobject_cast<QFormLayout *>(
    batch_param_group_combo_->parentWidget()->layout());
  set_form_label(batch_param_layout, batch_param_group_combo_, uiText("group"));
  set_form_label(batch_param_layout, batch_param_name_combo_, uiText("name"));
  set_form_label(batch_param_layout, batch_param_value_box_, uiText("value"));

  reset_combo(
    param_group_combo_,
    {{"group_control", "control"}, {"group_advanced", "advanced"}});
  reset_combo(
    batch_param_group_combo_,
    {{"group_control", "control"}, {"group_advanced", "advanced"}});
  reset_combo(
    batch_mode_combo_,
    {
      {"mode_absolute_position", "absolute_position"},
      {"mode_relative_position", "relative_position"},
      {"mode_velocity", "velocity"},
      {"mode_current", "current"},
    });
  reset_combo(
    plot_metric_combo_,
    {
      {"metric_position_rad", "position_rad"},
      {"metric_velocity_rad_s", "velocity_rad_s"},
      {"metric_q_current_a", "q_current_a"},
      {"metric_temperature_c", "temperature_c"},
      {"metric_torque_nm", "torque_nm"},
      {"metric_bus_voltage_v", "bus_voltage_v"},
    });

  updateParamNameOptions();
  updateBatchParamNameOptions();
  refreshPlot();
}

QString HexmovrPanel::uiText(const QString & key) const
{
  static const QMap<QString, QString> en = {
    {"language", "Language"},
    {"no_events_yet", "No events yet"},
    {"no_motor_selected", "No motor selected"},
    {"waiting_manager_state", "Waiting for manager state..."},
    {"waiting_for_telemetry", "Waiting for telemetry"},
    {"batch_ids_placeholder", "Example: 1,2,5"},
    {"tab_motors", "Motors"},
    {"tab_batch", "Batch"},
    {"tab_faults", "Faults"},
    {"tab_plots", "Plots"},
    {"scan", "Scan"},
    {"refresh_all", "Refresh All"},
    {"clear_history", "Clear History"},
    {"discovered_motors", "Discovered Motors"},
    {"selected_motor", "Selected Motor"},
    {"motor_actions", "Motor Actions"},
    {"refresh", "Refresh"},
    {"clear_error", "Clear Error"},
    {"set_zero", "Set Zero"},
    {"free_motor", "Free Motor"},
    {"return_to_zero", "Return To Zero"},
    {"brake_open", "Brake Open"},
    {"brake_close", "Brake Close"},
    {"control_forms", "Control Forms"},
    {"absolute_position_rad", "Absolute Position (rad)"},
    {"send_absolute_position", "Send Absolute Position"},
    {"relative_position_rad", "Relative Position (rad)"},
    {"send_relative_position", "Send Relative Position"},
    {"velocity_rad_s", "Velocity (rad/s)"},
    {"send_velocity", "Send Velocity"},
    {"current_a", "Current (A)"},
    {"send_current", "Send Current"},
    {"mit_position_rad", "MIT Position (rad)"},
    {"mit_velocity_rad_s", "MIT Velocity (rad/s)"},
    {"mit_stiffness", "MIT Stiffness"},
    {"mit_damping", "MIT Damping"},
    {"mit_torque_nm", "MIT Torque (Nm)"},
    {"send_mit_control", "Send MIT Control"},
    {"parameter_form", "Parameter Form"},
    {"group", "Group"},
    {"name", "Name"},
    {"value", "Value"},
    {"apply_parameter", "Apply Parameter"},
    {"telemetry_snapshot", "Telemetry Snapshot"},
    {"batch_targets", "Batch Targets"},
    {"use_all_discovered_motors", "Use all discovered motors"},
    {"manual_ids", "Manual IDs"},
    {"batch_actions", "Batch Actions"},
    {"clear_errors", "Clear Errors"},
    {"free_motors", "Free Motors"},
    {"batch_control", "Batch Control"},
    {"mode", "Mode"},
    {"apply_batch_control", "Apply Batch Control"},
    {"batch_parameter_form", "Batch Parameter Form"},
    {"apply_batch_parameter", "Apply Batch Parameter"},
    {"clear_visible_history", "Clear Visible History"},
    {"motor", "Motor"},
    {"metric", "Metric"},
    {"id", "ID"},
    {"fault", "Fault"},
    {"temp_c", "Temp C"},
    {"pos", "Pos"},
    {"vel", "Vel"},
    {"status", "Status"},
    {"status_ok", "OK"},
    {"time", "Time"},
    {"kind", "Kind"},
    {"message", "Message"},
    {"group_control", "Control"},
    {"group_advanced", "Advanced"},
    {"mode_absolute_position", "Absolute Position"},
    {"mode_relative_position", "Relative Position"},
    {"mode_velocity", "Velocity"},
    {"mode_current", "Current"},
    {"metric_position_rad", "Position (rad)"},
    {"metric_velocity_rad_s", "Velocity (rad/s)"},
    {"metric_q_current_a", "Q Current (A)"},
    {"metric_temperature_c", "Temperature (C)"},
    {"metric_torque_nm", "Torque (Nm)"},
    {"metric_bus_voltage_v", "Bus Voltage (V)"},
    {"param_position_max_speed", "Position Max Speed"},
    {"param_max_q_current", "Max Q Current"},
    {"param_current_slope", "Current Slope"},
    {"param_velocity_acceleration", "Velocity Acceleration"},
    {"param_position_kp", "Position Kp"},
    {"param_position_ki", "Position Ki"},
    {"param_velocity_kp", "Velocity Kp"},
    {"param_velocity_ki", "Velocity Ki"},
    {"param_trapezoid_acceleration", "Trapezoid Acceleration"},
    {"param_trapezoid_deceleration", "Trapezoid Deceleration"},
    {"param_position_filter_bandwidth", "Position Filter Bandwidth"},
    {"param_position_filter_inertia", "Position Filter Inertia"},
    {"param_position_filter_feedforward_current", "Position Filter Feedforward Current"},
    {"history_fault_active", "Fault Active"},
    {"history_fault_cleared", "Fault Cleared"},
    {"history_communication_error", "Communication Error"},
    {"history_communication_restored", "Communication Restored"},
    {"connected", "Connected"},
    {"disconnected", "Disconnected"},
    {"connected_detail", "Connected to %1 | %2 motor(s) | %3 history event(s)"},
    {"disconnected_detail", "Disconnected | %1"},
  };

  static const QMap<QString, QString> zh = {
    {"language", QString::fromUtf8("语言")},
    {"no_events_yet", QString::fromUtf8("暂时没有事件")},
    {"no_motor_selected", QString::fromUtf8("当前没有选中电机")},
    {"waiting_manager_state", QString::fromUtf8("等待管理节点状态...")},
    {"waiting_for_telemetry", QString::fromUtf8("等待遥测数据")},
    {"batch_ids_placeholder", QString::fromUtf8("例如: 1,2,5")},
    {"tab_motors", QString::fromUtf8("单电机")},
    {"tab_batch", QString::fromUtf8("批量操作")},
    {"tab_faults", QString::fromUtf8("故障历史")},
    {"tab_plots", QString::fromUtf8("曲线图")},
    {"scan", QString::fromUtf8("扫描")},
    {"refresh_all", QString::fromUtf8("全部刷新")},
    {"clear_history", QString::fromUtf8("清空历史")},
    {"discovered_motors", QString::fromUtf8("已识别电机")},
    {"selected_motor", QString::fromUtf8("当前电机")},
    {"motor_actions", QString::fromUtf8("电机动作")},
    {"refresh", QString::fromUtf8("刷新")},
    {"clear_error", QString::fromUtf8("清错")},
    {"set_zero", QString::fromUtf8("设零")},
    {"free_motor", QString::fromUtf8("释放电机")},
    {"return_to_zero", QString::fromUtf8("回零")},
    {"brake_open", QString::fromUtf8("打开抱闸")},
    {"brake_close", QString::fromUtf8("关闭抱闸")},
    {"control_forms", QString::fromUtf8("控制表单")},
    {"absolute_position_rad", QString::fromUtf8("绝对位置 (rad)")},
    {"send_absolute_position", QString::fromUtf8("发送绝对位置")},
    {"relative_position_rad", QString::fromUtf8("相对位置 (rad)")},
    {"send_relative_position", QString::fromUtf8("发送相对位置")},
    {"velocity_rad_s", QString::fromUtf8("速度 (rad/s)")},
    {"send_velocity", QString::fromUtf8("发送速度")},
    {"current_a", QString::fromUtf8("电流 (A)")},
    {"send_current", QString::fromUtf8("发送电流")},
    {"mit_position_rad", QString::fromUtf8("MIT 位置 (rad)")},
    {"mit_velocity_rad_s", QString::fromUtf8("MIT 速度 (rad/s)")},
    {"mit_stiffness", QString::fromUtf8("MIT 刚度")},
    {"mit_damping", QString::fromUtf8("MIT 阻尼")},
    {"mit_torque_nm", QString::fromUtf8("MIT 力矩 (Nm)")},
    {"send_mit_control", QString::fromUtf8("发送 MIT 控制")},
    {"parameter_form", QString::fromUtf8("参数表单")},
    {"group", QString::fromUtf8("分组")},
    {"name", QString::fromUtf8("名称")},
    {"value", QString::fromUtf8("数值")},
    {"apply_parameter", QString::fromUtf8("应用参数")},
    {"telemetry_snapshot", QString::fromUtf8("遥测快照")},
    {"batch_targets", QString::fromUtf8("批量目标")},
    {"use_all_discovered_motors", QString::fromUtf8("对全部已识别电机生效")},
    {"manual_ids", QString::fromUtf8("手动 ID")},
    {"batch_actions", QString::fromUtf8("批量动作")},
    {"clear_errors", QString::fromUtf8("批量清错")},
    {"free_motors", QString::fromUtf8("批量释放电机")},
    {"batch_control", QString::fromUtf8("批量控制")},
    {"mode", QString::fromUtf8("模式")},
    {"apply_batch_control", QString::fromUtf8("执行批量控制")},
    {"batch_parameter_form", QString::fromUtf8("批量参数表单")},
    {"apply_batch_parameter", QString::fromUtf8("执行批量参数")},
    {"clear_visible_history", QString::fromUtf8("清空当前历史表")},
    {"motor", QString::fromUtf8("电机")},
    {"metric", QString::fromUtf8("指标")},
    {"id", "ID"},
    {"fault", QString::fromUtf8("故障")},
    {"temp_c", QString::fromUtf8("温度 C")},
    {"pos", QString::fromUtf8("位置")},
    {"vel", QString::fromUtf8("速度")},
    {"status", QString::fromUtf8("状态")},
    {"status_ok", QString::fromUtf8("正常")},
    {"time", QString::fromUtf8("时间")},
    {"kind", QString::fromUtf8("类型")},
    {"message", QString::fromUtf8("消息")},
    {"group_control", QString::fromUtf8("控制参数")},
    {"group_advanced", QString::fromUtf8("高级参数")},
    {"mode_absolute_position", QString::fromUtf8("绝对位置")},
    {"mode_relative_position", QString::fromUtf8("相对位置")},
    {"mode_velocity", QString::fromUtf8("速度")},
    {"mode_current", QString::fromUtf8("电流")},
    {"metric_position_rad", QString::fromUtf8("位置 (rad)")},
    {"metric_velocity_rad_s", QString::fromUtf8("速度 (rad/s)")},
    {"metric_q_current_a", QString::fromUtf8("Q轴电流 (A)")},
    {"metric_temperature_c", QString::fromUtf8("温度 (C)")},
    {"metric_torque_nm", QString::fromUtf8("力矩 (Nm)")},
    {"metric_bus_voltage_v", QString::fromUtf8("母线电压 (V)")},
    {"param_position_max_speed", QString::fromUtf8("位置最大速度")},
    {"param_max_q_current", QString::fromUtf8("最大Q轴电流")},
    {"param_current_slope", QString::fromUtf8("电流斜率")},
    {"param_velocity_acceleration", QString::fromUtf8("速度加速度")},
    {"param_position_kp", QString::fromUtf8("位置 Kp")},
    {"param_position_ki", QString::fromUtf8("位置 Ki")},
    {"param_velocity_kp", QString::fromUtf8("速度 Kp")},
    {"param_velocity_ki", QString::fromUtf8("速度 Ki")},
    {"param_trapezoid_acceleration", QString::fromUtf8("梯形加速度")},
    {"param_trapezoid_deceleration", QString::fromUtf8("梯形减速度")},
    {"param_position_filter_bandwidth", QString::fromUtf8("位置滤波带宽")},
    {"param_position_filter_inertia", QString::fromUtf8("位置滤波惯量")},
    {"param_position_filter_feedforward_current", QString::fromUtf8("位置滤波前馈电流")},
    {"history_fault_active", QString::fromUtf8("故障激活")},
    {"history_fault_cleared", QString::fromUtf8("故障清除")},
    {"history_communication_error", QString::fromUtf8("通信异常")},
    {"history_communication_restored", QString::fromUtf8("通信恢复")},
    {"connected", QString::fromUtf8("已连接")},
    {"disconnected", QString::fromUtf8("未连接")},
    {"connected_detail", QString::fromUtf8("已连接到 %1 | %2 台电机 | %3 条历史记录")},
    {"disconnected_detail", QString::fromUtf8("未连接 | %1")},
  };

  const auto & table = language_ == DisplayLanguage::Chinese ? zh : en;
  return table.value(key, key);
}

QString HexmovrPanel::connectionDetailText(bool connected, const QString & detail) const
{
  if (connected) {
    const auto parts = detail.split('|');
    if (parts.size() == 3) {
      return uiText("connected_detail").arg(parts[0]).arg(parts[1]).arg(parts[2]);
    }
  }
  const QString message = detail.isEmpty() ? uiText("waiting_manager_state") : detail;
  return uiText("disconnected_detail").arg(message);
}

QString HexmovrPanel::metricDisplayName(const QString & metric_key) const
{
  return uiText(QString("metric_%1").arg(metric_key));
}

QString HexmovrPanel::paramDisplayName(const QString & param_key) const
{
  return uiText(QString("param_%1").arg(param_key));
}

QString HexmovrPanel::historyKindDisplayName(const QString & kind_key) const
{
  return uiText(QString("history_%1").arg(kind_key));
}

}  // namespace hexmovr_moto_panel

PLUGINLIB_EXPORT_CLASS(hexmovr_moto_panel::HexmovrPanel, rviz_common::Panel)
