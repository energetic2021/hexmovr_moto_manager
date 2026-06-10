#pragma once

#include <map>
#include <string>
#include <vector>

#include <QJsonArray>
#include <QJsonObject>
#include <QLocale>
#include <QMap>
#include <QPointF>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QPlainTextEdit;
class QTableWidget;
class QTabWidget;
class QTimer;

namespace hexmovr_moto_panel
{

enum class DisplayLanguage
{
  English,
  Chinese,
};

class PlotWidget : public QWidget
{
public:
  explicit PlotWidget(QWidget * parent = nullptr);

  void setSeries(const QVector<QPointF> & points, const QString & title);
  void setEmptyText(const QString & text);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  QVector<QPointF> points_;
  QString title_;
  QString empty_text_;
};

struct MotorRecord
{
  int motor_id = 0;
  double last_seen = 0.0;
  QString last_error;
  QJsonObject snapshot;
};

class HexmovrPanel : public rviz_common::Panel
{
public:
  explicit HexmovrPanel(QWidget * parent = nullptr);
  ~HexmovrPanel() override = default;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  void buildUi();
  void setupRos();
  void publishCommand(const QJsonObject & command);
  void handleStateMessage(const std_msgs::msg::String::SharedPtr msg);
  void handleEventMessage(const std_msgs::msg::String::SharedPtr msg);
  void handleHistoryMessage(const std_msgs::msg::String::SharedPtr msg);
  void updateConnectionBanner(bool connected, const QString & detail);
  void updateMotorWidgets();
  void updateMotorDetails();
  void updateParamNameOptions();
  void updateBatchParamNameOptions();
  void refreshPlot();
  void appendPlotSamples(int motor_id, const QJsonObject & snapshot);
  void applyLanguage();
  QString uiText(const QString & key) const;
  QString connectionDetailText(bool connected, const QString & detail) const;
  QString metricDisplayName(const QString & metric_key) const;
  QString paramDisplayName(const QString & param_key) const;
  QString historyKindDisplayName(const QString & kind_key) const;
  std::vector<int> currentBatchTargets() const;
  int selectedMotorId() const;
  void setSelectedMotorId(int motor_id);
  QString currentPlotMetric() const;
  QString currentBatchMode() const;
  QString currentParamGroup() const;
  QString currentBatchParamGroup() const;
  double readSnapshotNumber(const QJsonObject & snapshot, const QString & key) const;
  QString prettyJson(const QJsonObject & object) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr history_sub_;

  QTimer * spin_timer_ = nullptr;
  QLabel * connection_label_ = nullptr;
  QLabel * latest_event_label_ = nullptr;
  QLabel * language_label_ = nullptr;
  QComboBox * language_combo_ = nullptr;
  QTableWidget * motor_table_ = nullptr;
  QComboBox * motor_select_ = nullptr;
  QPlainTextEdit * motor_snapshot_view_ = nullptr;
  QDoubleSpinBox * absolute_position_box_ = nullptr;
  QDoubleSpinBox * relative_position_box_ = nullptr;
  QDoubleSpinBox * velocity_box_ = nullptr;
  QDoubleSpinBox * current_box_ = nullptr;
  QDoubleSpinBox * mit_position_box_ = nullptr;
  QDoubleSpinBox * mit_velocity_box_ = nullptr;
  QDoubleSpinBox * mit_stiffness_box_ = nullptr;
  QDoubleSpinBox * mit_damping_box_ = nullptr;
  QDoubleSpinBox * mit_torque_box_ = nullptr;
  QComboBox * param_group_combo_ = nullptr;
  QComboBox * param_name_combo_ = nullptr;
  QDoubleSpinBox * param_value_box_ = nullptr;
  QCheckBox * batch_all_checkbox_ = nullptr;
  QLineEdit * batch_ids_edit_ = nullptr;
  QComboBox * batch_mode_combo_ = nullptr;
  QDoubleSpinBox * batch_value_box_ = nullptr;
  QComboBox * batch_param_group_combo_ = nullptr;
  QComboBox * batch_param_name_combo_ = nullptr;
  QDoubleSpinBox * batch_param_value_box_ = nullptr;
  QTableWidget * fault_table_ = nullptr;
  QComboBox * plot_motor_combo_ = nullptr;
  QComboBox * plot_metric_combo_ = nullptr;
  PlotWidget * plot_widget_ = nullptr;
  QTabWidget * tabs_ = nullptr;

  bool connected_ = false;
  QString transport_error_;
  QString can_interface_;
  QString latest_event_;
  DisplayLanguage language_ = DisplayLanguage::English;
  int manager_history_size_ = 0;
  QMap<int, MotorRecord> motors_;
  QMap<int, QMap<QString, QVector<QPointF>>> plot_history_;
  QJsonArray fault_history_;
};

}  // namespace hexmovr_moto_panel
