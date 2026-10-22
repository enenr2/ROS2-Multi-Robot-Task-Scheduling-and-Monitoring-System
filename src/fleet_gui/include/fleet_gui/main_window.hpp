// 监控端主窗口：任务表、车辆状态、告警、日志、地图与相机，外加 rosbag 时间轴回放。
#pragma once

#include <QMainWindow>
#include <QString>

#include <cstdint>
#include <deque>
#include <vector>

#include "fleet_gui/bag_replay.hpp"
#include "fleet_gui/monitor_core.hpp"
#include "fleet_gui/qt_metatypes.hpp"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QListWidget;
class QPushButton;
class QSlider;
class QTableWidget;
class QTabWidget;
class QTextEdit;
class QTimer;
class QWidget;

namespace fleet::gui {

class CameraView;
class MapView;
class MonitorNode;
class SignalHub;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(MonitorNode* node, SignalHub* hub, QWidget* parent = nullptr);
  ~MainWindow() override;

  bool OpenBag(const QString& uri);
  void SelectCamera(const QString& robot);
  // 按名字切页：任务 / 车辆 / 告警 / 日志（kiosk 或多屏时固定到某页）
  void SelectTab(const QString& name);

private slots:
  void OnTaskSnapshot(const fleet::monitor::TaskView& task);
  void OnRobotStatus(const fleet::monitor::RobotView& robot);
  void OnTaskStatus(const fleet::monitor::StatusEvent& status);
  void OnMap(const fleet::monitor::MapData& map);
  void OnCameraFrame(const QString& robot, const QImage& image);
  void OnNotice(const QString& text, int severity);
  void OnRobotDiscovered(const QString& robot);
  void OnTick();

private:
  const fleet::monitor::MonitorModel& Model() const { return replay_active_ ? replay_ : live_; }

  void BuildUi();
  void ApplyLive(const std::vector<fleet::monitor::Event>& events);
  void AppendLiveEvent(const fleet::monitor::Event& event);
  void RefreshAll();
  void RefreshTables();
  void RefreshHeader();
  void RefreshMap();
  void RenderLog();
  void RenderAlarms();
  void SubmitFromFields();
  void CancelSelectedTask();
  void TogglePickMode(bool on);

  void EnterReplay(const ReplayData& data, const QString& uri);
  void ExitReplay();
  void ApplyReplayAt(int64_t stamp_ms);
  void UpdateReplayBar();
  void AdvanceReplay();

  MonitorNode* node_;
  SignalHub* hub_;

  fleet::monitor::MonitorModel live_;
  fleet::monitor::MonitorModel replay_;
  ReplayData bag_;
  bool replay_active_{false};
  bool replay_playing_{false};
  int64_t replay_pos_ms_{0};
  int64_t replay_last_render_ms_{0};
  int replay_speed_{1};
  qint64 last_play_tick_ms_{0};

  std::deque<fleet::monitor::Event> live_events_;
  std::vector<fleet::monitor::Event> replay_events_;
  QString selected_task_;

  MapView* map_view_{nullptr};
  CameraView* camera_view_{nullptr};
  QComboBox* camera_combo_{nullptr};
  QComboBox* color_combo_{nullptr};
  QDoubleSpinBox* x_spin_{nullptr};
  QDoubleSpinBox* y_spin_{nullptr};
  QPushButton* pick_button_{nullptr};
  QPushButton* cancel_button_{nullptr};
  QTableWidget* task_table_{nullptr};
  QTableWidget* robot_table_{nullptr};
  QTabWidget* tabs_{nullptr};
  QListWidget* alarm_list_{nullptr};
  QTextEdit* log_view_{nullptr};

  QLabel* status_robots_{nullptr};
  QLabel* status_tasks_{nullptr};
  QLabel* status_alarm_{nullptr};
  QLabel* status_mode_{nullptr};

  QWidget* replay_bar_{nullptr};
  QPushButton* replay_play_{nullptr};
  QSlider* replay_slider_{nullptr};
  QComboBox* replay_speed_combo_{nullptr};
  QLabel* replay_time_{nullptr};

  QTimer* tick_timer_{nullptr};
  QTimer* replay_timer_{nullptr};
};

}  // namespace fleet::gui