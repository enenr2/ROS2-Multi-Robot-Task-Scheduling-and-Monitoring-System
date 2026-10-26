#include "fleet_gui/main_window.hpp"

#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFont>
#include <QFontDatabase>
#include <QFrame>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSlider>
#include <QSplitter>
#include <QStatusBar>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabWidget>
#include <QTextCursor>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#include "fleet_gui/camera_view.hpp"
#include "fleet_gui/map_view.hpp"
#include "fleet_gui/ros_bridge.hpp"

namespace fleet::gui {

namespace {

const QColor kText(199, 206, 216);
const QColor kDimText(138, 147, 158);
const QColor kWarnColor(255, 191, 105);
const QColor kErrorColor(255, 107, 99);
const QColor kAccent(124, 196, 255);
const QColor kDoneColor(89, 217, 140);

QColor SeverityColor(fleet::monitor::Severity severity)
{
  switch (severity) {
    case fleet::monitor::Severity::kError: return kErrorColor;
    case fleet::monitor::Severity::kWarn: return kWarnColor;
    default: return kText;
  }
}

QColor TaskStateColor(int state)
{
  switch (state) {
    case fleet::monitor::kStateDone: return kDoneColor;
    case fleet::monitor::kStateFailed: return kErrorColor;
    case fleet::monitor::kStateCanceled: return QColor(150, 156, 166);
    case fleet::monitor::kStatePending: return QColor(190, 197, 206);
    default: return kAccent;
  }
}

QColor RobotStateColor(int state, bool online)
{
  if (!online || state == fleet::monitor::kRobotOffline) {
    return QColor(150, 156, 166);
  }
  switch (state) {
    case fleet::monitor::kRobotBusy: return kAccent;
    case fleet::monitor::kRobotError: return kErrorColor;
    default: return kDoneColor;
  }
}

QTableWidget* MakeTable(const QStringList& headers, QWidget* parent)
{
  auto* table = new QTableWidget(0, headers.size(), parent);
  table->setHorizontalHeaderLabels(headers);
  table->verticalHeader()->setVisible(false);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setAlternatingRowColors(true);
  table->setShowGrid(false);
  table->horizontalHeader()->setStretchLastSection(true);
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table->verticalHeader()->setDefaultSectionSize(24);
  return table;
}

void UpdateItem(QTableWidget* table, int row, int column, const QString& text,
                const QColor& color, const QString& tooltip = {})
{
  auto* item = table->item(row, column);
  if (item == nullptr) {
    item = new QTableWidgetItem();
    table->setItem(row, column, item);
  }
  if (item->text() != text) {
    item->setText(text);
  }
  if (item->foreground().color() != color) {
    item->setForeground(color);
  }
  if (item->toolTip() != tooltip) {
    item->setToolTip(tooltip);
  }
}

QString Elide(const QString& text, int max_chars)
{
  if (text.size() <= max_chars) {
    return text;
  }
  return text.left(max_chars - 1) + QStringLiteral("…");
}

QString FormatDuration(int64_t ms)
{
  const int64_t total_s = std::max<int64_t>(0, ms / 1000);
  return QStringLiteral("%1:%2")
    .arg(total_s / 60, 2, 10, QLatin1Char('0'))
    .arg(total_s % 60, 2, 10, QLatin1Char('0'));
}

}  // namespace

MainWindow::MainWindow(MonitorNode* node, SignalHub* hub, QWidget* parent)
: QMainWindow(parent), node_(node), hub_(hub)
{
  BuildUi();

  connect(hub_, &SignalHub::taskSnapshot, this, &MainWindow::OnTaskSnapshot);
  connect(hub_, &SignalHub::robotStatus, this, &MainWindow::OnRobotStatus);
  connect(hub_, &SignalHub::taskStatus, this, &MainWindow::OnTaskStatus);
  connect(hub_, &SignalHub::mapData, this, &MainWindow::OnMap);
  connect(hub_, &SignalHub::cameraFrame, this, &MainWindow::OnCameraFrame);
  connect(hub_, &SignalHub::notice, this, &MainWindow::OnNotice);
  connect(hub_, &SignalHub::robotDiscovered, this, &MainWindow::OnRobotDiscovered);

  tick_timer_ = new QTimer(this);
  tick_timer_->setInterval(500);
  connect(tick_timer_, &QTimer::timeout, this, &MainWindow::OnTick);
  tick_timer_->start();

  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(100);
  refresh_timer_->setSingleShot(true);
  connect(refresh_timer_, &QTimer::timeout, this, [this]() {
    refresh_pending_ = false;
    RefreshAll();
  });

  replay_timer_ = new QTimer(this);
  replay_timer_->setInterval(100);
  connect(replay_timer_, &QTimer::timeout, this, &MainWindow::AdvanceReplay);
  replay_timer_->start();

  SelectCamera(QStringLiteral("robot1"));
  RefreshAll();
  RenderLog();
  RenderAlarms();
}

MainWindow::~MainWindow() = default;

void MainWindow::BuildUi()
{
  setWindowTitle(QStringLiteral("Fleet Monitor · 多机调度监控台"));
  resize(1440, 880);

  auto* central = new QWidget(this);
  auto* root = new QVBoxLayout(central);
  root->setContentsMargins(8, 8, 8, 6);
  root->setSpacing(6);

  // ---- 顶部操作条 ----
  auto* control = new QWidget(central);
  control->setObjectName(QStringLiteral("controlPanel"));
  auto* row = new QHBoxLayout(control);
  row->setContentsMargins(10, 6, 10, 6);
  row->setSpacing(8);

  row->addWidget(new QLabel(QStringLiteral("相机"), control));
  camera_combo_ = new QComboBox(control);
  camera_combo_->setMinimumWidth(96);
  camera_combo_->addItem(QStringLiteral("robot1"));
  row->addWidget(camera_combo_);

  auto* sep1 = new QFrame(control);
  sep1->setFrameShape(QFrame::VLine);
  sep1->setObjectName(QStringLiteral("separator"));
  row->addWidget(sep1);

  row->addWidget(new QLabel(QStringLiteral("注入任务"), control));
  color_combo_ = new QComboBox(control);
  color_combo_->addItem(QStringLiteral("红"), fleet::monitor::kColorRed);
  color_combo_->addItem(QStringLiteral("绿"), fleet::monitor::kColorGreen);
  color_combo_->addItem(QStringLiteral("蓝"), fleet::monitor::kColorBlue);
  row->addWidget(color_combo_);

  row->addWidget(new QLabel(QStringLiteral("X"), control));
  x_spin_ = new QDoubleSpinBox(control);
  x_spin_->setRange(-12.0, 12.0);
  x_spin_->setDecimals(2);
  x_spin_->setSingleStep(0.1);
  x_spin_->setSuffix(QStringLiteral(" m"));
  x_spin_->setValue(3.0);
  row->addWidget(x_spin_);

  row->addWidget(new QLabel(QStringLiteral("Y"), control));
  y_spin_ = new QDoubleSpinBox(control);
  y_spin_->setRange(-12.0, 12.0);
  y_spin_->setDecimals(2);
  y_spin_->setSingleStep(0.1);
  y_spin_->setSuffix(QStringLiteral(" m"));
  y_spin_->setValue(0.0);
  row->addWidget(y_spin_);

  pick_button_ = new QPushButton(QStringLiteral("地图选点"), control);
  pick_button_->setCheckable(true);
  row->addWidget(pick_button_);

  auto* submit = new QPushButton(QStringLiteral("下发"), control);
  submit->setObjectName(QStringLiteral("primaryButton"));
  row->addWidget(submit);

  cancel_button_ = new QPushButton(QStringLiteral("取消选中任务"), control);
  row->addWidget(cancel_button_);

  row->addStretch(1);

  auto* replay_open = new QPushButton(QStringLiteral("打开 rosbag…"), control);
  row->addWidget(replay_open);

  root->addWidget(control);

  // ---- 主体：左地图+相机，右四页 ----
  auto* splitter = new QSplitter(Qt::Horizontal, central);
  auto* left = new QSplitter(Qt::Vertical, splitter);
  map_view_ = new MapView(left);
  camera_view_ = new CameraView(left);
  left->addWidget(map_view_);
  left->addWidget(camera_view_);
  left->setStretchFactor(0, 3);
  left->setStretchFactor(1, 2);
  left->setSizes({520, 320});

  auto* tabs = new QTabWidget(splitter);
  tabs_ = tabs;
  task_table_ = MakeTable(
    {QStringLiteral("任务"), QStringLiteral("目标"), QStringLiteral("状态"), QStringLiteral("车辆"),
     QStringLiteral("进度"), QStringLiteral("重试"), QStringLiteral("失败原因"),
     QStringLiteral("检测时间")},
    tabs);
  robot_table_ = MakeTable(
    {QStringLiteral("车辆"), QStringLiteral("状态"), QStringLiteral("位姿"),
     QStringLiteral("当前任务"), QStringLiteral("完成"), QStringLiteral("失败"),
     QStringLiteral("取消"), QStringLiteral("心跳")},
    tabs);

  alarm_list_ = new QListWidget(tabs);
  log_view_ = new QTextEdit(tabs);
  log_view_->setReadOnly(true);
  log_view_->document()->setMaximumBlockCount(2000);
  QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  mono.setPointSizeF(9.5);
  log_view_->setFont(mono);

  tabs->addTab(task_table_, QStringLiteral("任务"));
  tabs->addTab(robot_table_, QStringLiteral("车辆"));
  tabs->addTab(alarm_list_, QStringLiteral("告警"));
  tabs->addTab(log_view_, QStringLiteral("日志"));

  splitter->addWidget(left);
  splitter->addWidget(tabs);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);
  splitter->setSizes({780, 620});
  root->addWidget(splitter, 1);

  // ---- 回放控制条 ----
  replay_bar_ = new QWidget(central);
  replay_bar_->setObjectName(QStringLiteral("replayBar"));
  auto* replay_row = new QHBoxLayout(replay_bar_);
  replay_row->setContentsMargins(10, 4, 10, 4);
  replay_row->setSpacing(8);
  replay_play_ = new QPushButton(QStringLiteral("暂停"), replay_bar_);
  replay_play_->setFixedWidth(72);
  replay_slider_ = new QSlider(Qt::Horizontal, replay_bar_);
  replay_slider_->setRange(0, 1000);
  replay_time_ = new QLabel(QStringLiteral("00:00 / 00:00"), replay_bar_);
  replay_time_->setMinimumWidth(110);
  replay_speed_combo_ = new QComboBox(replay_bar_);
  for (int speed : {1, 2, 4, 8}) {
    replay_speed_combo_->addItem(QStringLiteral("%1×").arg(speed), speed);
  }
  auto* replay_exit = new QPushButton(QStringLiteral("退出回放"), replay_bar_);

  replay_row->addWidget(replay_play_);
  replay_row->addWidget(replay_slider_, 1);
  replay_row->addWidget(replay_time_);
  replay_row->addWidget(replay_speed_combo_);
  replay_row->addWidget(replay_exit);
  replay_bar_->setVisible(false);
  root->addWidget(replay_bar_);

  setCentralWidget(central);

  status_robots_ = new QLabel(QStringLiteral("车辆 0/0 在线"), this);
  status_tasks_ = new QLabel(QStringLiteral("任务 0 进行中 · 0 完成 · 0 失败"), this);
  status_alarm_ = new QLabel(QStringLiteral("告警 0"), this);
  status_mode_ = new QLabel(QStringLiteral("实时"), this);
  status_robots_->setStyleSheet(QStringLiteral("color:#c7ced8;"));
  status_tasks_->setStyleSheet(QStringLiteral("color:#c7ced8;"));
  status_alarm_->setStyleSheet(QStringLiteral("color:#c7ced8;"));
  status_mode_->setStyleSheet(QStringLiteral("color:#7cc4ff;"));
  statusBar()->addWidget(status_robots_);
  statusBar()->addWidget(new QLabel(QStringLiteral("  |  "), this));
  statusBar()->addWidget(status_tasks_);
  statusBar()->addWidget(new QLabel(QStringLiteral("  |  "), this));
  statusBar()->addWidget(status_alarm_);
  statusBar()->addPermanentWidget(status_mode_);

  setStyleSheet(QStringLiteral(R"(
    QMainWindow, QWidget { background: #14171c; color: #c7ced8; }
    #controlPanel, #replayBar {
      background: #1b1f25;
      border: 1px solid #262c34;
      border-radius: 4px;
    }
    #separator { color: #2a3038; }
    QLabel { background: transparent; }
    QPushButton {
      background: #232a33; border: 1px solid #313a45;
      padding: 4px 12px; border-radius: 3px;
    }
    QPushButton:hover { background: #2b3440; }
    QPushButton:pressed { background: #1f2630; }
    QPushButton:checked { background: #1d4ed8; border-color: #3b82f6; color: #ffffff; }
    #primaryButton { background: #1d4ed8; border-color: #3b82f6; color: #ffffff; }
    #primaryButton:hover { background: #2563eb; }
    QComboBox, QDoubleSpinBox {
      background: #1b1f25; border: 1px solid #2a3038;
      border-radius: 3px; padding: 2px 6px;
    }
    QComboBox QAbstractItemView { background: #1b1f25; selection-background-color: #2d3a47; }
    QTabWidget::pane { border: 1px solid #262c34; top: -1px; }
    QTabBar::tab {
      background: #1a1e24; border: 1px solid #262c34;
      padding: 6px 16px; margin-right: 2px;
      border-top-left-radius: 3px; border-top-right-radius: 3px;
    }
    QTabBar::tab:selected { background: #232a33; color: #ffffff; }
    QTableWidget {
      background: #171a1f; alternate-background-color: #1b1f25;
      gridline-color: #262c34; border: 0;
      selection-background-color: #2d3a47; selection-color: #ffffff;
    }
    QHeaderView::section {
      background: #1e2329; color: #93a0ae; border: 0;
      padding: 5px 6px; border-bottom: 1px solid #262c34;
    }
    QListWidget { background: #171a1f; border: 0; }
    QTextEdit { background: #171a1f; border: 0; }
    QSlider::groove:horizontal {
      height: 4px; background: #2a3038; border-radius: 2px;
    }
    QSlider::handle:horizontal {
      width: 12px; margin: -5px 0; border-radius: 6px; background: #7cc4ff;
    }
    QStatusBar { background: #101317; border-top: 1px solid #262c34; }
    QStatusBar QLabel { color: #93a0ae; }
  )"));

  connect(camera_combo_, &QComboBox::currentTextChanged, this, &MainWindow::SelectCamera);
  connect(pick_button_, &QPushButton::toggled, this, &MainWindow::TogglePickMode);
  connect(submit, &QPushButton::clicked, this, &MainWindow::SubmitFromFields);
  connect(cancel_button_, &QPushButton::clicked, this, &MainWindow::CancelSelectedTask);
  connect(replay_open, &QPushButton::clicked, this, [this]() {
    const QString uri = QFileDialog::getExistingDirectory(
      this, QStringLiteral("选择 rosbag2 目录"), QString(),
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!uri.isEmpty()) {
      OpenBag(uri);
    }
  });
  connect(replay_play_, &QPushButton::clicked, this, [this]() {
    replay_playing_ = !replay_playing_;
    last_play_tick_ms_ = 0;
    replay_play_->setText(replay_playing_ ? QStringLiteral("暂停") : QStringLiteral("播放"));
  });
  connect(replay_slider_, &QSlider::valueChanged, this, [this](int value) {
    if (replay_active_) {
      ApplyReplayAt(bag_.start_ms + value);
    }
  });
  connect(replay_speed_combo_, &QComboBox::currentIndexChanged, this, [this](int index) {
    replay_speed_ = replay_speed_combo_->itemData(index).toInt();
  });
  connect(replay_exit, &QPushButton::clicked, this, &MainWindow::ExitReplay);

  connect(map_view_, &MapView::mapClicked, this, [this](double x, double y) {
    x_spin_->setValue(x);
    y_spin_->setValue(y);
    pick_button_->setChecked(false);
    statusBar()->showMessage(QStringLiteral("目标点已选 (%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2),
                             3000);
  });
  connect(task_table_, &QTableWidget::itemSelectionChanged, this, [this]() {
    const auto rows = task_table_->selectionModel()->selectedRows();
    if (rows.isEmpty()) {
      selected_task_.clear();
    } else {
      const auto* item = task_table_->item(rows.first().row(), 0);
      selected_task_ = item != nullptr ? item->text() : QString();
    }
    map_view_->SetSelectedTask(selected_task_);
  });
}

// ---------------- 实时数据 ----------------

void MainWindow::OnTaskSnapshot(const fleet::monitor::TaskView& task)
{
  ApplyLive(live_.OnTaskSnapshot(task, QDateTime::currentMSecsSinceEpoch()));
  ScheduleRefresh();
}

void MainWindow::OnRobotStatus(const fleet::monitor::RobotView& robot)
{
  ApplyLive(live_.OnRobotStatus(robot, QDateTime::currentMSecsSinceEpoch()));
  ScheduleRefresh();
}

void MainWindow::OnTaskStatus(const fleet::monitor::StatusEvent& status)
{
  ApplyLive(live_.OnTaskStatus(status, QDateTime::currentMSecsSinceEpoch()));
  ScheduleRefresh();
}

void MainWindow::OnMap(const fleet::monitor::MapData& map)
{
  map_view_->SetMap(map);
}

void MainWindow::OnCameraFrame(const QString& robot, const QImage& image)
{
  if (!replay_active_ && robot == camera_combo_->currentText()) {
    camera_view_->SetFrame(image);
  }
}

void MainWindow::OnNotice(const QString& text, int severity)
{
  fleet::monitor::Event event;
  event.stamp_ms = QDateTime::currentMSecsSinceEpoch();
  event.text = text.toStdString();
  event.severity = severity >= 2 ? fleet::monitor::Severity::kError
                                 : (severity == 1 ? fleet::monitor::Severity::kWarn
                                                  : fleet::monitor::Severity::kInfo);
  AppendLiveEvent(event);
  if (!replay_active_) {
    statusBar()->showMessage(text, 4000);
  }
}

void MainWindow::OnRobotDiscovered(const QString& robot)
{
  if (camera_combo_->findText(robot) < 0) {
    camera_combo_->addItem(robot);
  }
  // 先登记成"离线占位"：agent 没起来时也要在车辆表/地图上留一行
  ApplyLive(live_.EnsureRobot(robot.toStdString(), QDateTime::currentMSecsSinceEpoch()));
  ScheduleRefresh();
}

void MainWindow::OnTick()
{
  if (replay_active_) {
    return;
  }
  const int64_t now = QDateTime::currentMSecsSinceEpoch();
  ApplyLive(live_.Sweep(now));
  // 只保留最近的终态任务：快照会一直重发历史任务，界面不能无限增长
  live_.PruneTasks(now);
  ScheduleRefresh();
}

void MainWindow::ScheduleRefresh()
{
  if (replay_active_ || refresh_pending_ || refresh_timer_ == nullptr) {
    return;
  }
  refresh_pending_ = true;
  refresh_timer_->start();
}

void MainWindow::ApplyLive(const std::vector<fleet::monitor::Event>& events)
{
  for (const auto& event : events) {
    AppendLiveEvent(event);
  }
}

void MainWindow::AppendLiveEvent(const fleet::monitor::Event& event)
{
  live_events_.push_back(event);
  while (live_events_.size() > 500) {
    live_events_.pop_front();
  }
  if (replay_active_) {
    return;   // 回放期间只缓存，退出回放时整体重绘
  }

  QString line = QStringLiteral("<span style=\"color:%1\">[%2] %3</span>")
                   .arg(SeverityColor(event.severity).name())
                   .arg(QString::fromStdString(fleet::monitor::FormatClock(event.stamp_ms)))
                   .arg(QString::fromStdString(event.text).toHtmlEscaped());
  log_view_->append(line);

  if (event.severity != fleet::monitor::Severity::kInfo) {
    auto* item = new QListWidgetItem(
      QStringLiteral("[%1] %2")
        .arg(QString::fromStdString(fleet::monitor::FormatClock(event.stamp_ms)))
        .arg(QString::fromStdString(event.text)));
    item->setForeground(SeverityColor(event.severity));
    alarm_list_->insertItem(0, item);
    while (alarm_list_->count() > 200) {
      delete alarm_list_->takeItem(alarm_list_->count() - 1);
    }
  }
}

// ---------------- 刷新 ----------------

void MainWindow::RefreshAll()
{
  RefreshTables();
  RefreshHeader();
  RefreshMap();
}

void MainWindow::RefreshTables()
{
  const auto& model = Model();

  // ---- 任务表 ----
  {
    const QSignalBlocker blocker(task_table_);
    task_table_->setUpdatesEnabled(false);
    const int scroll = task_table_->verticalScrollBar()->value();
    const auto tasks = model.SortedTasks();
    task_table_->setRowCount(static_cast<int>(tasks.size()));

    int selected_row = -1;
    for (int row = 0; row < static_cast<int>(tasks.size()); ++row) {
      const auto& task = tasks[row];
      const QString id = QString::fromStdString(task.task_id);
      const int state = task.state;
      const double progress = state == fleet::monitor::kStateDone
                                ? 1.0 : model.Progress(task.task_id);

      UpdateItem(task_table_, row, 0, id, kText);
      UpdateItem(task_table_, row, 1,
                 QString::fromStdString(fleet::monitor::ColorName(task.color)), kText);
      const QString detail = QString::fromStdString(model.Detail(task.task_id));
      UpdateItem(task_table_, row, 2,
                 QString::fromStdString(fleet::monitor::StateName(state)), TaskStateColor(state),
                 detail.isEmpty() ? QStringLiteral("（无详情）") : detail);
      UpdateItem(task_table_, row, 3,
                 task.robot.empty() ? QStringLiteral("-") : QString::fromStdString(task.robot),
                 task.robot.empty() ? kDimText : kText);
      UpdateItem(task_table_, row, 4,
                 QStringLiteral("%1%").arg(static_cast<int>(std::lround(progress * 100.0))),
                 progress > 0.05 ? kAccent : kDimText);

      const QString retry = task.max_retry > 0
                              ? QStringLiteral("%1/%2").arg(task.retry).arg(task.max_retry)
                              : QString::number(task.retry);
      UpdateItem(task_table_, row, 5, retry, task.retry > 0 ? kWarnColor : kDimText);

      const QString reason = QString::fromStdString(fleet::monitor::ReasonText(task.failure_reason));
      UpdateItem(task_table_, row, 6, Elide(reason, 18),
                 task.failure_reason.empty() || task.failure_reason == "NONE" ? kDimText
                                                                               : kErrorColor);

      const QString detected = task.first_seen_ms > 0
                                 ? QString::fromStdString(
                                     fleet::monitor::FormatClock(task.first_seen_ms))
                                 : QStringLiteral("-");
      UpdateItem(task_table_, row, 7, detected, kDimText);

      if (id == selected_task_) {
        selected_row = row;
      }
    }
    if (selected_row >= 0) {
      task_table_->selectRow(selected_row);
    }
    task_table_->verticalScrollBar()->setValue(scroll);
    task_table_->setUpdatesEnabled(true);
    task_table_->viewport()->update();
  }

  // ---- 车辆表 ----
  {
    const QSignalBlocker blocker(robot_table_);
    robot_table_->setUpdatesEnabled(false);
    const int scroll = robot_table_->verticalScrollBar()->value();
    const int64_t now = QDateTime::currentMSecsSinceEpoch();
    robot_table_->setRowCount(static_cast<int>(model.robots().size()));

    int row = 0;
    for (const auto& [id, robot] : model.robots()) {
      const QString state_text =
        robot.online ? QString::fromStdString(fleet::monitor::RobotStateName(robot.state))
                     : QStringLiteral("离线");
      UpdateItem(robot_table_, row, 0, QString::fromStdString(id), kText);
      UpdateItem(robot_table_, row, 1, state_text, RobotStateColor(robot.state, robot.online));
      UpdateItem(robot_table_, row, 2, QStringLiteral("(%1, %2) %3°")
                   .arg(robot.x, 0, 'f', 2)
                   .arg(robot.y, 0, 'f', 2)
                   .arg(robot.yaw * 180.0 / 3.14159265358979323846, 0, 'f', 0),
                 kText);
      UpdateItem(robot_table_, row, 3,
                 robot.current_task.empty() ? QStringLiteral("-")
                                            : QString::fromStdString(robot.current_task),
                 robot.current_task.empty() ? kDimText : kAccent);
      UpdateItem(robot_table_, row, 4, QString::number(robot.tasks_done), kDoneColor);
      UpdateItem(robot_table_, row, 5, QString::number(robot.tasks_failed),
                 robot.tasks_failed > 0 ? kErrorColor : kDimText);
      UpdateItem(robot_table_, row, 6, QString::number(robot.tasks_canceled),
                 robot.tasks_canceled > 0 ? kWarnColor : kDimText);

      const QString heartbeat =
        robot.heartbeat_seen
          ? (robot.online
               ? QStringLiteral("%1 前 · #%2")
                   .arg(QString::fromStdString(
                     fleet::monitor::FormatAge(now - robot.last_seen_ms)))
                   .arg(robot.heartbeat_seq)
               : QStringLiteral("中断 %1")
                   .arg(QString::fromStdString(
                     fleet::monitor::FormatAge(now - robot.last_seen_ms))))
          : QStringLiteral("从未收到");
      UpdateItem(robot_table_, row, 7, heartbeat, robot.online ? kDimText : kErrorColor);
      ++row;
    }
    robot_table_->verticalScrollBar()->setValue(scroll);
    robot_table_->setUpdatesEnabled(true);
    robot_table_->viewport()->update();
  }
}

void MainWindow::RefreshHeader()
{
  const auto& model = Model();
  int online = 0;
  for (const auto& [id, robot] : model.robots()) {
    if (robot.online) {
      ++online;
    }
  }
  status_robots_->setText(QStringLiteral("车辆 %1/%2 在线")
                            .arg(online)
                            .arg(model.robots().size()));
  status_tasks_->setText(QStringLiteral("任务 %1 进行中 · %2 完成 · %3 失败")
                           .arg(model.ActiveCount())
                           .arg(model.DoneCount())
                           .arg(model.FailedCount()));

  int alarms = 0;
  if (replay_active_) {
    for (const auto& event : bag_.events) {
      if (event.stamp_ms <= replay_pos_ms_ && event.severity != fleet::monitor::Severity::kInfo) {
        ++alarms;
      }
    }
  } else {
    for (const auto& event : live_events_) {
      if (event.severity != fleet::monitor::Severity::kInfo) {
        ++alarms;
      }
    }
  }
  status_alarm_->setText(QStringLiteral("告警 %1").arg(alarms));
}

void MainWindow::RefreshMap()
{
  const auto& model = Model();
  std::vector<fleet::monitor::RobotView> robots;
  robots.reserve(model.robots().size());
  for (const auto& [id, robot] : model.robots()) {
    robots.push_back(robot);
  }
  map_view_->SetRobots(robots);
  map_view_->SetTasks(model.SortedTasks());
  map_view_->SetSelectedTask(selected_task_);
}

void MainWindow::RenderLog()
{
  QString html;
  const QString prefix = QStringLiteral("<span style=\"color:%1\">[%2] %3</span><br>");
  auto append = [&html, &prefix](const fleet::monitor::Event& event) {
    html += prefix.arg(SeverityColor(event.severity).name())
              .arg(QString::fromStdString(fleet::monitor::FormatClock(event.stamp_ms)))
              .arg(QString::fromStdString(event.text).toHtmlEscaped());
  };

  if (replay_active_) {
    for (const auto& event : bag_.events) {
      if (event.stamp_ms <= replay_pos_ms_) {
        append(event);
      }
    }
  } else {
    for (const auto& event : live_events_) {
      append(event);
    }
  }
  log_view_->setHtml(html);
  log_view_->verticalScrollBar()->setValue(log_view_->verticalScrollBar()->maximum());
}

void MainWindow::RenderAlarms()
{
  alarm_list_->clear();
  std::vector<const fleet::monitor::Event*> visible;
  if (replay_active_) {
    for (const auto& event : bag_.events) {
      if (event.stamp_ms <= replay_pos_ms_ && event.severity != fleet::monitor::Severity::kInfo) {
        visible.push_back(&event);
      }
    }
  } else {
    for (const auto& event : live_events_) {
      if (event.severity != fleet::monitor::Severity::kInfo) {
        visible.push_back(&event);
      }
    }
  }
  for (auto it = visible.rbegin(); it != visible.rend() && alarm_list_->count() < 200; ++it) {
    const auto& event = **it;
    auto* item = new QListWidgetItem(
      QStringLiteral("[%1] %2")
        .arg(QString::fromStdString(fleet::monitor::FormatClock(event.stamp_ms)))
        .arg(QString::fromStdString(event.text)));
    item->setForeground(SeverityColor(event.severity));
    alarm_list_->addItem(item);
  }
}

// ---------------- 操作 ----------------

void MainWindow::SubmitFromFields()
{
  if (node_ == nullptr) {
    return;
  }
  node_->SubmitTask(static_cast<uint8_t>(color_combo_->currentData().toInt()),
                    x_spin_->value(), y_spin_->value());
}

void MainWindow::CancelSelectedTask()
{
  if (selected_task_.isEmpty()) {
    statusBar()->showMessage(QStringLiteral("先在任务表里选中一行"), 3000);
    return;
  }
  node_->CancelTask(selected_task_);
}

void MainWindow::TogglePickMode(bool on)
{
  map_view_->SetPickMode(on);
}

void MainWindow::SelectCamera(const QString& robot)
{
  if (robot.isEmpty()) {
    return;
  }
  {
    // 命令行/程序内切换相机时同步下拉框；阻断信号避免再回调一次
    const QSignalBlocker blocker(camera_combo_);
    if (camera_combo_->findText(robot) < 0) {
      camera_combo_->addItem(robot);
    }
    camera_combo_->setCurrentText(robot);
  }
  hub_->setCameraRobot(robot);
  camera_view_->SetRobot(robot);
}

void MainWindow::SelectTab(const QString& name)
{
  static const QStringList kNames{QStringLiteral("任务"), QStringLiteral("车辆"),
                                  QStringLiteral("告警"), QStringLiteral("日志")};
  const int index = kNames.indexOf(name);
  if (index >= 0 && tabs_ != nullptr) {
    tabs_->setCurrentIndex(index);
  }
}

// ---------------- 回放 ----------------

bool MainWindow::OpenBag(const QString& uri)
{
  ReplayData data;
  std::string error;
  if (!LoadBag(uri.toStdString(), &data, &error)) {
    QMessageBox::warning(this, QStringLiteral("打开 rosbag 失败"),
                         QString::fromStdString(error));
    return false;
  }
  EnterReplay(data, uri);
  return true;
}

void MainWindow::EnterReplay(const ReplayData& data, const QString& uri)
{
  bag_ = data;
  replay_active_ = true;
  replay_playing_ = true;
  replay_ = fleet::monitor::MonitorModel();
  replay_pos_ms_ = bag_.start_ms;
  replay_last_render_ms_ = 0;
  last_play_tick_ms_ = 0;

  {
    const QSignalBlocker blocker(replay_slider_);
    replay_slider_->setRange(0, static_cast<int>(std::max<int64_t>(1, bag_.end_ms - bag_.start_ms)));
    replay_slider_->setValue(0);
  }
  replay_play_->setText(QStringLiteral("暂停"));
  replay_bar_->setVisible(true);
  status_mode_->setText(QStringLiteral("回放 · %1").arg(uri.section('/', -1)));
  ApplyReplayAt(replay_pos_ms_);
}

void MainWindow::ExitReplay()
{
  replay_active_ = false;
  replay_playing_ = false;
  replay_bar_->setVisible(false);
  status_mode_->setText(QStringLiteral("实时"));
  RefreshAll();
  RenderLog();
  RenderAlarms();
}

void MainWindow::ApplyReplayAt(int64_t stamp_ms)
{
  replay_pos_ms_ = std::clamp(stamp_ms, bag_.start_ms, bag_.end_ms);

  // 回放相当于"从头重放到当前时刻"：条目数只有几百，重建比维护差分简单且不会错
  replay_.Clear();
  for (const auto& entry : bag_.entries) {
    if (entry.stamp_ms > replay_pos_ms_) {
      break;
    }
    switch (entry.kind) {
      case ReplayEntry::Kind::kTaskSnapshot:
        replay_.OnTaskSnapshot(entry.task, entry.stamp_ms);
        break;
      case ReplayEntry::Kind::kRobotStatus:
        replay_.OnRobotStatus(entry.robot, entry.stamp_ms);
        break;
      case ReplayEntry::Kind::kTaskStatus:
        replay_.OnTaskStatus(entry.status, entry.stamp_ms);
        break;
    }
  }
  replay_.Sweep(replay_pos_ms_);
  replay_.PruneTasks(replay_pos_ms_);

  RefreshAll();
  // 日志/告警每次全量重建，按 200ms 节流；拖动滑块（暂停态）时立即刷新
  if (!replay_playing_ || replay_pos_ms_ - replay_last_render_ms_ >= 200) {
    replay_last_render_ms_ = replay_pos_ms_;
    RenderLog();
    RenderAlarms();
  }
  UpdateReplayBar();
}

void MainWindow::UpdateReplayBar()
{
  replay_time_->setText(QStringLiteral("%1 / %2")
                          .arg(FormatDuration(replay_pos_ms_ - bag_.start_ms))
                          .arg(FormatDuration(bag_.end_ms - bag_.start_ms)));
  const QSignalBlocker blocker(replay_slider_);
  replay_slider_->setValue(static_cast<int>(replay_pos_ms_ - bag_.start_ms));
}

void MainWindow::AdvanceReplay()
{
  if (!replay_active_ || !replay_playing_) {
    last_play_tick_ms_ = 0;
    return;
  }
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  if (last_play_tick_ms_ == 0) {
    last_play_tick_ms_ = now;
    return;
  }
  const qint64 delta = now - last_play_tick_ms_;
  last_play_tick_ms_ = now;

  if (replay_pos_ms_ >= bag_.end_ms) {
    replay_playing_ = false;
    replay_play_->setText(QStringLiteral("播放"));
    return;
  }
  ApplyReplayAt(replay_pos_ms_ + delta * replay_speed_);
  if (replay_pos_ms_ >= bag_.end_ms) {
    replay_playing_ = false;
    replay_play_->setText(QStringLiteral("播放"));
  }
}

}  // namespace fleet::gui
