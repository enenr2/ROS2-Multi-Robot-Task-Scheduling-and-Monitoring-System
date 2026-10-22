// 场地视图：栅格地图 + 机器人位姿 + 任务目标点，支持缩放/平移/点选。
#pragma once

#include <QImage>
#include <QPointF>
#include <QWidget>

#include <deque>
#include <map>
#include <string>
#include <vector>

#include "fleet_gui/monitor_core.hpp"

class QPainter;

namespace fleet::gui {

class MapView : public QWidget
{
  Q_OBJECT

public:
  explicit MapView(QWidget* parent = nullptr);

  void SetMap(const fleet::monitor::MapData& map);
  void SetRobots(const std::vector<fleet::monitor::RobotView>& robots);
  void SetTasks(const std::vector<fleet::monitor::TaskView>& tasks);
  void SetSelectedTask(const QString& task_id);
  void SetPickMode(bool on);
  void FitToContent();

signals:
  void mapClicked(double x, double y);

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

private:
  QPointF WorldToScreen(double x, double y) const;
  QPointF ScreenToWorld(const QPointF& p) const;
  QRectF WorldRect() const;
  void DrawGrid(QPainter& p) const;
  void DrawTasks(QPainter& p) const;
  void DrawRobots(QPainter& p) const;
  QColor TaskColor(int color) const;

  fleet::monitor::MapData map_;
  QImage map_image_;
  std::vector<fleet::monitor::RobotView> robots_;
  std::vector<fleet::monitor::TaskView> tasks_;
  std::map<std::string, std::deque<QPointF>> trails_;
  QString selected_task_;
  QString pick_hint_;

  QPointF center_{-1.0, 1.5};   // 世界坐标
  double scale_{45.0};          // 像素 / 米
  bool pick_mode_{false};
  bool panning_{false};
  QPointF pan_last_;
  bool fit_pending_{true};
  // 用户一旦缩放/平移过，窗口尺寸变化时就不再强行回到适配视图
  bool user_interacted_{false};
};

}  // namespace fleet::gui