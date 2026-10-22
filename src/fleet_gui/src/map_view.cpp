#include "fleet_gui/map_view.hpp"

#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QResizeEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace fleet::gui {

namespace {

const QColor kBackground(16, 19, 23);
const QColor kGrid(38, 44, 52);
const QColor kFree(46, 52, 60);
const QColor kOccupied(150, 160, 172);
const QColor kUnknown(26, 30, 36);
const QColor kText(196, 203, 211);
const QColor kDimText(120, 129, 140);

const QColor kRobotPalette[] = {
  QColor(56, 189, 248),   // robot1 青
  QColor(251, 191, 36),   // robot2 琥珀
  QColor(167, 139, 250),
  QColor(52, 211, 153),
};

QColor RobotColor(size_t index)
{
  return kRobotPalette[index % (sizeof(kRobotPalette) / sizeof(kRobotPalette[0]))];
}

}  // namespace

MapView::MapView(QWidget* parent)
: QWidget(parent)
{
  setMinimumSize(360, 300);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
  pick_hint_ = QStringLiteral("滚轮缩放 · 左键拖拽平移");
}

void MapView::SetMap(const fleet::monitor::MapData& map)
{
  map_ = map;
  if (map.Empty()) {
    map_image_ = QImage();
    update();
    return;
  }

  // OccupancyGrid 的第 0 行在最小 y；QImage 第 0 行在顶部，所以建图时上下翻转，
  // 之后 world -> 屏幕就是一次线性映射，不用再记方向
  map_image_ = QImage(map.width, map.height, QImage::Format_RGB32);
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const int8_t v = map.cells[static_cast<size_t>(y) * map.width + x];
      const QColor c = v < 0 ? kUnknown : (v > 50 ? kOccupied : kFree);
      map_image_.setPixelColor(x, map.height - 1 - y, c);
    }
  }
  user_interacted_ = false;   // 新地图到达时回到全图视图
  FitToContent();
  update();
}

void MapView::SetRobots(const std::vector<fleet::monitor::RobotView>& robots)
{
  robots_ = robots;
  for (const auto& robot : robots_) {
    auto& trail = trails_[robot.robot_id];
    const QPointF p(robot.x, robot.y);
    if (trail.empty() || std::hypot(trail.back().x() - p.x(), trail.back().y() - p.y()) > 0.12) {
      trail.push_back(p);
      while (trail.size() > 240) {
        trail.pop_front();
      }
    }
  }
  update();
}

void MapView::SetTasks(const std::vector<fleet::monitor::TaskView>& tasks)
{
  tasks_ = tasks;
  update();
}

void MapView::SetSelectedTask(const QString& task_id)
{
  selected_task_ = task_id;
  update();
}

void MapView::SetPickMode(bool on)
{
  pick_mode_ = on;
  setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
  pick_hint_ = on ? QStringLiteral("点击地图选择目标点（Esc 取消）") : QStringLiteral("滚轮缩放 · 左键拖拽平移");
  update();
}

QRectF MapView::WorldRect() const
{
  if (!map_.Empty()) {
    return QRectF(map_.origin_x, map_.origin_y,
                  map_.width * map_.resolution, map_.height * map_.resolution);
  }
  return QRectF(-10.0, -10.0, 20.0, 20.0);
}

QPointF MapView::WorldToScreen(double x, double y) const
{
  return QPointF(width() / 2.0 + (x - center_.x()) * scale_,
                 height() / 2.0 - (y - center_.y()) * scale_);
}

QPointF MapView::ScreenToWorld(const QPointF& p) const
{
  return QPointF(center_.x() + (p.x() - width() / 2.0) / scale_,
                 center_.y() - (p.y() - height() / 2.0) / scale_);
}

void MapView::FitToContent()
{
  const QRectF world = WorldRect();
  if (width() <= 1 || height() <= 1 || world.width() <= 0.0 || world.height() <= 0.0) {
    fit_pending_ = true;
    return;
  }
  const double margin = 18.0;
  const double sx = (width() - 2 * margin) / world.width();
  const double sy = (height() - 2 * margin) / world.height();
  scale_ = std::clamp(std::min(sx, sy), 2.0, 400.0);
  center_ = world.center();
  fit_pending_ = false;
}

void MapView::resizeEvent(QResizeEvent* event)
{
  QWidget::resizeEvent(event);
  if (fit_pending_ || !user_interacted_) {
    FitToContent();
  }
}

void MapView::mouseDoubleClickEvent(QMouseEvent* event)
{
  if (event->button() == Qt::LeftButton && !pick_mode_) {
    user_interacted_ = false;
    FitToContent();
    update();
  }
}

QColor MapView::TaskColor(int color) const
{
  switch (color) {
    case fleet::monitor::kColorRed: return QColor(255, 95, 86);
    case fleet::monitor::kColorGreen: return QColor(61, 220, 132);
    case fleet::monitor::kColorBlue: return QColor(74, 163, 255);
    default: return QColor(200, 200, 200);
  }
}

void MapView::DrawGrid(QPainter& p) const
{
  const QPointF top_left = ScreenToWorld(QPointF(0, 0));
  const QPointF bottom_right = ScreenToWorld(QPointF(width(), height()));
  const double step = scale_ < 30.0 ? 2.0 : 1.0;

  p.setPen(QPen(kGrid, 1.0));
  for (double x = std::floor(top_left.x() / step) * step; x <= bottom_right.x(); x += step) {
    p.drawLine(WorldToScreen(x, top_left.y()), WorldToScreen(x, bottom_right.y()));
  }
  for (double y = std::floor(bottom_right.y() / step) * step; y <= top_left.y(); y += step) {
    p.drawLine(WorldToScreen(top_left.x(), y), WorldToScreen(bottom_right.x(), y));
  }
}

void MapView::DrawTasks(QPainter& p) const
{
  const QFont base = p.font();
  QFont small = base;
  small.setPointSizeF(std::max(7.0, base.pointSizeF() - 2.0));

  for (const auto& task : tasks_) {
    const QPointF c = WorldToScreen(task.x, task.y);
    const QColor color = TaskColor(task.color);
    const bool selected = selected_task_ == QString::fromStdString(task.task_id);
    const bool active = fleet::monitor::IsActiveState(task.state);

    QPolygonF diamond;
    const double r = selected ? 9.0 : 7.0;
    diamond << QPointF(c.x(), c.y() - r) << QPointF(c.x() + r, c.y())
            << QPointF(c.x(), c.y() + r) << QPointF(c.x() - r, c.y());

    QColor fill = color;
    if (task.state == fleet::monitor::kStateDone) {
      fill.setAlpha(70);
    } else if (task.state == fleet::monitor::kStateCanceled) {
      fill = QColor(110, 116, 126);
      fill.setAlpha(90);
    } else if (task.state == fleet::monitor::kStateFailed) {
      fill = QColor(255, 95, 86);
      fill.setAlpha(60);
    }

    if (active) {
      QColor halo = color;
      halo.setAlpha(50);
      p.setPen(Qt::NoPen);
      p.setBrush(halo);
      p.drawEllipse(c, 15.0, 15.0);
    }

    p.setPen(QPen(task.state == fleet::monitor::kStatePending ? QColor(220, 226, 233) : color,
                  1.6, task.state == fleet::monitor::kStatePending ? Qt::DashLine : Qt::SolidLine));
    p.setBrush(fill);
    p.drawPolygon(diamond);

    if (task.state == fleet::monitor::kStateFailed ||
        task.state == fleet::monitor::kStateCanceled) {
      p.setPen(QPen(QColor(18, 20, 24), 1.8));
      p.drawLine(c + QPointF(-3, -3), c + QPointF(3, 3));
      p.drawLine(c + QPointF(-3, 3), c + QPointF(3, -3));
    }
    if (selected) {
      p.setPen(QPen(QColor(255, 255, 255), 1.2, Qt::DotLine));
      p.setBrush(Qt::NoBrush);
      p.drawEllipse(c, 13.0, 13.0);
    }

    if (active || task.state == fleet::monitor::kStatePending || selected) {
      p.setFont(small);
      p.setPen(kText);
      p.drawText(QPointF(c.x() + 11.0, c.y() - 8.0), QString::fromStdString(task.task_id));
    }
  }
}

void MapView::DrawRobots(QPainter& p) const
{
  const QFont base = p.font();
  QFont small = base;
  small.setPointSizeF(std::max(7.0, base.pointSizeF() - 2.0));

  for (size_t i = 0; i < robots_.size(); ++i) {
    const auto& robot = robots_[i];
    const QColor color = robot.online ? RobotColor(i) : QColor(110, 116, 126);
    const QPointF c = WorldToScreen(robot.x, robot.y);

    // 轨迹：离线时也画，方便回溯最后位置
    const auto trail_it = trails_.find(robot.robot_id);
    if (trail_it != trails_.end() && trail_it->second.size() > 1) {
      QPainterPath path;
      bool first = true;
      for (const auto& wp : trail_it->second) {
        const QPointF sp = WorldToScreen(wp.x(), wp.y());
        if (first) {
          path.moveTo(sp);
          first = false;
        } else {
          path.lineTo(sp);
        }
      }
      QColor trail_color = color;
      trail_color.setAlpha(70);
      p.setPen(QPen(trail_color, 1.4));
      p.setBrush(Qt::NoBrush);
      p.drawPath(path);
    }

    if (!robot.online) {
      p.setPen(QPen(QColor(150, 156, 166), 1.2, Qt::DashLine));
      p.setBrush(QColor(60, 66, 76, 160));
      p.drawEllipse(c, 10.0, 10.0);
    } else {
      QColor halo = color;
      halo.setAlpha(45);
      p.setPen(Qt::NoPen);
      p.setBrush(halo);
      p.drawEllipse(c, 16.0, 16.0);

      p.setPen(QPen(QColor(245, 248, 250), 1.4));
      p.setBrush(color);
      p.drawEllipse(c, 9.0, 9.0);

      // 朝向：车头线段
      const QPointF head = c + QPointF(std::cos(robot.yaw), -std::sin(robot.yaw)) * 15.0;
      p.setPen(QPen(QColor(245, 248, 250), 2.0));
      p.drawLine(c, head);
    }

    p.setFont(small);
    p.setPen(robot.online ? kText : kDimText);
    QString label = QString::fromStdString(robot.robot_id);
    if (!robot.online) {
      // 区分"掉线"和"从来没有过心跳"：前者是故障，后者多半是 agent 没起来
      label += robot.heartbeat_seen ? QStringLiteral(" · 离线") : QStringLiteral(" · 无心跳");
    }
    p.drawText(QPointF(c.x() - 22.0, c.y() + 24.0), label);
  }
}

void MapView::paintEvent(QPaintEvent*)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.fillRect(rect(), kBackground);

  if (!map_image_.isNull()) {
    const QRectF world = WorldRect();
    // QRectF::bottom() 返回 y + height（不是最小 y），屏幕底边必须用 top() 推
    const QPointF tl = WorldToScreen(world.left(), world.top() + world.height());
    const QPointF br = WorldToScreen(world.right(), world.top());
    p.setRenderHint(QPainter::SmoothPixmapTransform, false);
    p.drawImage(QRectF(tl, br), map_image_);
    p.setPen(QPen(QColor(64, 71, 82), 1.0));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(tl, br));
  }

  DrawGrid(p);
  DrawTasks(p);
  DrawRobots(p);

  // 左上角状态提示
  QFont small = p.font();
  small.setPointSizeF(std::max(7.0, small.pointSizeF() - 2.0));
  p.setFont(small);
  p.setPen(kDimText);
  p.drawText(QPointF(12.0, 20.0), pick_hint_);
  if (map_.Empty()) {
    p.setPen(QColor(255, 180, 90));
    p.drawText(QPointF(12.0, 38.0), QStringLiteral("尚未收到 /map，场地按 20×20 m 估算"));
  } else {
    p.setPen(kDimText);
    p.drawText(QPointF(12.0, 38.0),
               QStringLiteral("地图 %1×%2 m · 比例 %3 px/m")
                 .arg(map_.width * map_.resolution, 0, 'f', 1)
                 .arg(map_.height * map_.resolution, 0, 'f', 1)
                 .arg(scale_, 0, 'f', 1));
  }
}

void MapView::mousePressEvent(QMouseEvent* event)
{
  if (event->button() == Qt::LeftButton) {
    if (pick_mode_) {
      const QPointF world = ScreenToWorld(event->position());
      emit mapClicked(world.x(), world.y());
      return;
    }
    panning_ = true;
    user_interacted_ = true;
    pan_last_ = event->position();
  }
}

void MapView::mouseMoveEvent(QMouseEvent* event)
{
  if (panning_) {
    const QPointF delta = event->position() - pan_last_;
    center_ -= QPointF(delta.x() / scale_, -delta.y() / scale_);
    pan_last_ = event->position();
    update();
  }
}

void MapView::mouseReleaseEvent(QMouseEvent* event)
{
  if (event->button() == Qt::LeftButton) {
    panning_ = false;
  }
}

void MapView::wheelEvent(QWheelEvent* event)
{
  const QPointF cursor = event->position();
  const QPointF before = ScreenToWorld(cursor);
  const double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
  scale_ = std::clamp(scale_ * factor, 2.0, 400.0);
  user_interacted_ = true;
  const QPointF after = ScreenToWorld(cursor);
  center_ += before - after;   // 让光标下的世界点保持不动
  update();
}

}  // namespace fleet::gui