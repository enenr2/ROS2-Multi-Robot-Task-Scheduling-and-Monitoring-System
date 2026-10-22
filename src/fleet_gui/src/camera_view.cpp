#include "fleet_gui/camera_view.hpp"

#include <QDateTime>
#include <QFont>
#include <QPainter>
#include <QTimer>

#include <algorithm>

namespace fleet::gui {

CameraView::CameraView(QWidget* parent)
: QWidget(parent)
{
  setMinimumSize(320, 200);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(1000);
  connect(refresh_timer_, &QTimer::timeout, this, [this]() { update(); });
  refresh_timer_->start();
}

void CameraView::SetRobot(const QString& robot)
{
  if (robot_ == robot) {
    return;
  }
  robot_ = robot;
  image_ = QImage();
  source_robot_.clear();
  frames_ = 0;
  fps_ = 0.0;
  fps_window_start_ms_ = 0;
  last_frame_ms_ = 0;
  update();
}

void CameraView::SetFrame(const QImage& image)
{
  image_ = image;
  last_frame_ms_ = QDateTime::currentMSecsSinceEpoch();
  if (fps_window_start_ms_ == 0) {
    fps_window_start_ms_ = last_frame_ms_;
  }
  ++frames_;
  const qint64 elapsed = last_frame_ms_ - fps_window_start_ms_;
  if (elapsed >= 1000) {
    fps_ = frames_ * 1000.0 / static_cast<double>(elapsed);
    frames_ = 0;
    fps_window_start_ms_ = last_frame_ms_;
  }
  update();
}

void CameraView::paintEvent(QPaintEvent*)
{
  QPainter p(this);
  p.fillRect(rect(), QColor(16, 18, 22));

  if (image_.isNull()) {
    p.setPen(QColor(120, 129, 140));
    QFont f = p.font();
    f.setPointSizeF(f.pointSizeF() + 1.0);
    p.setFont(f);
    p.drawText(rect(), Qt::AlignCenter,
               robot_.isEmpty() ? QStringLiteral("未选择相机")
                                : QStringLiteral("等待 %1 的相机画面…").arg(robot_));
    return;
  }

  const QSize target = image_.size().scaled(size(), Qt::KeepAspectRatio);
  const QRect frame_rect(QPoint((width() - target.width()) / 2, (height() - target.height()) / 2),
                         target);
  p.setRenderHint(QPainter::SmoothPixmapTransform, true);
  p.drawImage(frame_rect, image_);

  // 顶栏：机器人、分辨率、帧率；超过 2s 没有新帧就标黄，避免把定格的旧画面当成实时流
  const qint64 age = QDateTime::currentMSecsSinceEpoch() - last_frame_ms_;
  const bool stale = last_frame_ms_ > 0 && age > 2000;

  QFont small = p.font();
  small.setPointSizeF(std::max(7.0, small.pointSizeF() - 2.0));
  p.setFont(small);

  const QString info = QString("%1  %2×%3  %4 fps%5")
                         .arg(robot_.isEmpty() ? QStringLiteral("-") : robot_)
                         .arg(image_.width())
                         .arg(image_.height())
                         .arg(fps_, 0, 'f', 1)
                         .arg(stale ? QStringLiteral("  · 画面已停更 %1s").arg(age / 1000) : QString());

  QFontMetrics metrics(small);
  const int text_width = metrics.horizontalAdvance(info) + 16;
  const QRect bar(frame_rect.left() + 8, frame_rect.top() + 8, text_width, 22);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(10, 12, 15, 170));
  p.drawRoundedRect(bar, 4, 4);
  p.setPen(stale ? QColor(255, 190, 90) : QColor(210, 216, 224));
  p.drawText(bar.adjusted(8, 0, -8, 0), Qt::AlignVCenter | Qt::AlignLeft, info);
}

}  // namespace fleet::gui