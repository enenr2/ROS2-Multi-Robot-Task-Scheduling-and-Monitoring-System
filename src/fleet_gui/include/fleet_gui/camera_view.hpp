// 相机画面：等比缩放显示选中机器人的 /<robot>/camera/image，叠一层状态信息。
#pragma once

#include <QImage>
#include <QString>
#include <QWidget>

class QTimer;

namespace fleet::gui {

class CameraView : public QWidget
{
  Q_OBJECT

public:
  explicit CameraView(QWidget* parent = nullptr);

  void SetRobot(const QString& robot);
  void SetFrame(const QImage& image);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  QImage image_;
  QString robot_;
  QString source_robot_;
  int frames_{0};
  double fps_{0.0};
  qint64 fps_window_start_ms_{0};
  qint64 last_frame_ms_{0};
  QTimer* refresh_timer_;
};

}  // namespace fleet::gui