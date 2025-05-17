#pragma once

#include <QPushButton>
#include <QWidget>
#include <QLabel>
#include <memory>

#ifdef SUNNYPILOT
#include "selfdrive/ui/sunnypilot/ui.h"
#else
#include "selfdrive/ui/ui.h"
#endif

const int btn_size = 192;
const int img_size = (btn_size / 4) * 3;

class ExperimentalButton : public QPushButton {
  Q_OBJECT

public:
  explicit ExperimentalButton(QWidget *parent = 0);
  virtual void updateState(const UIState &s);

private:
  void paintEvent(QPaintEvent *event) override;
  void changeMode();

  Params params;

protected:
  virtual void drawButton(QPainter &p);

  QPixmap engage_img;
  QPixmap experimental_img;
  bool experimental_mode;
  bool engageable;
};

  class SpeedControl : public QWidget {
    Q_OBJECT

  public:
    explicit SpeedControl(QWidget *parent = nullptr);

  private:
    std::unique_ptr<PubMaster> pm;
    uint32_t speed;
    QString setSpeedValue;
    QPushButton *decreaseButton;
    QPushButton *increaseButton;
 };

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity);
