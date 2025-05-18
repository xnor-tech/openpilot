#include "selfdrive/ui/qt/onroad/buttons.h"

#include <QPainter>
#include <QHBoxLayout>
#include <QLabel>
#include <QStyleOption>

#include "selfdrive/ui/qt/util.h"
#include "common/swaglog.h"

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(opacity);
  p.drawPixmap(center - QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
}

// ExperimentalButton
ExperimentalButton::ExperimentalButton(QWidget *parent) : experimental_mode(false), engageable(false), QPushButton(parent) {
  setFixedSize(btn_size, btn_size);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &ExperimentalButton::changeMode);
}

void ExperimentalButton::changeMode() {
  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool can_change = hasLongitudinalControl(cp) && params.getBool("ExperimentalModeConfirmed");
  if (can_change) {
    params.putBool("ExperimentalMode", !experimental_mode);
  }
}

void ExperimentalButton::updateState(const UIState &s) {
  const auto cs = (*s.sm)["selfdriveState"].getSelfdriveState();
  bool eng = cs.getEngageable() || cs.getEnabled();
  if ((cs.getExperimentalMode() != experimental_mode) || (eng != engageable)) {
    engageable = eng;
    experimental_mode = cs.getExperimentalMode();
    update();
  }
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  drawButton(p);
}

void ExperimentalButton::drawButton(QPainter &p) {
  QPixmap img = experimental_mode ? experimental_img : engage_img;
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), img, QColor(0, 0, 0, 166), (isDown() || !engageable) ? 0.6 : 1.0);
}

  // SpeedControl
  SpeedControl::SpeedControl(QWidget *parent) : QWidget(parent), speed(30) {
    pm = std::make_unique<PubMaster>(std::vector<const char*>{"uiSetSpeed"});

    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    QFont buttonFont;
    buttonFont.setPointSize(100);
    QPalette buttonPalette;
    buttonPalette.setColor(QPalette::ButtonText, Qt::white);
    QString baseButtonStyleSheet = "QPushButton {"
                                   "border: 15px solid rgba(15, 15, 15, 0.5);"
                                   "border-radius: 25px;"
                                   "background-color: rgba(21, 21, 21, 0.1);"
                                   "}"
                                   "QPushButton:pressed {"
                                   "background-color: rgba(255, 255, 255, 0.1);"
                                   "}";

    decreaseButton = new QPushButton("-", this);
    decreaseButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    decreaseButton->setFixedWidth(375);
    decreaseButton->setFixedHeight(200);
    decreaseButton->setFont(buttonFont);
    decreaseButton->setPalette(buttonPalette);
    decreaseButton->setStyleSheet(baseButtonStyleSheet);
    layout->addWidget(decreaseButton);

    QSpacerItem *spacer = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);
    layout->addItem(spacer);

    increaseButton = new QPushButton("+", this);
    increaseButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    increaseButton->setFixedWidth(375);
    increaseButton->setFixedHeight(200);
    increaseButton->setFont(buttonFont);
    increaseButton->setPalette(buttonPalette);
    increaseButton->setStyleSheet(baseButtonStyleSheet);
    layout->addWidget(increaseButton);

    QObject::connect(increaseButton, &QPushButton::pressed, [this]() {
      MessageBuilder msg;
      auto m = msg.initEvent().initUiSetSpeed();
      m.setButtonSignal(1);
      pm->send("uiSetSpeed", msg);
    });

    QObject::connect(increaseButton, &QPushButton::released, [this]() {
      MessageBuilder msg;
      auto m = msg.initEvent().initUiSetSpeed();
      m.setButtonSignal(0);
      pm->send("uiSetSpeed", msg);
    });

    QObject::connect(decreaseButton, &QPushButton::pressed, [this]() {
      MessageBuilder msg;
      auto m = msg.initEvent().initUiSetSpeed();
      m.setButtonSignal(-1);
      pm->send("uiSetSpeed", msg);
    });

    QObject::connect(decreaseButton, &QPushButton::released, [this]() {
      MessageBuilder msg;
      auto m = msg.initEvent().initUiSetSpeed();
      m.setButtonSignal(0);
      pm->send("uiSetSpeed", msg);
    });

    setLayout(layout);
 }
