#pragma once

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedLayout>
#include <QTimer>
#include <QWidget>

#include "common/params.h"
#include "selfdrive/ui/qt/offroad/driverview.h"
#include "selfdrive/ui/qt/body.h"
#include "selfdrive/ui/qt/onroad.h"
#include "selfdrive/ui/qt/sidebar.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/offroad_alerts.h"
#include "selfdrive/ui/ui.h"

class OffroadHome : public QFrame {
  Q_OBJECT

public:
  explicit OffroadHome(QWidget* parent = 0);

signals:
  void openSettings(int index = 0, const QString &param = "");

private:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void refresh();

  Params params;

  QTimer* timer;
  ElidedLabel* date;
  ElidedLabel* version;
  QStackedLayout* center_layout;
  UpdateAlert *update_widget;
  OffroadAlert* alerts_widget;
  QPushButton* alert_notif;
  QPushButton* update_notif;

  // FrogPilot variables
  std::map<QString, QString> MODEL_NAME;
};

// The HomeWindow class extends QWidget, which is a base class for all UI objects in Qt.
class HomeWindow : public QWidget {
  Q_OBJECT // This macro enables the class to use signals and slots mechanism provided by Qt

public:
  explicit HomeWindow(QWidget* parent = 0); // Constructor with an optional parent widget

signals:
  // Signals that can be emitted to indicate that settings should be opened or closed
  void openSettings(int index = 0, const QString &param = "");
  void closeSettings();

public slots:
  // Slots that handle various transitions in the UI
  void offroadTransition(bool offroad);
  void showDriverView(bool show);
  void showSidebar(bool show);
  void showMapPanel(bool show);

protected:
  // Event handlers for mouse press and double-click events
  void mousePressEvent(QMouseEvent* e) override;
  void mouseDoubleClickEvent(QMouseEvent* e) override;

private:
  // Pointers to various UI components that make up the HomeWindow
  Sidebar *sidebar;
  OffroadHome *home;
  OnroadWindow *onroad;
  BodyWindow *body;
  DriverViewWindow *driver_view;
  // The layout that allows switching between different screens (widgets)
  QStackedLayout *slayout;

  // FrogPilot-specific variable, possibly for storing persistent settings
  Params params;

private slots:
  // Slot to handle UI state updates
  void updateState(const UIState &s);
};