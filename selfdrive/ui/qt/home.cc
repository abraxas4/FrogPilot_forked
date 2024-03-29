#include "selfdrive/ui/qt/home.h"

#include <QHBoxLayout>
#include <QMouseEvent>
#include <QStackedWidget>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/offroad/experimental_mode.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/drive_stats.h"
#include "selfdrive/ui/qt/widgets/prime.h"

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

// =======================================
// HomeWindow: the container for the offroad and onroad UIs

// HomeWindow is likely the top-level widget that contains all other widgets like sidebar, onroad/offroad views, etc.
HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  // Main layout is horizontal, meaning widgets will be placed side by side
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0); // No margins for the layout, widgets will occupy the full space available
  main_layout->setSpacing(0); // No spacing between widgets, they will be adjacent to each other

  // Initialize the sidebar widget and add it to the main layout
  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  // Connect a signal emitted by the sidebar (likely when a button is clicked) to a slot in this window
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  // Stack layout allows stacking widgets on top of each other, showing one at a time
  slayout = new QStackedLayout();
  // Add the stacked layout to the horizontal layout next to the sidebar
  main_layout->addLayout(slayout);

  // The home widget is the offroad home screen and is added to the stack layout
  home = new OffroadHome(this);
  // Connect a signal to open settings from the offroad home screen
  QObject::connect(home, &OffroadHome::openSettings, this, &HomeWindow::openSettings);
  // Add the home widget to the stack layout
  slayout->addWidget(home);

  // The onroad widget is the driving interface and is added to the stack layout
  onroad = new OnroadWindow(this);
  // When the map panel is requested to be opened in the onroad window, hide the sidebar
  QObject::connect(onroad, &OnroadWindow::mapPanelRequested, this, [=] { sidebar->hide(); });
  // Add the onroad widget to the stack layout
  slayout->addWidget(onroad);

  // The body widget is another screen, possibly for additional settings or information, added to the stack layout
  body = new BodyWindow(this);
  slayout->addWidget(body);

  // The driver_view widget shows the driver's view camera feed and is added to the stack layout
  driver_view = new DriverViewWindow(this);
  // When done with the driver view, switch back to the previous screen
  connect(driver_view, &DriverViewWindow::done, [=] { 
    showDriverView(false); 
  });
  slayout->addWidget(driver_view);

  // Indicates that the window's background is provided by the system, no need to paint it
  setAttribute(Qt::WA_NoSystemBackground);

  // Connect various UI state update signals to the respective slots in this window
  QObject::connect(uiState(), &UIState::uiUpdate, this, &HomeWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &HomeWindow::offroadTransition);
  // Ensure the sidebar is aware of the offroad transition state
  QObject::connect(uiState(), &UIState::offroadTransition, sidebar, &Sidebar::offroadTransition);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::showMapPanel(bool show) {
  onroad->showMapPanel(show);
}

void HomeWindow::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // switch to the generic robot UI
  if (onroad->isVisible() && !body->isEnabled() && sm["carParams"].getCarParams().getNotCar()) {
    body->setEnabled(true);
    slayout->setCurrentWidget(body);
  }
}

void HomeWindow::offroadTransition(bool offroad) {
  body->setEnabled(false);
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
}

void HomeWindow::showDriverView(bool show) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  sidebar->setVisible(show == false);
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Handle sidebar collapsing
  if ((onroad->isVisible() || body->isVisible()) && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
    params.putBool("Sidebar", sidebar->isVisible());
  }
}

void HomeWindow::mouseDoubleClickEvent(QMouseEvent* e) {
  HomeWindow::mousePressEvent(e);
  const SubMaster &sm = *(uiState()->sm);
  if (sm["carParams"].getCarParams().getNotCar()) {
    if (onroad->isVisible()) {
      slayout->setCurrentWidget(body);
    } else if (body->isVisible()) {
      slayout->setCurrentWidget(onroad);
    }
    showSidebar(false);
  }
}

// =======================================
// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 40);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(16);

  update_notif = new QPushButton(tr("UPDATE"));
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  date = new ElidedLabel();
  header_layout->addWidget(date, 0, Qt::AlignHCenter | Qt::AlignLeft);

  version = new ElidedLabel();
  header_layout->addWidget(version, 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget *home_widget = new QWidget(this);
  {
    QHBoxLayout *home_layout = new QHBoxLayout(home_widget);
    home_layout->setContentsMargins(0, 0, 0, 0);
    home_layout->setSpacing(30);

    // left: MapSettings/PrimeAdWidget
    QStackedWidget *left_widget = new QStackedWidget(this);
#ifdef ENABLE_MAPS
    left_widget->addWidget(new MapSettings);
#else
    left_widget->addWidget(new QWidget);
#endif
    left_widget->addWidget(new PrimeAdWidget);
    left_widget->addWidget(new DriveStats);
    left_widget->setStyleSheet("border-radius: 10px;");

    left_widget->setCurrentIndex(params.getBool("DriveStats") ? 2 : uiState()->hasPrime() ? 0 : 1);
    connect(uiState(), &UIState::primeChanged, [=](bool prime) {
      left_widget->setCurrentIndex(params.getBool("DriveStats") ? 2 : uiState()->hasPrime() ? 0 : 1);
    });

    home_layout->addWidget(left_widget, 1);

    // right: ExperimentalModeButton, SetupWidget
    QWidget* right_widget = new QWidget(this);
    QVBoxLayout* right_column = new QVBoxLayout(right_widget);
    right_column->setContentsMargins(0, 0, 0, 0);
    right_widget->setFixedWidth(750);
    right_column->setSpacing(30);

    ExperimentalModeButton *experimental_mode = new ExperimentalModeButton(this);
    QObject::connect(experimental_mode, &ExperimentalModeButton::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(experimental_mode, 1);

    SetupWidget *setup_widget = new SetupWidget;
    QObject::connect(setup_widget, &SetupWidget::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(setup_widget, 1);

    home_layout->addWidget(right_widget, 1);
  }
  center_layout->addWidget(home_widget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
      color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");

  // Set the model name
  MODEL_NAME = {
    {"los-angeles", "Los Angeles"},
    {"certified-herbalist", "Certified Herbalist"},
    {"recertified-herbalist", "Recertified Herbalist"},
  };
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  QString model = QString::fromStdString(params.get("Model"));

  date->setText(QLocale(uiState()->language.mid(5)).toString(QDateTime::currentDateTime(), "dddd, MMMM d"));
  version->setText(getBrand() + " v" + getVersion().left(14).trimmed() + " - " + MODEL_NAME[model]);

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + (alerts > 1 ? tr(" ALERTS") : tr(" ALERT")));
  }
}
