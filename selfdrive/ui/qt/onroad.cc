#include "selfdrive/ui/qt/onroad.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>

#include <QApplication>
#include <QDebug>
#include <QMouseEvent>

#include "common/swaglog.h"
#include "common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/maps/map_panel.h"
#endif

static void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity) {
  p.save(); // Save the QPainter's current state

  p.setRenderHint(QPainter::Antialiasing);
  
  // Draw the background ellipse with full opacity set by bg
  p.setOpacity(1.0); // Ensure the background is fully opaque
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);

  // Draw the icon pixmap with the specified opacity
  p.setOpacity(opacity); // Apply the specified opacity for the icon image
  p.drawPixmap(center - QPoint(img.width() / 2, img.height() / 2), img);

  p.restore(); // Restore the QPainter's state to what it was before modifications
}
static void drawIconRotate(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity, const int angle) {
  p.save();

  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  //p.save();
  p.translate(center);
  p.rotate(-angle);
  p.setOpacity(opacity);
  p.drawPixmap(-QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);

  p.restore();
}

// ==============================================
// OnroadWindow (main widget for the on-road UI)
OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent), scene(uiState()->scene) {
  // Vertical box layout that contains all the elements of the OnroadWindow.
  QVBoxLayout *main_layout_local = new QVBoxLayout(this);
  main_layout_local->setMargin(UI_BORDER_SIZE); // Set the outer margin of the layout to UI_BORDER_SIZE, which is 30 pixels.

  // A stacked layout that can contain multiple widgets and overlay them on top of each other.
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll); // Widgets in this layout will be stacked in the z-order.
  main_layout_local->addLayout(stacked_layout); // Add the stacked layout to the main vertical box layout.

  // This widget shows the annotated camera feed.
  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  // Wrapper for the split view, which holds multiple widgets horizontally.
  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0); // Set the internal margins of the layout to 0, so widgets use the full space.
  split->setSpacing(0); // Set the spacing between widgets in the layout to 0.
  split->addWidget(nvg); // Add the AnnotatedCameraWidget to the horizontal layout.

  // Check for environment variables that enable additional camera and map views.
  if (getenv("DUAL_CAMERA_VIEW")) {
    // Add a secondary camera view if the corresponding environment variable is set.
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, true, this);
    split->insertWidget(0, arCam); // Insert at the start of the horizontal layout.
  }

  if (getenv("MAP_RENDER_VIEW")) {
    // Add a map rendering view if the corresponding environment variable is set.
    CameraWidget *map_render = new CameraWidget("navd", VISION_STREAM_MAP, false, this);
    split->insertWidget(0, map_render); // Insert at the start of the horizontal layout.
  }

  // Add the split view wrapper to the stacked layout, making it part of the overlay.
  stacked_layout->addWidget(split_wrapper);

  // Create and configure the alerts widget.
  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true); // Alerts should not intercept mouse events.
  stacked_layout->addWidget(alerts); // Add the alerts widget to the stacked layout on top of the split wrapper.

  // Ensure that alerts are displayed in front of other widgets in the stacked layout.
  alerts->raise();

  // Optimize the paint event for opaque regions to improve performance.
  setAttribute(Qt::WA_OpaquePaintEvent);

  // Connect various signals from the UIState object to slots in this widget.
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::primeChanged, this, &OnroadWindow::primeChanged);

  // Configure a timer to simulate a click event after a certain period of time.
  QObject::connect(&clickTimer, &QTimer::timeout, this, [this]() {
    clickTimer.stop(); // Stop the timer to prevent repeated clicks.
    // Generate a fake mouse press event at the specified coordinates.
    QMouseEvent *event = new QMouseEvent(QEvent::MouseButtonPress, timeoutPoint, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::postEvent(this, event); // Post the fake event to the application for processing.
  });

  // In your OnroadWindow constructor or setup function, connect the signal to a new slot.
  connect(nvg, &AnnotatedCameraWidget::leadInfoUpdated, this, &OnroadWindow::handleLeadInfo);
}

void OnroadWindow::handleLeadInfo(const QString &info) {
  leadInfoLog = info;
  update(); // Trigger a repaint to display the new log
}


void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  alerts->updateAlert(alert);

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  nvg->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  Params params = Params();

  // FrogPilot clickable widgets
  bool widgetClicked = false;

  // Change cruise control increments button
  QRect maxSpeedRect(7, 25, 225, 225);
  bool isMaxSpeedClicked = maxSpeedRect.contains(e->pos());

  // Hide speed button
  QRect hideSpeedRect(rect().center().x() - 175, 50, 350, 350);
  bool isSpeedClicked = hideSpeedRect.contains(e->pos());

  // Speed limit confirmation buttons
  QSize size = this->size();
  QRect leftRect(0, 0, size.width() / 2, size.height());
  QRect rightRect = leftRect.translated(size.width() / 2, 0);
  bool isLeftSideClicked = leftRect.contains(e->pos()) && scene.speed_limit_changed;
  bool isRightSideClicked = rightRect.contains(e->pos()) && scene.speed_limit_changed;

  // Speed limit offset button
  QRect speedLimitRect(7, 250, 225, 225);
  bool isSpeedLimitClicked = speedLimitRect.contains(e->pos());

  if (isMaxSpeedClicked || isSpeedClicked || isSpeedLimitClicked) {
    if (isMaxSpeedClicked && scene.reverse_cruise_ui) {
      bool currentReverseCruise = scene.reverse_cruise;

      uiState()->scene.reverse_cruise = !currentReverseCruise;
      params.putBoolNonBlocking("ReverseCruise", !currentReverseCruise);

    } else if (isSpeedClicked && scene.hide_speed_ui) {
      bool currentHideSpeed = scene.hide_speed;

      uiState()->scene.hide_speed = !currentHideSpeed;
      params.putBoolNonBlocking("HideSpeed", !currentHideSpeed);
    } else if (isSpeedLimitClicked && scene.show_slc_offset_ui) {
      bool currentShowSLCOffset = scene.show_slc_offset;

      uiState()->scene.show_slc_offset = !currentShowSLCOffset;
      params.putBoolNonBlocking("ShowSLCOffset", !currentShowSLCOffset);
    }

    widgetClicked = true;
  } else if (isLeftSideClicked || isRightSideClicked) {
    bool slcConfirmed = isLeftSideClicked && !scene.right_hand_drive || isRightSideClicked && scene.right_hand_drive;
    paramsMemory.putBool("SLCConfirmed", slcConfirmed);
    paramsMemory.putBool("SLCConfirmedPressed", true);

    widgetClicked = true;
  // If the click wasn't for anything specific, change the value of "ExperimentalMode"
  } else if (scene.experimental_mode_via_screen && e->pos() != timeoutPoint) {
    if (clickTimer.isActive()) {
      clickTimer.stop();

      if (scene.conditional_experimental) {
        int override_value = (scene.conditional_status >= 1 && scene.conditional_status <= 4) ? 0 : scene.conditional_status >= 5 ? 3 : 4;
        paramsMemory.putIntNonBlocking("CEStatus", override_value);
      } else {
        bool experimentalMode = params.getBool("ExperimentalMode");
        params.putBoolNonBlocking("ExperimentalMode", !experimentalMode);
      }

    } else {
      clickTimer.start(500);
    }
    widgetClicked = true;
  }

#ifdef ENABLE_MAPS
  if (map != nullptr && !widgetClicked) {
    // Switch between map and sidebar when using navigate on openpilot
    bool sidebarVisible = geometry().x() > 0;
    bool show_map = uiState()->scene.navigate_on_openpilot ? sidebarVisible : !sidebarVisible;
    if (!clickTimer.isActive()) {
      map->setVisible(show_map && !map->isVisible());
      if (scene.full_map) {
        map->setFixedWidth(width());
      } else {
        map->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
      }
    }
  }
#endif
  // propagation event to parent(HomeWindow)
  if (!widgetClicked) {
    QWidget::mousePressEvent(e);
  }
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->hasPrime() || !MAPBOX_TOKEN.isEmpty())) {
      auto m = new MapPanel(get_mapbox_settings());
      map = m;

      QObject::connect(m, &MapPanel::mapPanelRequested, this, &OnroadWindow::mapPanelRequested);
      QObject::connect(nvg->map_settings_btn, &MapSettingsButton::clicked, m, &MapPanel::toggleMapSettings);
      nvg->map_settings_btn->setEnabled(true);

      m->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
      split->insertWidget(0, m);

      // hidden by default, made visible when navRoute is published
      m->setVisible(false);
    }
  }
#endif

  alerts->updateAlert({});
}

void OnroadWindow::primeChanged(bool prime) {
#ifdef ENABLE_MAPS
  if (map && (!prime && MAPBOX_TOKEN.isEmpty())) {
    nvg->map_settings_btn->setEnabled(false);
    nvg->map_settings_btn->setVisible(false);
    map->deleteLater();
    map = nullptr;
  }
#endif
}
// OnroadWindow's event handler for paint events. This is called when the part of the widget needs to be redrawed, 
// such as when the window is first shown, when it's resized, or when update() is called.
void OnroadWindow::paintEvent(QPaintEvent *event) {
  // QPainter object is used for all drawing operations
  QPainter p(this);

  // Fill the entire widget area with the current background color. rect() returns the QRect representing the widget's area.
  // Here, rect() would return the rectangle that covers the entire OnroadWindow widget.
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));

  QFont monospaceFont("Monospace", 22, QFont::DemiBold); // Adjust size as necessary
  monospaceFont.setStyleHint(QFont::TypeWriter);
  p.setFont(monospaceFont);
  p.setRenderHint(QPainter::TextAntialiasing);
  p.setPen(Qt::white);

  QFontMetrics metrics(monospaceFont);
  int textHeight = metrics.capHeight();
  //int topTextWidth = metrics.horizontalAdvance(leadInfoLog);

  // Get the current rectangle area of the widget
  QRect currentRect = rect();
  // Define the left margin for the text placement
  int leftMargin = 10;  // center : currentRect.width() - topTextWidth) / 2
  // at the bottom ----------------------------------------------------------------
  // Check if the frames per second (fps) counter should be displayed on the screen
  if (scene.fps_counter) {
    // If so, first update the fps counter to get the latest values
    updateFPSCounter();

    // Construct a string that will display the fps information
    QString fpsDisplayString = QString("FPS: %1 (%2) | Min: %3 | Max: %4 | Avg: %5")
        .arg(fps, 0, 'f', 2) // Current fps with 2 decimal places
        .arg(paramsMemory.getInt("CameraFPS")) // Camera fps fetched from the memory parameters
        .arg(minFPS, 0, 'f', 2) // Minimum recorded fps with 2 decimal places
        .arg(maxFPS, 0, 'f', 2) // Maximum recorded fps with 2 decimal places
        .arg(avgFPS, 0, 'f', 2); // Average fps over some interval

    // Set the y position to just above the bottom of the widget area, adjusted by a margin
    int yPos = currentRect.bottom() - 5;

    // Draw the fps information text at the defined x and y position
    p.drawText(leftMargin, yPos, fpsDisplayString);
  }
  // Draw the leadInfoLog at the top of the window --------------------------------------
  if (!leadInfoLog.isEmpty()) {
    //monospaceFont.setStretch(/*80*/QFont::SemiCondensed); // Make the font more condensed
    #if 0
    QFont::UltraCondensed: 50%
    QFont::ExtraCondensed: 62.5%
    QFont::Condensed: 75%
    QFont::SemiCondensed: 87.5%
    QFont::Unstretched: 100% (the default width)
    QFont::SemiExpanded: 112.5%
    QFont::Expanded: 125%
    QFont::ExtraExpanded: 150%
    QFont::UltraExpanded: 200%
    #endif
    //p.setFont(monospaceFont);
    int yPosTop = currentRect.top() + UI_BORDER_SIZE - (UI_BORDER_SIZE - textHeight)/2; // Position the text below the top edge
    // Draw the lead info log (signal : leadInfoUpdated)
    p.drawText(leftMargin, yPosTop, leadInfoLog); // Y : where the baseline of the text begins
  }
}

void OnroadWindow::updateFPSCounter() {
  qint64 currentMillis = QDateTime::currentMSecsSinceEpoch();
  std::queue<std::pair<qint64, double>> fpsQueue = std::queue<std::pair<qint64, double>>();

  minFPS = qMin(minFPS, fps);
  maxFPS = qMax(maxFPS, fps);
  fpsQueue.push({currentMillis, fps});

  while (!fpsQueue.empty() && currentMillis - fpsQueue.front().first > 60000) {
    fpsQueue.pop();
  }

  if (!fpsQueue.empty()) {
    double totalFPS = 0;
    std::queue<std::pair<qint64, double>> tempQueue = fpsQueue;
    while (!tempQueue.empty()) {
      totalFPS += tempQueue.front().second;
      tempQueue.pop();
    }
    avgFPS = totalFPS / fpsQueue.size();
  }
}

// ==========================
// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a) {
  if (!alert.equal(a)) {
    alert = a;
    update();
  }
}
// OnroadAlerts
void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE || scene.show_driver_camera) {
    return; // Do not draw the alert if the size is NONE or if the driver camera is shown.
  }
  
  // Define the height of the alert based on the alert size.
  static std::map<cereal::ControlsState::AlertSize, const int> alert_heights = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_heights[alert.size];

  // Define margins and radius for the alert box.
  int margin = 40;
  int radius = 30;
  int offset = scene.always_on_lateral || scene.conditional_experimental || scene.road_name_ui ? 25 : 0;
  if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    margin = 0;
    radius = 0;
    offset = 0;
  }

  // Define the rectangle in which the alert will be drawn.
  QRect r = QRect(0 + margin, height() - h + margin - offset, width() - margin * 2, h - margin * 2);

  QPainter p(this);

  // Set the background color to semi-transparent yellow (50% opacity) and no border.
  p.setPen(Qt::NoPen);
  QColor semiTransparentYellow(0xff, 0xff, 0x00, 127); // Yellow with 50% opacity.
  p.setBrush(semiTransparentYellow);
  p.drawRoundedRect(r, radius, radius); // Draw the rounded rectangle with the yellow background.

  // Set up the text to be drawn.
  const QPoint c = r.center();
  p.setPen(QColor(0x00, 0x00, 0x00)); // Set the pen to black for the text color.
  p.setRenderHint(QPainter::TextAntialiasing);

  // Draw the text based on the size of the alert.
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    p.setFont(InterFont(74, QFont::DemiBold));
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    p.setFont(InterFont(88, QFont::Bold));
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    p.setFont(InterFont(66));
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    p.setFont(InterFont(l ? 132 : 177, QFont::Bold));
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    p.setFont(InterFont(88));
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// ==========================
// AnnotatedCameraWidget
// Constructor for AnnotatedCameraWidget, which is a window displaying the camera view and other overlays.
AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent)
  : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), // Initialize a filter to smooth out the frames per second display
    CameraWidget("camerad", type, true, parent), // Call the base class constructor with parameters for camera setup
    scene(uiState()->scene) // Initialize scene with the current UI state's scene
{
  // Create a publisher to send debug messages to the 'uiDebug' channel
  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"uiDebug"});

  // Create and set up the corner container widgets
  topLeftWidget = new QWidget(this);
  topRightWidget = new QWidget(this);
  bottomLeftWidget = new QWidget(this);
  bottomRightWidget = new QWidget(this);
#if 0
  topLeftWidget->setStyleSheet("border: 3px solid blue;");
  topRightWidget->setStyleSheet("border: 3px solid green;");
  bottomLeftWidget->setStyleSheet("border: 3px solid yellow;");
  bottomRightWidget->setStyleSheet("border: 3px solid orange;");
#endif
  // Initialize the widgets
  brake_disc_icons = new BrakeDiscIcons(uiState(), this);
  tire_pressure_icons = new TirePressureIcons(uiState(), this);
  recorder_btn = new ScreenRecorder(this);
  experimental_btn = new ExperimentalButton(this);
  pedal_icons = new PedalIcons(this);
  map_settings_btn = new MapSettingsButton(this);
  personality_btn = new PersonalityButton(this);
  driver_face_icon = new DriverFaceIcon(uiState(), this);
  compass_img = new Compass(this);
#if 0
  // Apply borders to icon widgets
  QString iconWidgetStyle = "border: 2px solid magenta;";
  brake_disc_icons->setStyleSheet(iconWidgetStyle);
  tire_pressure_icons->setStyleSheet(iconWidgetStyle);
  recorder_btn->setStyleSheet(iconWidgetStyle);
  experimental_btn->setStyleSheet(iconWidgetStyle);
  pedal_icons->setStyleSheet(iconWidgetStyle);
  map_settings_btn->setStyleSheet(iconWidgetStyle);
  personality_btn->setStyleSheet(iconWidgetStyle);
  driver_face_icon->setStyleSheet(iconWidgetStyle);
  compass_img->setStyleSheet(iconWidgetStyle);
#endif
  // top right corner ----------------------------------------------------------
  // Set up the layout for the topRightWidget with two lines of icons
  QVBoxLayout *topRightMainLayout = new QVBoxLayout();  // default : up-aligned.

  // First line of icons
  QHBoxLayout *topRightFirstLineLayout = new QHBoxLayout();
  topRightFirstLineLayout->addStretch(); // right-aligned. default : left-aligned.
  topRightFirstLineLayout->addWidget(brake_disc_icons);
  topRightFirstLineLayout->addWidget(tire_pressure_icons);
  topRightFirstLineLayout->addWidget(recorder_btn);
  topRightFirstLineLayout->addWidget(experimental_btn);

  // Second line of icons
  QHBoxLayout *topRightSecondLineLayout = new QHBoxLayout();
  // Add a stretch before the pedal icon to push it to the right
  topRightSecondLineLayout->addStretch(); // default : left-aligned.
  topRightSecondLineLayout->addWidget(pedal_icons);

  // Add the first and second line layouts to the main layout
  topRightMainLayout->addLayout(topRightFirstLineLayout);
  topRightMainLayout->addLayout(topRightSecondLineLayout);
  // Add a stretch to push all content to the top
  topRightMainLayout->addStretch();

  // Set the main layout to the topRightWidget
  topRightWidget->setLayout(topRightMainLayout);

  // bottom right corner ----------------------------------------------------------
  // Set up the layout for the bottomRightWidget
  bottomRightLayout = new QVBoxLayout();
  bottomRightLayout->addStretch(); // down-aligned. default : up-aligned.
  bottomRightLayout->addWidget(map_settings_btn);
  bottomRightLayout->addWidget(compass_img);
  bottomRightWidget->setLayout(bottomRightLayout);

  // bottom left corner ----------------------------------------------------------
  bottomLeftMainLayout = new QVBoxLayout();
  bottomLeft1stFloorLayout = new QHBoxLayout(); // default : left-aligned.
  bottomLeft1stFloorLayout->addWidget(personality_btn);  
  bottomLeft1stFloorLayout->addWidget(driver_face_icon);
  #if 0
  // Create a spacer item with an expanding vertical size policy
  QSpacerItem* bottomSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
  // Add the spacer item to the bottomLeftMainLayout first
  bottomLeftMainLayout->addSpacerItem(bottomSpacer);
  #endif
  // Then add the first floor layout
  bottomLeftMainLayout->addLayout(bottomLeft1stFloorLayout);
  bottomLeftWidget->setLayout(bottomLeftMainLayout);

  // -----------------------------------------------------------------------------
  // Now, set up the main layout for the widget
  mainLayout = new QGridLayout(this);

#if 1
  int liftAmount = 50;  // Amount to lift the widget by from the bottom
  // Alternatively, if you want to lift both bottom widgets up by 25 pixels from the window's bottom edge:
  mainLayout->setContentsMargins(0, 0, 0, liftAmount); // Left, Top, Right, Bottom margins.
#else
  //offset = ((alwaysOnLateral || conditionalExperimental || roadNameUI) ? 25 : 0);
#endif

  // Add corner widgets to the main layout
  mainLayout->addWidget(topLeftWidget, 0, 0, Qt::AlignTop | Qt::AlignLeft);
  mainLayout->addWidget(topRightWidget, 0, 1, Qt::AlignTop | Qt::AlignRight);
  mainLayout->addWidget(bottomLeftWidget, 1, 0, Qt::AlignBottom | Qt::AlignLeft);
  mainLayout->addWidget(bottomRightWidget, 1, 1, Qt::AlignBottom | Qt::AlignRight);

  // Adjust stretch factors as needed to position the corner widgets at the edges
  mainLayout->setRowStretch(0, 1); // Add stretch to push the bottom widgets to the bottom
  mainLayout->setRowStretch(1, 0); // No stretch for the bottom row
  mainLayout->setColumnStretch(0, 1); // Add stretch to push the right widgets to the right
  mainLayout->setColumnStretch(1, 0); // No stretch for the right column

  // Set the main layout for the AnnotatedCameraWidget
  setLayout(mainLayout);
  updateWidgetAlignment(); // Initial alignment update

  // Call a function to initialize additional widgets specific to FrogPilot
  initializeFrogPilotWidgets();
}

void AnnotatedCameraWidget::updateWidgetAlignment() {
  static bool previousRightHandDM = !rightHandDM;
  // Check if rightHandDM has changed
  if (rightHandDM != previousRightHandDM) {
    // Get the map settings layout for the bottom left or bottom right based on rightHandDM
    QVBoxLayout* targetLayout = rightHandDM ? bottomLeftMainLayout : bottomRightLayout;
    if (map_settings_btn->parentWidget() != nullptr) {
      // Remove map_settings_btn from its current layout
      map_settings_btn->parentWidget()->layout()->removeWidget(map_settings_btn);
    }
    // Add the map_settings_btn directly to the target layout without a spacer
    targetLayout->insertWidget(0, map_settings_btn); // Insert at the beginning of the layout
    //targetLayout->addWidget(map_settings_btn, 0, Qt::AlignTop);

    // Save the new state as previous state
    previousRightHandDM = rightHandDM;
  }
}



// This function is responsible for initializing the widgets specific to FrogPilot in the AnnotatedCameraWidget.
void AnnotatedCameraWidget::initializeFrogPilotWidgets() {
  // Define a custom themes configuration, possibly for theming the UI with different color schemes and visual elements.
  // Each theme has an ID, name, an unknown numeric value (perhaps related to its selection order or priority),
  // a base color, and a gradient configuration with three points defined by a position and a QBrush with a color and opacity.
  themeConfiguration = {
    // Entry for the "frog_theme" with a green color scheme.
    {1, {"frog_theme", 4, QColor(23, 134, 68, 242), {{0.0, QBrush(QColor::fromHslF(144 / 360., 0.71, 0.31, 0.9))},
                                                      {0.5, QBrush(QColor::fromHslF(144 / 360., 0.71, 0.31, 0.5))},
                                                      {1.0, QBrush(QColor::fromHslF(144 / 360., 0.71, 0.31, 0.1))}}}},
    // Entry for the "tesla_theme" with a blue color scheme.
    {2, {"tesla_theme", 4, QColor(0, 72, 255, 255), {{0.0, QBrush(QColor::fromHslF(223 / 360., 1.0, 0.5, 0.9))},
                                                      {0.5, QBrush(QColor::fromHslF(223 / 360., 1.0, 0.5, 0.5))},
                                                      {1.0, QBrush(QColor::fromHslF(223 / 360., 1.0, 0.5, 0.1))}}}},
    // Entry for the "stalin_theme" with a red color scheme.
    {3, {"stalin_theme", 6, QColor(255, 0, 0, 255), {{0.0, QBrush(QColor::fromHslF(0 / 360., 1.0, 0.5, 0.9))},
                                                      {0.5, QBrush(QColor::fromHslF(0 / 360., 1.0, 0.5, 0.5))},
                                                      {1.0, QBrush(QColor::fromHslF(0 / 360., 1.0, 0.5, 0.1))}}}}
  };

  // Initialize a timer that will be used for animating the turn signal, assuming there's a turn signal animation.
  animationTimer = new QTimer(this);
  connect(animationTimer, &QTimer::timeout, this, [this] {
    animationFrameIndex = (animationFrameIndex + 1) % totalFrames; // Cycle through the frames of the animation.
  });

  // Set up a timer that updates the screen recorder button, potentially to show recording status or time.
  QTimer *record_timer = new QTimer(this);
  connect(record_timer, &QTimer::timeout, this, [this]() {
    if (recorder_btn) {
      recorder_btn->update_screen(); // Call a function to update the screen recorder button.
    }
  });
  // Start the timer with an interval set to refresh at the UI's frame rate.
  record_timer->start(1000 / UI_FREQ);
}
void AnnotatedCameraWidget::updateState(const UIState &s) {
  const int SET_SPEED_NA = 255;
  const SubMaster &sm = *(s.sm);

  const bool cs_alive = sm.alive("controlsState");
  const bool nav_alive = sm.alive("navInstruction") && sm["navInstruction"].getValid();
  const auto cs = sm["controlsState"].getControlsState();
  const auto car_state = sm["carState"].getCarState();
  const auto nav_instruction = sm["navInstruction"].getNavInstruction();

  // Handle older routes where vCruiseCluster is not set
  // Choose between getVCruiseCluster and getVCruise based on if getVCruiseCluster returns 0.0
  float v_cruise = cs.getVCruiseCluster() == 0.0 ? cs.getVCruise() : cs.getVCruiseCluster();
  // If cs (car state) is alive (active), use v_cruise as setSpeed; otherwise, use a predefined "not available" value
  setSpeed = cs_alive ? v_cruise : SET_SPEED_NA;
  // Determine if cruise control is set by checking if setSpeed is positive and not equal to the "not available" value
  is_cruise_set = setSpeed > 0 && (int)setSpeed != SET_SPEED_NA;
  // If cruise control is set and the unit is not metric, convert setSpeed from km/h to mph
  if (is_cruise_set && !s.scene.is_metric) {
    setSpeed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  // Update v_ego_cluster_seen if it's previously seen or if getVEgoCluster returns a non-zero value
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  // Choose between getVEgoCluster and getVEgo based on v_ego_cluster_seen and if wheel speed is not being used
  float v_ego = ((v_ego_cluster_seen && !scene.wheel_speed) ? car_state.getVEgoCluster() : car_state.getVEgo());
  // If car state is alive, use max of 0.0 and v_ego as speed; otherwise, use 0.0
  speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  // Convert speed to the appropriate unit based on whether the scene is metric
  speed *= s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH;

  // Obtain the speed limit sign from navigation instructions
  auto speed_limit_sign = nav_instruction.getSpeedLimitSign();
  // Determine speed limit based on various conditions such as whether speed limit control is overridden,
  // if speed limit control is active, or if navigation is alive
  speedLimit = slcOverridden 
                ? scene.speed_limit_overridden_speed 
                : speedLimitController 
                  ? slcSpeedLimit 
                  : nav_alive 
                    ? nav_instruction.getSpeedLimit() 
                    : 0.0;
  // Convert the speed limit to the appropriate unit based on the scene's metric system
  speedLimit *= (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
  // If speed limit control is active and not overridden, adjust speed limit by subtracting the offset, if applicable
  if (speedLimitController && !slcOverridden) {
    speedLimit = speedLimit - (showSLCOffset ? slcSpeedLimitOffset : 0);
  }


  has_us_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::MUTCD) || (speedLimitController && !useViennaSLCSign);
  has_eu_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::VIENNA) || (speedLimitController && useViennaSLCSign);
  is_metric = s.scene.is_metric;
  speedUnit =  s.scene.is_metric ? tr("km/h") : tr("mph");

  hideBottomIcons = (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE || customSignals && (turnSignalLeft || turnSignalRight)) || fullMapOpen || showDriverCamera;
  status = s.status;

}
void AnnotatedCameraWidget::drawHud(QPainter &p) {
  p.save();

  // Header gradient
  drawHeaderGradient(p);

  // Set speed (cruise control speed) && Speed limit sign
  drawSpeedLimitSign(p);

  // current speed
  if (!scene.hide_speed && !fullMapOpen) {
    drawSpeed(p);
  }

  // Draw the car state values
  drawCarStateOverlay(p);

  p.restore();

}
// Enhances the visibility of text and icons at the top. 
// This is important as the top area can be hard to see against a bright sky.
void AnnotatedCameraWidget::drawHeaderGradient(QPainter &p) {
  p.save();  // Save the QPainter's current state

  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), UI_HEADER_HEIGHT, bg);

  p.restore();
}
void AnnotatedCameraWidget::drawSpeedLimitSign(QPainter &p) {
  p.save();  // Save the QPainter's current state
  
  QString speedLimitStr = (speedLimit > 1) ? QString::number(std::nearbyint(speedLimit)) : "–";
  QString speedLimitOffsetStr = slcSpeedLimitOffset == 0 ? "–" : QString::number(slcSpeedLimitOffset, 'f', 0).prepend(slcSpeedLimitOffset > 0 ? "+" : "");
  QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(setSpeed - cruiseAdjustment)) : "–";

  // Draw outer box + border to contain set speed and speed limit
  const int sign_margin = 12;
  const int us_sign_height = 186;
  const int eu_sign_size = 176;

  const QSize default_size = {172, 204};
  QSize set_speed_size = default_size;
  if (is_metric || has_eu_speed_limit) set_speed_size.rwidth() = UI_SPEEDLIMIT_WIDTH_METRIC;
  if (has_us_speed_limit && speedLimitStr.size() >= 3) set_speed_size.rwidth() = UI_SPEEDLIMIT_WIDTH_ETC;

  if (has_us_speed_limit) set_speed_size.rheight() += us_sign_height + sign_margin;
  else if (has_eu_speed_limit) set_speed_size.rheight() += eu_sign_size + sign_margin;

  int top_radius = 32;
  int bottom_radius = has_eu_speed_limit ? 100 : 32;

  QRect set_speed_rect(QPoint(UI_CAM_WIDGET_MARGIN, UI_CAM_WIDGET_MARGIN), set_speed_size); // top left corner
  if (is_cruise_set && cruiseAdjustment) {
    float transition = qBound(0.0f, 4.0f * (cruiseAdjustment / setSpeed), 1.0f);
    QColor min = whiteColor(75);
    QColor max = vtscControllingCurve ? redColor(75) : greenColor(75);

    p.setPen(QPen(QColor::fromRgbF(
      min.redF()   + transition * (max.redF()   - min.redF()),
      min.greenF() + transition * (max.greenF() - min.greenF()),
      min.blueF()  + transition * (max.blueF()  - min.blueF())
    ), 6));
  } else if (scene.reverse_cruise) {
    p.setPen(QPen(QColor(0, 150, 255), 6));
  } else {
    p.setPen(QPen(whiteColor(75), 6));
  }
  p.setBrush(blackColor(166));
  drawRoundedRect(p, set_speed_rect, top_radius, top_radius, bottom_radius, bottom_radius);

  // Draw MAX
  QColor max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
  QColor set_speed_color = whiteColor();
  // Check if cruise control is enabled
  if (is_cruise_set) {
    // Check if the cruise control status is disengaged
    if (status == STATUS_DISENGAGED) {
      // Set the maximum color to white
      max_color = whiteColor();
    } 
    // Check if the cruise control status is overridden
    else if (status == STATUS_OVERRIDE) {
      // Set the maximum color to a specific shade of grey
      max_color = QColor(0x91, 0x9b, 0x95, 0xff);
    } 
    // Check if the speed limit is greater than 0
    else if (speedLimit > 0) {
      // Define a lambda function to interpolate between colors based on the current speed limit
      auto interp_color = [=](QColor c1, QColor c2, QColor c3) {
        // If speed limit is positive, interpolate colors based on set speed and speed limit thresholds; otherwise, return the first color
        return speedLimit > 0 ? interpColor(setSpeed, {speedLimit + 5, speedLimit + 15, speedLimit + 25}, {c1, c2, c3}) : c1;
      };
      // Interpolate the max color between the current max color, a light orange, and a light red based on the current set speed
      max_color = interp_color(max_color, QColor(0xff, 0xe4, 0xbf), QColor(0xff, 0xbf, 0xbf));
      // Interpolate the set speed color between dark orange and red based on the current set speed
      set_speed_color = interp_color(set_speed_color, QColor(0xff, 0x95, 0x00), QColor(0xff, 0x00, 0x00));
    }
  } else {
    max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
    set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
  }
  p.setFont(InterFont(40, QFont::DemiBold));
  p.setPen(max_color);
  p.drawText(set_speed_rect.adjusted(0, 27, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("MAX"));
  p.setFont(InterFont(90, QFont::Bold));
  p.setPen(set_speed_color);
  p.drawText(set_speed_rect.adjusted(0, 77, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);

  const QRect sign_rect = set_speed_rect.adjusted(sign_margin, default_size.height(), -sign_margin, -sign_margin);
  // US/Canada (MUTCD style) sign
  if (has_us_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawRoundedRect(sign_rect, 24, 24);
    p.setPen(QPen(blackColor(), 6));
    p.drawRoundedRect(sign_rect.adjusted(9, 9, -9, -9), 16, 16);

    p.save();
    p.setOpacity(slcOverridden ? 0.25 : 1.0);
    if (showSLCOffset) {
      p.setFont(InterFont(28, QFont::DemiBold));
      p.drawText(sign_rect.adjusted(0, 22, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("LIMIT"));
      p.setFont(InterFont(70, QFont::Bold));
      p.drawText(sign_rect.adjusted(0, 51, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitStr);
      p.setFont(InterFont(50, QFont::DemiBold));
      p.drawText(sign_rect.adjusted(0, 120, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitOffsetStr);
    } else {
      p.setFont(InterFont(28, QFont::DemiBold));
      p.drawText(sign_rect.adjusted(0, 22, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("SPEED"));
      p.drawText(sign_rect.adjusted(0, 51, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("LIMIT"));
      p.setFont(InterFont(70, QFont::Bold));
      p.drawText(sign_rect.adjusted(0, 85, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitStr);
    }
    p.restore();
  }

  // EU (Vienna style) sign
  if (has_eu_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawEllipse(sign_rect);
    p.setPen(QPen(Qt::red, 20));
    p.drawEllipse(sign_rect.adjusted(16, 16, -16, -16));

    p.save();
    p.setOpacity(slcOverridden ? 0.25 : 1.0);
    p.setPen(blackColor());
    if (speedLimitController && showSLCOffset && !slcOverridden) {
      p.setFont(InterFont((speedLimitStr.size() >= 3) ? 60 : 70, QFont::Bold));
      p.drawText(sign_rect.adjusted(0, -25, 0, 0), Qt::AlignCenter, speedLimitStr);
      p.setFont(InterFont(40, QFont::DemiBold));
      p.drawText(sign_rect.adjusted(0, 100, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitOffsetStr);
    } else {
      p.setFont(InterFont((speedLimitStr.size() >= 3) ? 60 : 70, QFont::Bold));
      p.drawText(sign_rect, Qt::AlignCenter, speedLimitStr);
    }
    p.restore();  // Restore the QPainter's state before exiting the function
  }

  p.restore();
}

void AnnotatedCameraWidget::drawSpeed(QPainter &p) {
  p.save(); // Save the current state of the painter

  // Set dynamic color based on acceleration
  const SubMaster &sm = *(uiState()->sm);
  const auto car_state = sm["carState"].getCarState();
  float accel = car_state.getAEgo();

  QColor color = whiteColor(230); // Start with white color for text
  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    color = QColor(a, a, 255, 230); // bluish
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    color = QColor(255, a, a, 230); // reddish
  }

  // Prepare the strings for display
  QString speedStr = QString::number(static_cast<int>(std::nearbyint(speed)));
  QString speedUnitStr = speedUnit;

  // Set up fonts and metrics for text size calculation
  QFont speedFont = InterFont(176, QFont::Bold);
  QFont unitFont = InterFont(66, QFont::Normal);
  p.setFont(speedFont);
  QFontMetrics speedMetrics(speedFont);
  QFontMetrics unitMetrics(unitFont);

  // Calculate rectangles for speed and unit with padding
  int rectWidth = std::max(speedMetrics.horizontalAdvance(speedStr), unitMetrics.horizontalAdvance(speedUnitStr)) + 40; // Extra padding
  int speedRectHeight = speedMetrics.height() + 0;
  int unitRectHeight = unitMetrics.height() + 10;

  // Position rectangles
  int x = rect().center().x() - (rectWidth / 2);
  int speedY = 50;
  int unitY = speedY + speedRectHeight + 0;

  // Draw speed text with dynamic color
  QRect speedRect(x, speedY, rectWidth, speedRectHeight); // (x, y) width, height
  p.setPen(QPen(color));
  p.drawText(speedRect, Qt::AlignCenter, speedStr);

  // Draw unit text
  QRect unitRect(x, unitY, rectWidth, unitRectHeight);
  p.setFont(unitFont);
  p.setPen(QPen(is_metric ? whiteColor() : yellowColor())); // Color for unit based on metric or imperial
  p.drawText(unitRect, Qt::AlignCenter, speedUnitStr);

  p.restore(); // Restore the painter's state
}

void AnnotatedCameraWidget::drawCarStateOverlay(QPainter &p) {
  if(mapOpen) return;

  p.save(); // Save the current state of the painter

  const SubMaster &sm = *(uiState()->sm);
  const auto car_state = sm["carState"].getCarState();
  const auto cruise_state = car_state.getCruiseState();

  // Create and set a monospaced font for consistent text spacing and alignment
  QFont monospaceFont("Monospace", 20, QFont::Normal); // Configure the font
  monospaceFont.setStyleHint(QFont::TypeWriter); // Suggest using a typewriter-style monospaced font
  p.setFont(monospaceFont); // Apply the font to the QPainter
  QFontMetrics fm(monospaceFont);

  int marginX = btn_size + 30; // width of icons in the right (vertical)
  int marginY = btn_size + 30; // height of icons in the top (horizontal)
  int spacing = fm.height() + 0;
  int rectWidth = 450; // Adjust as needed to fit longer strings
  int rectX1 = width() - rectWidth - marginX; // Position to the right
  int rectY = marginY; // Position from top

  // Create a semi-transparent rectangle
  int rectHeight1 = spacing * 25; // Adjust based on the number of items you're displaying
  QRect rect1(rectX1, rectY, rectWidth, rectHeight1);
  
  int rectHeight2 = spacing * 15; // Height for 20 lines
  int rectX2 = 0 + fmax(UI_SPEEDLIMIT_WIDTH_METRIC, UI_SPEEDLIMIT_WIDTH_ETC);//fm.horizontalAdvance('A')*16;
  QRect rect2(rectX2, rectY, rectWidth, rectHeight2);
  #if 0
  {//fill the rectangle
    QColor semiTransparentColor(0, 0, 0, 100); // Adjust the alpha for transparency
    // Draw semi-transparent rectangle with a border
    p.fillRect(rect, semiTransparentColor);
  }
  #endif

  // Prepare to draw text
  p.setPen(QColor(255, 255, 255)); // White text

  // Set initial position for text
  int lineX = rectX1; // Use full width starting from the left edge
  int lineY = rectY;

  // Helper lambda to draw text and increment line position
  auto drawTextLine = [&](const QString &text) {
    QRect textRect(lineX, lineY, rectWidth, spacing);
    p.drawText(textRect, Qt::AlignLeft | Qt::AlignVCenter, text);
    lineY += spacing; // Move to the next line position
  };

  // Assuming car_state.getVEgo() gives you the speed in m/s
  drawTextLine(QString("Speed    : %1 km/h").arg(static_cast<int>(std::round(car_state.getVEgo()        * MS_TO_KPH)), 3, 10, QChar(' ')));
  drawTextLine(QString("  Raw    : %1 km/h").arg(static_cast<int>(std::round(car_state.getVEgoRaw()     * MS_TO_KPH)), 3, 10, QChar(' ')));
  drawTextLine(QString("  Cluster: %1 km/h").arg(static_cast<int>(std::round(car_state.getVEgoCluster() * MS_TO_KPH)), 3, 10, QChar(' ')));

  drawTextLine(QString("Gas      : %1").arg(car_state.getGas()));  
  drawTextLine(QString("  Pressed: %1").arg(car_state.getGasPressed() ? "Yes" : "-"));
  drawTextLine(QString("  Accel  : %1 m/s²").arg(car_state.getAEgo()));

  drawTextLine(QString("Brake Pressed : %1").arg(car_state.getBrakePressed() ? "Yes" : "-"));
  drawTextLine(QString("  Brake Lights: %1").arg(car_state.getBrakeLights() ? "ON" : "Off"));
  drawTextLine(QString("  Regenerative: %1").arg(car_state.getRegenBraking() ? "Yes" : "-"));
  drawTextLine(QString("  Standstill  : %1").arg(car_state.getStandstill() ? "Stopped" : "-"));
  drawTextLine(QString("  Hold Active : %1").arg(car_state.getBrakeHoldActive() ? "Yes" : "-"));
  drawTextLine(QString("  Auto Hold   : %1").arg(car_state.getAutoHold()? "Yes" : "-"));
  drawTextLine(QString("  Parking     : %1").arg(car_state.getParkingBrake() ? "Engaged" : "Disengaged"));

  drawTextLine(QString("Longitudinal : %1").arg(scene.longitudinal_control ? "Yes" : "-")); // CP.openpilotLongitudinalControl
  drawTextLine(QString("  Cruise GapAdjust : %1").arg(cruise_state.getGapAdjust()));
  drawTextLine(QString("  Cruise StandStill: %1").arg(cruise_state.getStandstill() ? "Yes" : "-"));


  drawTextLine(QString("Cruise : %1").arg(cruise_state.getEnabled() ? "Yes" : "-"));
  drawTextLine(QString("  Available: %1").arg(cruise_state.getAvailable() ? "Yes" : "-"));
  drawTextLine(QString("  Adaptive : %1").arg(cruise_state.getNonAdaptive() ? "-" : "Yes"));
  drawTextLine(QString("  Speed    : %1 km/h").arg(static_cast<int>(std::round(cruise_state.getSpeed()       * MS_TO_KPH)), 3, 10, QChar(' ')));
  drawTextLine(QString("    Cluster: %1 km/h").arg(static_cast<int>(std::round(cruise_state.getSpeedCluster()* MS_TO_KPH)), 3, 10, QChar(' ')));
  drawTextLine(QString("    Offset : %1 km/h").arg(static_cast<int>(std::round(cruise_state.getSpeedOffset() * MS_TO_KPH)), 3, 10, QChar(' ')));
  
  // --------------------------------------
  // Set initial position for text in rect2
  lineX = rectX2;
  lineY = rectY;

  drawTextLine(QString("Nav Speed Limit: %1").arg(car_state.getNavSpeedLimit()));
  drawTextLine(QString("RPM: %1").arg(car_state.getEngineRpm()));    
  drawTextLine(QString("Yaw Rate: %1 rad/s").arg(car_state.getYawRate()));
  drawTextLine(QString("ESP : %1").arg(car_state.getEspDisabled() ? "No" : "Yes"));

  // Translate enum to string for gear shifter
  QString gearShifterString;
  switch(car_state.getGearShifter()) {
    case cereal::CarState::GearShifter::PARK: 
      gearShifterString = "PARK"; break;
    case cereal::CarState::GearShifter::REVERSE: 
      gearShifterString = "REVERSE"; break;
    case cereal::CarState::GearShifter::NEUTRAL: 
      gearShifterString = "NEUTRAL"; break;
    case cereal::CarState::GearShifter::ECO: 
      gearShifterString = "ECO"; break;
    case cereal::CarState::GearShifter::MANUMATIC: 
      gearShifterString = "MANUMATIC"; break;
    case cereal::CarState::GearShifter::DRIVE: 
      gearShifterString = "DRIVE"; break;
    case cereal::CarState::GearShifter::SPORT: 
      gearShifterString = "SPORT"; break;
    case cereal::CarState::GearShifter::LOW: 
      gearShifterString = "LOW"; break;
    case cereal::CarState::GearShifter::BRAKE: 
      gearShifterString = "BRAKE"; break;
    default: 
      gearShifterString = "UNKNOWN"; break;
  }
  drawTextLine(QString("Gear: %1").arg(gearShifterString));

  // More boolean values to strings
  drawTextLine(QString("Door Open: %1").arg(car_state.getDoorOpen() ? "OPENED" : "all closed"));
  drawTextLine(QString("Seatbelt : %1").arg(car_state.getSeatbeltUnlatched() ? "NOT YET" : "latched"));  
  drawTextLine(QString("TPMS FL: %1 FR: %2").arg(car_state.getTpms().getFl()).arg(car_state.getTpms().getFr()));
  drawTextLine(QString("     RL: %1 RR: %2").arg(car_state.getTpms().getRl()).arg(car_state.getTpms().getRr()));
  drawTextLine(QString("Fuel Gauge: %1%").arg(static_cast<int>(std::round(car_state.getFuelGauge() * 100))));
  drawTextLine(QString("Charging: %1").arg(car_state.getCharging() ? "Yes" : "No"));

  // Finish up by restoring the painter's state
  p.restore();
}


// ----------------------------------------------------------------------------------

void AnnotatedCameraWidget::initializeGL() {
  CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void AnnotatedCameraWidget::updateFrameMat() {
  CameraWidget::updateFrameMat();
  UIState *s = uiState();
  int w = width(), h = height();

  s->fb_w = w;
  s->fb_h = h;

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2 - x_offset, h / 2 - y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  SubMaster &sm = *(s->sm);

  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    if (customColors != 0) {
      painter.setBrush(std::get<3>(themeConfiguration[customColors]).begin()->second);
    } else {
      painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    }
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    if (customColors != 0) {
      painter.setBrush(std::get<3>(themeConfiguration[customColors]).begin()->second);
    } else {
      painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    }
    painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, 0);
  if (sm["controlsState"].getControlsState().getExperimentalMode() || scene.acceleration_path) {
    // The first half of track_vertices are the points for the right side of the path
    // and the indices match the positions of accel from uiPlan
    const auto &acceleration_const = sm["uiPlan"].getUiPlan().getAccel();
    const int max_len = std::min<int>(scene.track_vertices.length() / 2, acceleration_const.size());

    // Copy of the acceleration vector
    std::vector<float> acceleration;
    for (int i = 0; i < acceleration_const.size(); i++) {
      acceleration.push_back(acceleration_const[i]);
    }

    for (int i = 0; i < max_len; ++i) {
      // Some points are out of frame
      if (scene.track_vertices[i].y() < 0 || scene.track_vertices[i].y() > height()) continue;

      // Flip so 0 is bottom of frame
      float lin_grad_point = (height() - scene.track_vertices[i].y()) / height();

      // If acceleration is between -0.2 and 0.2, resort to the theme color
      if (std::abs(acceleration[i]) < 0.2 && (customColors != 0)) {
        const auto &colorMap = std::get<3>(themeConfiguration[customColors]);
        for (const auto &[position, brush] : colorMap) {
          bg.setColorAt(position, brush.color());
        }
      } else {
        // speed up: 120, slow down: 0
        float path_hue = fmax(fmin(60 + acceleration[i] * 35, 120), 0);
        // FIXME: painter.drawPolygon can be slow if hue is not rounded
        path_hue = int(path_hue * 100 + 0.5) / 100;

        float saturation = fmin(fabs(acceleration[i] * 1.5), 1);
        float lightness = util::map_val(saturation, 0.0f, 1.0f, 0.95f, 0.62f);  // lighter when grey
        float alpha = util::map_val(lin_grad_point, 0.75f / 2.f, 0.75f, 0.4f, 0.0f);  // matches previous alpha fade
        bg.setColorAt(lin_grad_point, QColor::fromHslF(path_hue / 360., saturation, lightness, alpha));

        // Skip a point, unless next is last
        i += (i + 2) < max_len ? 1 : 0;
      }
    }

  } else if (customColors != 0) {
    const auto &colorMap = std::get<3>(themeConfiguration[customColors]);
    for (const auto &[position, brush] : colorMap) {
      bg.setColorAt(position, brush.color());
    }
  } else {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  }

  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices);

  // Create new path with track vertices and track edge vertices
  QPainterPath path;
  path.addPolygon(scene.track_vertices);
  path.addPolygon(scene.track_edge_vertices);

  // Paint path edges
  QLinearGradient pe(0, height(), 0, 0);
  if (alwaysOnLateralActive) {
    pe.setColorAt(0.0, QColor::fromHslF(178 / 360., 0.90, 0.38, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(178 / 360., 0.90, 0.38, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(178 / 360., 0.90, 0.38, 0.1));
  } else if (conditionalStatus == 1 || conditionalStatus == 3) {
    pe.setColorAt(0.0, QColor::fromHslF(58 / 360., 1.00, 0.50, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(58 / 360., 1.00, 0.50, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(58 / 360., 1.00, 0.50, 0.1));
  } else if (experimentalMode) {
    pe.setColorAt(0.0, QColor::fromHslF(25 / 360., 0.71, 0.50, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(25 / 360., 0.71, 0.50, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(25 / 360., 0.71, 0.50, 0.1));
  } else if (scene.navigate_on_openpilot) {
    pe.setColorAt(0.0, QColor::fromHslF(205 / 360., 0.85, 0.56, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(205 / 360., 0.85, 0.56, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(205 / 360., 0.85, 0.56, 0.1));
  } else if (customColors != 0) {
    const auto &colorMap = std::get<3>(themeConfiguration[customColors]);
    for (const auto &[position, brush] : colorMap) {
      QColor darkerColor = brush.color().darker(120);
      pe.setColorAt(position, darkerColor);
    }
  } else {
    pe.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.00, 0.68, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.00, 0.68, 0.1));
  }

  painter.setBrush(pe);
  painter.drawPath(path);

  // Paint blindspot path
  if (scene.blind_spot_path) {
    QLinearGradient bs(0, height(), 0, 0);
    if (blindSpotLeft || blindSpotRight) {
      bs.setColorAt(0.0, QColor::fromHslF(0 / 360., 0.75, 0.50, 0.6));
      bs.setColorAt(0.5, QColor::fromHslF(0 / 360., 0.75, 0.50, 0.4));
      bs.setColorAt(1.0, QColor::fromHslF(0 / 360., 0.75, 0.50, 0.2));
    }

    painter.setBrush(bs);
    if (blindSpotLeft) {
      painter.drawPolygon(scene.track_adjacent_vertices[4]);
    }
    if (blindSpotRight) {
      painter.drawPolygon(scene.track_adjacent_vertices[5]);
    }
  }

  // Paint adjacent lane paths
  if (scene.adjacent_path && (laneWidthLeft != 0 || laneWidthRight != 0)) {
    // Set up the units
    double distanceValue = is_metric ? 1.0 : METER_TO_FOOT;
    QString unit_d = is_metric ? " meters" : " feet";

    // Declare the lane width thresholds
    constexpr float minLaneWidth = 2.0f;
    constexpr float maxLaneWidth = 4.0f;

    // Set gradient colors based on laneWidth and blindspot
    auto setGradientColors = [](QLinearGradient &gradient, float laneWidth, bool blindspot) {
      // Make the path red for smaller paths or if there's a car in the blindspot and green for larger paths
      double hue = (laneWidth < minLaneWidth || laneWidth > maxLaneWidth || blindspot)
                         ? 0.0 : 120.0 * (laneWidth - minLaneWidth) / (maxLaneWidth - minLaneWidth);
      double hue_ratio = hue / 360.0;
      gradient.setColorAt(0.0, QColor::fromHslF(hue_ratio, 0.75, 0.50, 0.6));
      gradient.setColorAt(0.5, QColor::fromHslF(hue_ratio, 0.75, 0.50, 0.4));
      gradient.setColorAt(1.0, QColor::fromHslF(hue_ratio, 0.75, 0.50, 0.2));
    };

    // Paint the lanes
    auto paintLane = [&](QPainter &painter, const QPolygonF &lane, float laneWidth, bool blindspot) {
      QLinearGradient gradient(0, height(), 0, 0);
      setGradientColors(gradient, laneWidth, blindspot);

      painter.setFont(InterFont(30, QFont::DemiBold));
      painter.setBrush(gradient);
      painter.setPen(Qt::transparent);
      painter.drawPolygon(lane);
      painter.setPen(Qt::white);

      QRectF boundingRect = lane.boundingRect();
      if (scene.adjacent_path_metrics) {
        painter.drawText(boundingRect.center(),
                         blindspot ? "Vehicle in blind spot" :
                         QString("%1%2").arg(laneWidth * distanceValue, 0, 'f', 2).arg(unit_d));
      }
      painter.setPen(Qt::NoPen);
    };

    paintLane(painter, scene.track_adjacent_vertices[4], laneWidthLeft, blindSpotLeft);
    paintLane(painter, scene.track_adjacent_vertices[5], laneWidthRight, blindSpotRight);
  }

  painter.restore();
}

// This function is used to draw the lead vehicle indicator on the QPainter canvas
void AnnotatedCameraWidget::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd) {
  // Save the current state of the painter to preserve settings for future paint operations
  painter.save();

  // Determine the buffer for speed, affecting visibility of chevron based on the theme
  const float speedBuff = customColors ? 25. : 10.;  // Make the center of the chevron appear sooner if a custom theme is active
  // Determine the buffer for lead distance, affecting visibility of chevron based on the theme
  const float leadBuff = customColors ? 100. : 40.;  // Make the center of the chevron appear sooner if a custom theme is active
  // Get the relative distance from the radar data
  const float d_rel = lead_data.getDRel();
  // Get the relative speed from the radar data
  const float v_rel = lead_data.getVRel();

  // Initialize the alpha value for the chevron fill color
  float fillAlpha = 0;
  // Calculate the alpha for fill color based on relative distance to the lead vehicle
  if (d_rel < leadBuff) {
    // Increase the alpha as the vehicle gets closer within the lead buffer
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    // Further increase the alpha if the relative speed is negative (approaching)
    if (v_rel < 0) {
      // Increase the alpha based on the speed buffer when the lead car is approaching
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    // Clamp the alpha value to a maximum of 255 to ensure it stays within valid color value range
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  // Calculate the size of the chevron based on relative distance, clamping to a minimum and maximum size
  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  // Clamp the x position of the chevron to stay within the widget's width boundaries
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  // Set the y position of the chevron, ensuring it's not drawn below the bottom of the widget
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  // Calculate offsets for drawing the glow effect
  float g_xo = sz / 5;
  float g_yo = sz / 10;

  // Define the points for drawing the glow effect around the chevron
  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  // Set the brush color for the glow effect
  painter.setBrush(QColor(218, 202, 37, 255));
  // Draw the glow effect as a polygon behind the chevron
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  // Define the points for the chevron shape
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  // Set the brush color for the chevron based on custom theme settings
  if (customColors != 0) {
    // Use the color from the theme configuration if custom colors are enabled
    painter.setBrush(std::get<3>(themeConfiguration[customColors]).begin()->second);
  } else {
    // Use a standard red color with calculated transparency if custom colors are not enabled
    painter.setBrush(redColor(fillAlpha));
  }
  // Draw the chevron shape on the canvas
  painter.drawPolygon(chevron, std::size(chevron));

  // Add lead info
  // Check if lead vehicle info should be added to the indicator
  if (leadInfo) {
    // Declare the variables
    // Calculate the speed of the lead vehicle, ensuring it does not go below zero
    float lead_speed = std::max(lead_data.getVLead(), 0.0f);  // Ensure lead speed doesn't go under 0 m/s cause that's dumb

    // Form the text and center it below the chevron
    // Set the pen color and font for the lead vehicle info text
    painter.setPen(Qt::white);
    painter.setFont(InterFont(35, QFont::Bold));

    // Compose the lead vehicle info text with distance and speed
    QString text = QString("%1 %2 | %3 %4")
                           .arg(qRound(d_rel * distanceConversion))
                           .arg(leadDistanceUnit)
                           .arg(qRound(lead_speed * speedConversion))
                           .arg(leadSpeedUnit);

    // Calculate the text starting position
    // Calculate the metrics for the text to centralize it below the chevron
    QFontMetrics metrics(painter.font());
    int middle_x = (chevron[2].x() + chevron[0].x()) / 2;
    int textWidth = metrics.horizontalAdvance(text);
    // Draw the lead vehicle info text below the chevron
    painter.drawText(middle_x - textWidth / 2, chevron[0].y() + metrics.height() + 5, text);
  }

  // Restore the painter state to what it was before drawing the lead vehicle indicator
  painter.restore();
}


void AnnotatedCameraWidget::paintGL() {
}
// The paintEvent function is called by the Qt framework to update the widget's appearance
void AnnotatedCameraWidget::paintEvent(QPaintEvent *event) {
  // Retrieve the UI state object that contains various state information
  UIState *s = uiState();
  // Create a SubMaster instance for receiving data from other processes
  SubMaster &sm = *(s->sm);
  // Create a QPainter object for drawing on the widget
  QPainter painter(this);
  // Get the current time since boot for performance measurement
  const double start_draw_t = millis_since_boot();
  // Retrieve the latest model data from the messaging framework
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();

  // draw camera frame
  // Block to draw the camera frame
  {
    // Acquire the lock to safely access the frames buffer
    std::lock_guard lk(frame_lock);

    // Check if there are any frames to draw, skip drawing if not ready yet
    if (frames.empty()) {
      // Count down the number of frames to skip if there are any pending
      if (skip_frame_count > 0) {
        skip_frame_count--;
        qDebug() << "skipping frame, not ready";
        // Exit the function early if we're skipping the frame
        return;
      }
    } else {
      // skip drawing up to this many frames if we're
      // missing camera frames. this smooths out the
      // transitions from the narrow and wide cameras

      // Reset the skip frame count after drawing a frame successfully
      skip_frame_count = 5;
    }

    // Wide or narrow cam dependent on speed
    // Determine whether to use the wide or narrow camera based on the vehicle's speed
    bool has_wide_cam = available_streams.count(VISION_STREAM_WIDE_ROAD);
    // Camera switching logic based on vehicle speed and available camera streams
    if (has_wide_cam && cameraView == 0) {
      // Retrieve the vehicle's speed from the car state message
      float v_ego = sm["carState"].getCarState().getVEgo();
      // Decide whether to request the wide camera based on vehicle speed
      if ((v_ego < 10) || available_streams.size() == 1) {
        wide_cam_requested = true;
      } else if (v_ego > 15) {
        wide_cam_requested = false;
      }
      // Check if the controls are in experimental mode before switching to the wide camera
      wide_cam_requested = wide_cam_requested && sm["controlsState"].getControlsState().getExperimentalMode();
      // for replay of old routes, never go to widecam
      // Avoid switching to the wide camera during replay of old routes
      wide_cam_requested = wide_cam_requested && s->scene.calibration_wide_valid;
    }
    // Set the stream type for the CameraWidget based on the determined camera to use
    CameraWidget::setStreamType(cameraView == 3 || showDriverCamera ? VISION_STREAM_DRIVER :
                                cameraView == 2 || wide_cam_requested ? VISION_STREAM_WIDE_ROAD :
                                VISION_STREAM_ROAD);

    // Update the scene object with whether we're currently using the wide camera
    s->scene.wide_cam = CameraWidget::getStreamType() == VISION_STREAM_WIDE_ROAD;
    // Update the calibration for the CameraWidget based on current calibration data
    if (s->scene.calibration_valid) {
      // Choose calibration based on whether we're using the wide camera
      auto calib = s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;
      CameraWidget::updateCalibration(calib);
    } else {
      // Use default calibration if no valid calibration is available
      CameraWidget::updateCalibration(DEFAULT_CALIBRATION);
    }
    // Begin native painting to use OpenGL for rendering the camera frame
    painter.beginNativePainting();
    // Set the frame ID for the CameraWidget based on the model's frame ID
    CameraWidget::setFrameId(model.getFrameId());
    // Use OpenGL to draw the camera frame
    CameraWidget::paintGL();
    // End native painting as we switch back to QPainter
    painter.endNativePainting();
  }

  // Enable anti-aliasing for smoother drawing
  painter.setRenderHint(QPainter::Antialiasing);
  // Disable the pen as no outlines will be drawn
  painter.setPen(Qt::NoPen);

  // Draw the world objects and lane lines if they're enabled and the driver camera is not showing
  if (s->scene.world_objects_visible && !showDriverCamera) {
    // Update the model for the UI state
    update_model(s, model, sm["uiPlan"].getUiPlan());
    // Draw lane lines on the QPainter canvas
    drawLaneLines(painter, s);

    // Draw lead vehicle indicators if longitudinal control is enabled and radar data is available
    if (s->scene.longitudinal_control && sm.rcv_frame("radarState") > s->scene.started_frame) {
      // Get radar state information
      auto radar_state = sm["radarState"].getRadarState();
      // Update the lead vehicle positions based on radar and model data
      update_leads(s, radar_state, model.getPosition());
      // Retrieve information about the first and second lead vehicles
      auto lead_one = radar_state.getLeadOne();
      auto lead_two = radar_state.getLeadTwo();
      // Draw the lead vehicle indicators if they're detected and sufficiently spaced apart
      if (lead_one.getStatus()) {
        drawLead(painter, lead_one, s->scene.lead_vertices[0]);
      }
      if (lead_two.getStatus() && (std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
        drawLead(painter, lead_two, s->scene.lead_vertices[1]);
      }
    }
  }
  // Update the flag indicating if the driver monitoring system detected a right-hand driver
  rightHandDM = sm["driverMonitoringState"].getDriverMonitoringState().getIsRHD();

  // DMoji
  // Draw the Driver Monitoring emoji or the driver state
  #if 1
  // Show the driver face icon if the bottom icons are not hidden and driver state data is recent
  driver_face_icon->setVisible(!hideBottomIcons && (sm.rcv_frame("driverStateV2") > s->scene.started_frame));
  // Update the driver face icon based on whether the driver is on the right-hand side
  driver_face_icon->updateState(rightHandDM);
  #else
  // Update and draw the driver state if bottom icons are visible and driver state data is recent
  if (!hideBottomIcons && (sm.rcv_frame("driverStateV2") > s->scene.started_frame)) {
    // Update driver monitoring data
    update_dmonitoring(s, sm["driverStateV2"].getDriverStateV2(), dm_fade_state, rightHandDM);  // /selfdrive/ui.h
    // Draw the driver state on the canvas
    drawDriverState(painter, s);
  }
  #endif
  // Draw the heads-up display (HUD)
  drawHud(painter);

  // Get the current time to measure the drawing performance
  double cur_draw_t = millis_since_boot();
  // Calculate the time difference from the previous draw call
  double dt = cur_draw_t - prev_draw_t;
  // Update frames per second (fps) using a filter for smooth changes
  fps = fps_filter.update(1. / dt * 1000);
  // Log a warning if the frame rate is below a threshold
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  // Store the current time for the next draw call comparison
  prev_draw_t = cur_draw_t;

  // publish debug msg
  // Construct a message to publish debug information
  MessageBuilder msg;
  // Initialize a new event for UI debugging
  auto m = msg.initEvent().initUiDebug();
  // Set the time taken for drawing operations in the message
  m.setDrawTimeMillis(cur_draw_t - start_draw_t);
  // Send the debug message over the messaging framework
  pm->send("uiDebug", msg);

  // Update FrogPilot widgets
  // Call a function to update any widgets related to FrogPilot
  updateFrogPilotWidgets(painter);
}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
  CameraWidget::showEvent(event);

  ui_update_params(uiState());
  prev_draw_t = millis_since_boot();
}

void AnnotatedCameraWidget::updateFrogPilotWidgets(QPainter &p) {
  alwaysOnLateral = scene.always_on_lateral;
  alwaysOnLateralActive = scene.always_on_lateral_active;

  blindSpotLeft = scene.blind_spot_left;
  blindSpotRight = scene.blind_spot_right;

  cameraView = scene.camera_view;

  compass = scene.compass;

  conditionalExperimental = scene.conditional_experimental;
  conditionalSpeed = scene.conditional_speed;
  conditionalSpeedLead = scene.conditional_speed_lead;
  conditionalStatus = scene.conditional_status;

  bool disableSmoothing = vtscControllingCurve ? scene.disable_smoothing_vtsc : scene.disable_smoothing_mtsc;
  cruiseAdjustment = disableSmoothing ? fmax(setSpeed - scene.adjusted_cruise, 0) : fmax(0.25 * (setSpeed - scene.adjusted_cruise) + 0.75 * cruiseAdjustment - 1, 0);
  vtscControllingCurve = scene.vtsc_controlling_curve;

  customColors = scene.custom_colors;

  experimentalMode = scene.experimental_mode;

  laneWidthLeft = scene.lane_width_left;
  laneWidthRight = scene.lane_width_right;

  leadInfo = scene.lead_info;
  obstacleDistance = scene.obstacle_distance;
  obstacleDistanceStock = scene.obstacle_distance_stock;

  mapOpen = scene.map_open;
  fullMapOpen = mapOpen && scene.full_map;

  onroadAdjustableProfiles = scene.personalities_via_screen;

  pedalsOnUI = scene.pedals_on_ui;

  rightHandDrive = scene.right_hand_drive;

  roadNameUI = scene.road_name_ui;

  showDriverCamera = scene.show_driver_camera;

  speedLimitController = scene.speed_limit_controller;
  showSLCOffset = speedLimitController && scene.show_slc_offset;
  slcOverridden = speedLimitController && scene.speed_limit_overridden;
  slcSpeedLimit = scene.speed_limit;
  slcSpeedLimitOffset = scene.speed_limit_offset * (is_metric ? MS_TO_KPH : MS_TO_MPH);
  useViennaSLCSign = scene.use_vienna_slc_sign;

  turnSignalLeft = scene.turn_signal_left;
  turnSignalRight = scene.turn_signal_right;

  if (!(showDriverCamera || fullMapOpen)) {
    if (leadInfo) {
      drawLeadInfo(p);
    }

    if (alwaysOnLateral || conditionalExperimental || roadNameUI) {
      drawStatusBar(p);
    }

    if (customSignals && (turnSignalLeft || turnSignalRight)) {
      if (!animationTimer->isActive()) {
        animationTimer->start(totalFrames * 11);  // 440 milliseconds per loop; syncs up perfectly with my 2019 Lexus ES 350 turn signal clicks
      }
      drawTurnSignals(p);
    } else if (animationTimer->isActive()) {
      animationTimer->stop();
    }

    if (scene.speed_limit_changed) {
      drawSLCConfirmation(p);
    }
  }
  // -----------------------------------
  // update/draw bottons, Icons, ...
  tire_pressure_icons->setVisible(!mapOpen);
  if (!mapOpen) {
    tire_pressure_icons->updateState();
  }

  brake_disc_icons->setVisible(true);
  brake_disc_icons->updateState();

  bool enablePedalIcons = pedalsOnUI && !fullMapOpen;
  pedal_icons->setVisible(enablePedalIcons);
  if (enablePedalIcons) {
    pedal_icons->updateState();
  }

  bool enableCompass = compass && !hideBottomIcons;
  compass_img->setVisible(enableCompass);
  if (enableCompass) {
    compass_img->updateState(scene.bearing_deg);
    // The dynamic alignment for compass_img is handled in updateWidgetAlignment
    //bottom_layout->setAlignment(compass_img, (rightHandDM ? Qt::AlignLeft : Qt::AlignRight));
  }

  bool enablePersonalityButton = onroadAdjustableProfiles && !hideBottomIcons;
  personality_btn->setVisible(enablePersonalityButton);
  if (enablePersonalityButton) {
    if (paramsMemory.getBool("PersonalityChangedViaWheel")) {
      personality_btn->checkUpdate();
    }
    // The dynamic alignment for personality_btn is handled in updateWidgetAlignment
    //bottom_layout->setAlignment(personality_btn, (rightHandDM ? Qt::AlignRight : Qt::AlignLeft));
  }

  recorder_btn->setVisible(!mapOpen);

  experimental_btn->updateState(leadInfo);

  if (map_settings_btn->isEnabled()) {
    map_settings_btn->setVisible(!hideBottomIcons && compass);
    //main_layout->setAlignment(map_settings_btn, (rightHandDM ? Qt::AlignLeft : Qt::AlignRight) | (compass ? Qt::AlignTop : Qt::AlignBottom));
    updateWidgetAlignment();
  }

  // ----------------------------------------------------------
  // Update the turn signal animation images upon toggle change
  if (customSignals != scene.custom_signals) {
    customSignals = scene.custom_signals;

    QString theme_path = QString("../frogpilot/assets/custom_themes/%1/images").arg(themeConfiguration.find(customSignals) != themeConfiguration.end() ?
                                 std::get<0>(themeConfiguration[customSignals]) : "");

    QStringList imagePaths;
    int availableImages = std::get<1>(themeConfiguration[customSignals]);

    for (int i = 1; i <= totalFrames; ++i) {
      int imageIndex = ((i - 1) % availableImages) + 1;
      QString imagePath = theme_path + QString("/turn_signal_%1.png").arg(imageIndex);
      imagePaths.push_back(imagePath);
    }

    signalImgVector.clear();
    signalImgVector.reserve(2 * imagePaths.size());  // Reserve space for both regular and flipped images
    for (const QString &imagePath : imagePaths) {
      QPixmap pixmap(imagePath);
      signalImgVector.push_back(pixmap);  // Regular image
      signalImgVector.push_back(pixmap.transformed(QTransform().scale(-1, 1)));  // Flipped image
    }

    signalImgVector.push_back(QPixmap(theme_path + "/turn_signal_1_red.png"));  // Regular blindspot image
    signalImgVector.push_back(QPixmap(theme_path + "/turn_signal_1_red.png").transformed(QTransform().scale(-1, 1)));  // Flipped blindspot image
  }
}

// -----------------------------------------------------
void AnnotatedCameraWidget::drawLeadInfo(QPainter &p) {
  // Declare the variables
  static QElapsedTimer timer;
  static bool isFiveSecondsPassed = false;
  constexpr int maxAccelDuration = 5000;

  // Constants for units and conversions
  QString accelerationUnit = " m/s²";
  leadDistanceUnit = mapOpen ? "m" : "meters";
  leadSpeedUnit = "m/2";

  float accelerationConversion = 1.0f;
  distanceConversion = 1.0f;
  speedConversion = 1.0f;

  if (!scene.use_si) {
    if (is_metric) {
      // Metric conversion
      leadSpeedUnit = "kph";
      speedConversion = MS_TO_KPH;
    } else {
      // US imperial conversion
      accelerationUnit = " ft/s²";
      leadDistanceUnit = mapOpen ? "ft" : "feet";
      leadSpeedUnit = "mph";

      accelerationConversion = METER_TO_FOOT;
      distanceConversion = METER_TO_FOOT;
      speedConversion = MS_TO_MPH;
    }
  }

  // Update acceleration
  double currentAcceleration = std::round(scene.acceleration * 100) / 100;
  static double maxAcceleration = 0.0;

  if (currentAcceleration > maxAcceleration && status == STATUS_ENGAGED) {
    maxAcceleration = currentAcceleration;
    isFiveSecondsPassed = false;
    timer.start();
  } else {
    isFiveSecondsPassed = timer.hasExpired(maxAccelDuration);
  }

  // Construct text segments
  auto createText = [&](const QString &title, const double data) {
    return title + QString::number(std::round(data * distanceConversion)) + " " + leadDistanceUnit;
  };

  // Create segments for insights
  QString accelText = QString("Accel: %1%2")
    .arg(currentAcceleration * accelerationConversion, 0, 'f', 2, QChar('+'))
    .arg(accelerationUnit);
  bool bNarrow = true;
  QString maxAccSuffix = QString(bNarrow ? "" : " - Max: %1%2")
    .arg(maxAcceleration * accelerationConversion, 0, 'f', 2)
    .arg(accelerationUnit);

  QString obstacleText = createText(bNarrow ? " | Obstacle: " : " | Obstacle Factor: ", obstacleDistance);
  QString stopText = createText(bNarrow ? " - Stop: " : " - Stop Factor: ", scene.stopped_equivalence);
  QString followText = " = " + createText(bNarrow ? "Follow: " : "Follow Distance: ", scene.desired_follow);

  // ----------------------------------------------------------------------------------------------------------------
  // This lambda function, 'createDiffText', is designed to create a string that represents the difference 
  // between two values, 'data' and 'stockData'. It is intended to be used for showing the difference in some 
  // measurement (like distance) between a custom value and a stock (default) value.

  auto createDiffText = [&](const double data, const double stockData) {
    // Calculate the difference between 'data' and 'stockData', and apply a conversion factor ('distanceConversion').
    // This factor is likely to convert the value into a different unit of measurement if needed.
    double difference = std::round((data - stockData) * distanceConversion);

    // Check if the difference is not zero. If there is a difference, prepare to display it.
    // We round the difference to the nearest integer for display purposes.
    return difference != 0 
      ? QString(" (%1%2)") // If there is a difference, format the string to show it.
        .arg(difference > 0 ? "+" : "") // If the difference is positive, include a plus sign; otherwise, it's negative or zero, so no sign.
        .arg(difference) // Append the actual difference value to the string.
      : QString(); // If there is no difference, return an empty string.
  };
#if 1
  // Construct the full log string with all parts of the lead info.
  QString fullLog = QString("%1 | %2 | %3 | %4 | %5 | %6")
                      .arg(accelText)
                      .arg(maxAccSuffix)
                      .arg(obstacleText)
                      .arg(createDiffText(obstacleDistance, obstacleDistanceStock))
                      .arg(stopText)
                      .arg(followText);

  emit leadInfoUpdated(fullLog); // Emit the signal with the concatenated lead info string.
#else
  // ------------------------------
  // Prepare rectangle for insights
  p.save();

  // Create a QFont object for a monospaced font. This ensures that each character takes up the same amount of horizontal space.
  // "Monospace" is the font family name. decimal is the font size, and QFont::Normal or QFont::DemiBold is the font weight.
  QFont monospaceFont("Monospace", 20, QFont::Normal);

  // Set the style hint to TypeWriter, which advises the rendering system to prefer monospaced fonts.
  // This can help ensure that the font substitution (if "Monospace" is not available) still respects the monospaced requirement.
  monospaceFont.setStyleHint(QFont::TypeWriter);

  // Apply the QFont object to the QPainter. This changes the font used by QPainter for text rendering to our configured monospace font.
  p.setFont(monospaceFont);

  // Enable text antialiasing rendering hint for the QPainter.
  // This makes the edges of the text smoother, which is especially useful for improving the readability of text at various sizes.
  p.setRenderHint(QPainter::TextAntialiasing);

  // Define a rectangle that will be used for drawing the insights box.
  // It starts 1 pixel left of the rect's left edge, 60 pixels above the top, extends 2 pixels wider than rect's width,
  // and has a fixed height of 100 pixels. This rectangle serves as the background for the insights.
  QRect insightsRect(rect().left() - 1, rect().top() - 60, rect().width() + 2, 100);

  // Set the brush of QPainter to a semi-transparent black color (alpha value of 150 out of 255).
  // This color will be used to fill shapes, including the rounded rectangle for the insights box.
  p.setBrush(QColor(0, 0, 0, 150));

  // Draw a rounded rectangle using the defined insightsRect as its boundary.
  // The corner radius for both x and y directions is set to 30 pixels, giving it rounded corners.
  p.drawRoundedRect(insightsRect, 30, 30);

  // Calculate the positioning for the text drawing within the insights box.
  // Adjusts the rectangle to create a margin for the text. The margins are added to the top and bottom (27 pixels each).
  QRect adjustedRect = insightsRect.adjusted(0, 27, 0, 27);

  // Calculate the baseline for drawing text. This is done by finding the vertical center of the adjusted rectangle,
  // then adjusting for the font's metrics to ensure text is vertically centered.
  // p.fontMetrics().height() gives the height of the font. p.fontMetrics().descent() is subtracted to align the baseline correctly.
  int textBaseLine = adjustedRect.y() + (adjustedRect.height() + p.fontMetrics().height()) / 2 - p.fontMetrics().descent();


  // Calculate the entire text width to ensure perfect centering
  int totalTextWidth = p.fontMetrics().horizontalAdvance(accelText)
                     + p.fontMetrics().horizontalAdvance(maxAccSuffix)
                     + p.fontMetrics().horizontalAdvance(obstacleText)
                     + p.fontMetrics().horizontalAdvance(createDiffText(obstacleDistance, obstacleDistanceStock))
                     + p.fontMetrics().horizontalAdvance(stopText)
                     + p.fontMetrics().horizontalAdvance(followText);

  int textStartPos = adjustedRect.x() + (adjustedRect.width() - totalTextWidth) / 2;
  // Draw the text
  auto drawText = [&](const QString &text, const QColor color) {
    p.setPen(color);
    p.drawText(textStartPos, textBaseLine, text);
    textStartPos += p.fontMetrics().horizontalAdvance(text);
  };

  drawText(accelText, Qt::white);
  drawText(maxAccSuffix, isFiveSecondsPassed ? Qt::white : Qt::red);
  drawText(obstacleText, Qt::white);
  drawText(createDiffText(obstacleDistance, obstacleDistanceStock), (obstacleDistance - obstacleDistanceStock) > 0 ? Qt::green : Qt::red);
  drawText(stopText, Qt::white);
  drawText(followText, Qt::white);
  p.restore();
#endif
}

void AnnotatedCameraWidget::drawSLCConfirmation(QPainter &p) {
  p.save();

  // Get screen size
  QSize size = this->size();

  // Configure the screen halves
  QRect leftRect(0, 0, size.width() / 2, size.height());
  QRect rightRect = leftRect.translated(size.width() / 2, 0);

  // Fill in the screen halves
  p.setOpacity(0.5);
  p.fillRect(leftRect, rightHandDrive ? redColor() : greenColor());
  p.fillRect(rightRect, rightHandDrive ? greenColor() : redColor());
  p.setOpacity(1.0);

  // Configure the font
  p.setFont(InterFont(75, QFont::Bold));
  p.setPen(Qt::white);

  // Configure the text
  QString unitText = is_metric ? "kph" : "mph";
  QString speedText = QString::number(std::nearbyint(scene.unconfirmed_speed_limit * (is_metric ? MS_TO_KPH : MS_TO_MPH))) + " " + unitText;
  QString confirmText = "Confirm speed limit\n" + speedText;
  QString ignoreText = "Ignore speed limit\n" + speedText;

  // Draw the text
  p.drawText(leftRect, Qt::AlignCenter | Qt::AlignTop, confirmText);
  p.drawText(rightRect, Qt::AlignCenter | Qt::AlignTop, ignoreText);

  p.restore();
}

void AnnotatedCameraWidget::drawStatusBar(QPainter &p) {
  p.save();

  // Variable declarations
  static QElapsedTimer timer;
  static QString lastShownStatus;

  QString newStatus;

  static bool displayStatusText = false;

  constexpr qreal fadeDuration = 1500.0;
  constexpr qreal textDuration = 5000.0;

  // Draw status bar
  QRect currentRect = rect();
  QRect statusBarRect(currentRect.left() - 1, currentRect.bottom() - 50, currentRect.width() + 2, 100);
  p.setBrush(QColor(0, 0, 0, 150));
  p.setOpacity(1.0);
  p.drawRoundedRect(statusBarRect, 30, 30);

  std::map<int, QString> conditionalStatusMap = {
    {0, "Conditional Experimental Mode ready"},
    {1, "Conditional Experimental overridden"},
    {2, "Experimental Mode manually activated"},
    {3, "Conditional Experimental overridden"},
    {4, "Experimental Mode manually activated"},
    {5, "Experimental Mode activated for" + (mapOpen ? " intersection" : QString(" upcoming intersection"))},
    {6, "Experimental Mode activated for" + (mapOpen ? " turn" : QString(" upcoming turn"))},
    {7, "Experimental Mode activated due to" + (mapOpen ? "SLC" : QString(" no speed limit set"))},
    {8, "Experimental Mode activated due to" + (mapOpen ? " speed" : " speed being less than " + QString::number(conditionalSpeedLead) + (is_metric ? " kph" : " mph"))},
    {9, "Experimental Mode activated due to" + (mapOpen ? " speed" : " speed being less than " + QString::number(conditionalSpeed) + (is_metric ? " kph" : " mph"))},
    {10, "Experimental Mode activated for slower lead"},
    {11, "Experimental Mode activated for turn" + (mapOpen ? "" : QString(" / lane change"))},
    {12, "Experimental Mode activated for curve"},
    {13, "Experimental Mode activated for stop" + (mapOpen ? "" : QString(" sign / stop light"))},
  };

  QString roadName = roadNameUI ? QString::fromStdString(paramsMemory.get("RoadName")) : QString();

  // Update status text
  if (alwaysOnLateralActive) {
    newStatus = QString("Always On Lateral active") + (mapOpen ? "" : ". Press the \"Cruise Control\" button to disable");
  } else if (conditionalExperimental) {
    newStatus = conditionalStatusMap[status != STATUS_DISENGAGED ? conditionalStatus : 0];
  }

  // Append suffix to the status
  QString screenSuffix = ". Double tap the screen to revert";
  QString wheelSuffix = ". Double press the \"LKAS\" button to revert";

  if (!alwaysOnLateralActive && !mapOpen && status != STATUS_DISENGAGED && !newStatus.isEmpty()) {
    newStatus += (conditionalStatus == 3 || conditionalStatus == 4) ? screenSuffix : (conditionalStatus == 1 || conditionalStatus == 2) ? wheelSuffix : "";
  }

  // Check if status has changed or if the road name is empty
  if (newStatus != lastShownStatus || roadName.isEmpty()) {
    displayStatusText = true;
    lastShownStatus = newStatus;
    timer.restart();
  } else if (displayStatusText && timer.hasExpired(textDuration + fadeDuration)) {
    displayStatusText = false;
  }

  // Configure the text
  p.setFont(InterFont(40, QFont::Bold));
  p.setPen(Qt::white);
  p.setRenderHint(QPainter::TextAntialiasing);

  // Calculate text opacity
  static qreal roadNameOpacity;
  static qreal statusTextOpacity;
  int elapsed = timer.elapsed();
  if (displayStatusText) {
    statusTextOpacity = qBound(0.0, 1.0 - (elapsed - textDuration) / fadeDuration, 1.0);
    roadNameOpacity = 1.0 - statusTextOpacity;
  } else {
    roadNameOpacity = qBound(0.0, elapsed / fadeDuration, 1.0);
    statusTextOpacity = 0.0;
  }

  // Draw the status text
  p.setOpacity(statusTextOpacity);
  QRect textRect = p.fontMetrics().boundingRect(statusBarRect, Qt::AlignCenter | Qt::TextWordWrap, newStatus);
  textRect.moveBottom(statusBarRect.bottom() - 50);
  p.drawText(textRect, Qt::AlignCenter | Qt::TextWordWrap, newStatus);

  // Draw the road name with the calculated opacity
  if (!roadName.isEmpty()) {
    p.setOpacity(roadNameOpacity);
    textRect = p.fontMetrics().boundingRect(statusBarRect, Qt::AlignCenter | Qt::TextWordWrap, roadName);
    textRect.moveBottom(statusBarRect.bottom() - 50);
    p.drawText(textRect, Qt::AlignCenter | Qt::TextWordWrap, roadName);
  }

  p.restore();
}

void AnnotatedCameraWidget::drawTurnSignals(QPainter &p) {
  // Declare the turn signal size
  constexpr int signalHeight = 480;
  constexpr int signalWidth = 360;

  // Enable Antialiasing
  p.setRenderHint(QPainter::Antialiasing);

  // Calculate the vertical position for the turn signals
  int baseYPosition = (height() - signalHeight) / 2 + (alwaysOnLateral || conditionalExperimental || roadNameUI ? 225 : 300);
  // Calculate the x-coordinates for the turn signals
  int leftSignalXPosition = 75 + width() - signalWidth - 300 * (blindSpotLeft ? 0 : animationFrameIndex);
  int rightSignalXPosition = -75 + 300 * (blindSpotRight ? 0 : animationFrameIndex);

  // Draw the turn signals
  if (animationFrameIndex < signalImgVector.size()) {
    auto drawSignal = [&](bool signalActivated, int xPosition, bool flip, bool blindspot) {
      if (signalActivated) {
        // Get the appropriate image from the signalImgVector
        int uniqueImages = signalImgVector.size() / 4;  // Each image has a regular, flipped, and two blindspot versions
        int index = (blindspot ? 2 * uniqueImages : 2 * animationFrameIndex % totalFrames) + (flip ? 1 : 0);
        QPixmap &signal = signalImgVector[index];
        p.drawPixmap(xPosition, baseYPosition, signalWidth, signalHeight, signal);
      }
    };

    // Display the animation based on which signal is activated
    drawSignal(turnSignalLeft, leftSignalXPosition, false, blindSpotLeft);
    drawSignal(turnSignalRight, rightSignalXPosition, true, blindSpotRight);
  }
}

// ==========================
// ExperimentalButton
ExperimentalButton::ExperimentalButton(QWidget *parent) : experimental_mode(false), engageable(false), QPushButton(parent), scene(uiState()->scene) {
  setFixedSize(btn_size, btn_size);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &ExperimentalButton::changeMode);

  // Custom steering wheel images
  wheelImages = {
    {0, loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size})},
    {1, loadPixmap("../frogpilot/assets/wheel_images/lexus.png", {img_size, img_size})},
    {2, loadPixmap("../frogpilot/assets/wheel_images/toyota.png", {img_size, img_size})},
    {3, loadPixmap("../frogpilot/assets/wheel_images/frog.png", {img_size, img_size})},
    {4, loadPixmap("../frogpilot/assets/wheel_images/rocket.png", {img_size, img_size})},
    {5, loadPixmap("../frogpilot/assets/wheel_images/hyundai.png", {img_size, img_size})},
    {6, loadPixmap("../frogpilot/assets/wheel_images/stalin.png", {img_size, img_size})},
    {7, loadPixmap("../frogpilot/assets/random_events/images/firefox.png", {img_size, img_size})}
  };
}

void ExperimentalButton::changeMode() {
  Params paramsMemory = Params("/dev/shm/params");

  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool can_change = hasLongitudinalControl(cp) && (params.getBool("ExperimentalModeConfirmed") || scene.experimental_mode_via_screen);
  if (can_change) {
    if (scene.conditional_experimental) {
      int override_value = (scene.conditional_status >= 1 && scene.conditional_status <= 4) ? 0 : scene.conditional_status >= 5 ? 3 : 4;
      paramsMemory.putIntNonBlocking("ConditionalStatus", override_value);
    } else {
      params.putBool("ExperimentalMode", !experimental_mode);
    }
  }
}

void ExperimentalButton::updateState(bool leadInfo) {
  const auto cs = (*uiState()->sm)["controlsState"].getControlsState();
  bool eng = cs.getEngageable() || cs.getEnabled() || scene.always_on_lateral_active;
  if ((cs.getExperimentalMode() != experimental_mode) || (eng != engageable)) {
    engageable = eng;
    experimental_mode = cs.getExperimentalMode();
    update();
  }

  // FrogPilot variables
  firefoxRandomEventTriggered = scene.current_random_event == 1;
  rotatingWheel = scene.rotating_wheel;
  wheelIcon = scene.wheel_icon;

  y_offset = 0;
  //y_offset = leadInfo ? 10 : 0;

  if (firefoxRandomEventTriggered) {
    static int rotationDegree = 0;
    rotationDegree = (rotationDegree + 36) % 360;
    steeringAngleDeg = rotationDegree;
    wheelIcon = 7;
    update();
  // Update the icon so the steering wheel rotates in real time
  } else if (rotatingWheel && steeringAngleDeg != scene.steering_angle_deg) {
    steeringAngleDeg = scene.steering_angle_deg;
    update();
  }
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  // Custom steering wheel icon
  engage_img = wheelImages[wheelIcon];
  QPixmap img = wheelIcon ? engage_img : (experimental_mode ? experimental_img : engage_img);

  QColor background_color = wheelIcon && !isDown() && engageable ?
      (scene.always_on_lateral_active ? QColor(10, 186, 181, 255) :
      (scene.conditional_status == 1 ? QColor(255, 246, 0, 255) :
      (experimental_mode ? QColor(218, 111, 37, 241) :
      (scene.navigate_on_openpilot ? QColor(49, 161, 238, 255) : QColor(0, 0, 0, 166))))) :
      QColor(0, 0, 0, 166);

  if (!(scene.show_driver_camera || scene.map_open && scene.full_map)) {
    if (rotatingWheel || firefoxRandomEventTriggered) {
      drawIconRotate(p, QPoint(btn_size / 2, btn_size / 2 + y_offset), img, background_color, (isDown() || !(engageable || scene.always_on_lateral_active)) ? 0.6 : 1.0, steeringAngleDeg);
    } else {
      drawIcon(
        p, 
        QPoint(btn_size / 2, btn_size / 2 + y_offset), 
        img, 
        background_color, 
        (isDown() || !(engageable || scene.always_on_lateral_active)) ? 0.6 : 1.0);
    }
  }
}

// ==========================
// MapSettingsButton
MapSettingsButton::MapSettingsButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size);
  settings_img = loadPixmap("../assets/navigation/icon_directions_outlined.svg", {img_size, img_size});

  // hidden by default, made visible if map is created (has prime or mapbox token)
  setVisible(false);
  setEnabled(false);
}

void MapSettingsButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), settings_img, QColor(0, 0, 0, 166), isDown() ? 0.6 : 1.0);
}

// ==========================
// Compass
Compass::Compass(QWidget *parent) : QWidget(parent), bearingDeg(0) {
  // Load the outer compass image, set to the size of btn_size
  compassOuterImg = loadPixmap("../frogpilot/assets/other_images/compass_outer.png", QSize(btn_size, btn_size));

  // Set the widget size to the size of the outer compass image
  setFixedSize(compassOuterImg.size());

  // Calculate the center coordinates of the widget
  x = width() / 2;
  y = height() / 2;

  // Load the inner compass image (the needle) and scale it down to 75%
  int innerSize = btn_size * 0.55; // % of btn_size
  compassInnerImg = loadPixmap("../frogpilot/assets/other_images/compass_inner.png", QSize(innerSize, innerSize));
}
void Compass::updateState(int bearing_deg) {
  if (bearingDeg != bearing_deg) {
    update();
    bearingDeg = bearing_deg;
  }
}
// Paint event method
void Compass::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

  // Draw a semi-transparent black circle as background
  QColor semiTransparentColor(0, 0, 0, 128); // Black with 50% opacity
  p.setBrush(semiTransparentColor);
  p.setPen(Qt::NoPen); // No border
  p.drawEllipse(x - compassOuterImg.width() / 2, y - compassOuterImg.height() / 2, compassOuterImg.width(), compassOuterImg.height());

  // Draw the outer compass image
  p.drawPixmap((width() - compassOuterImg.width()) / 2, (height() - compassOuterImg.height()) / 2, compassOuterImg);

  // Translate and rotate to draw the inner compass image
  p.translate(x, y);
  p.rotate(bearingDeg);
  p.drawPixmap(-compassInnerImg.width() / 2, -compassInnerImg.height() / 2, compassInnerImg);
  p.rotate(-bearingDeg);
  p.translate(-x, -y);

  // Additional drawing code (e.g., for the bearing degree numbers and cardinal directions) should go here,
  // taking care to adjust positions and sizes by multiplying by 0.75 where necessary.
}
// ==========================
// BrakeDiscIcons Constructor
BrakeDiscIcons::BrakeDiscIcons(UIState* ui_state, QWidget *parent)
    : QWidget(parent), ui_state(ui_state) {
    setFixedSize(btn_size, btn_size);
    //ic_brake = loadPixmap("../frogpilot/assets/other_images/img_brake_disc.png", QSize(img_size, img_size));
    ic_brake = loadPixmap("../frogpilot/assets/other_images/braking_lights.png", QSize(img_size, img_size));
}

void BrakeDiscIcons::paintEvent(QPaintEvent *event) {
    QPainter p(this);
    const auto car_state = (*uiState()->sm)["carState"].getCarState();
    //bool brake_valid = car_state.getBrakePressed();
    bool brake_valid = car_state.getBrakeLights();
    float img_alpha = brake_valid ? 1.0f : 0.15f;
    QColor bg_color = QColor(0, 0, 0, brake_valid ? 255 * 0.3f : 255 * 0.1f);
    QPoint center = this->rect().center();
    drawIcon(p, center, ic_brake, bg_color, img_alpha);
}

void BrakeDiscIcons::updateState() {
  update(); // Trigger repaint
}

// ==========================
// TIRE PRESSURE
static const QColor get_tpms_color(float tpms) {
    if(tpms < 5 || tpms > 60) // N/A
        return QColor(255, 255, 255, 220);
    if(tpms < 31)
        return QColor(255, 90, 90, 220);
    return QColor(255, 255, 255, 220);
}
static const QString get_tpms_text(float tpms) {
    if(tpms < 5 || tpms > 60)
        return "";

    char str[32];
    snprintf(str, sizeof(str), "%.0f", round(tpms));
    return QString(str);
}
TirePressureIcons::TirePressureIcons(UIState* ui_state, QWidget *parent)
    : QWidget(parent), ui_state(ui_state) {
    setFixedSize(btn_size, btn_size);
    ic_tire_pressure = loadPixmap("../frogpilot/assets/other_images/img_tire_pressure.png", QSize(img_size, img_size));
}
// AnnotatedCameraWidget::updateFrogPilotWidgets 에서
// setVisible과 updateState를 call.
void TirePressureIcons::updateState() {
  update();
}
// Paint event handler for TirePressureIcons. This is called by the Qt framework to draw.
void TirePressureIcons::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  auto tpms = (*uiState()->sm)["carState"].getCarState().getTpms();

  if (tpms.getEnabled()) {
    int iconWidth = ic_tire_pressure.width();
    int iconHeight = ic_tire_pressure.height();
    QPoint iconCenter = this->rect().center();

    p.setOpacity(0.8f);
    p.drawPixmap(iconCenter.x() - iconWidth / 2, iconCenter.y() - iconHeight / 2, ic_tire_pressure);

    p.setFont(InterFont(38, QFont::Bold));
    QFontMetrics metrics(p.font());

    const float fl = tpms.getFl();
    const float fr = tpms.getFr();
    const float rl = tpms.getRl();
    const float rr = tpms.getRr();

    // Increase xOffset for FL and RL to move them more to the left
    int xOffsetFL_RL = iconWidth*5/4;
    int xOffsetFR_RR = iconWidth/2;

    int yOffset = iconHeight / 4;
    //LEFT
    QPoint flPos = iconCenter - QPoint(xOffsetFL_RL,  yOffset);
    QPoint rlPos = iconCenter - QPoint(xOffsetFL_RL, -yOffset);
    //RIGHT
    QPoint frPos = iconCenter + QPoint(xOffsetFR_RR, -yOffset);
    QPoint rrPos = iconCenter + QPoint(xOffsetFR_RR,  yOffset);

    // Draw the text next to each wheel
    p.setPen(get_tpms_color(fl));
    p.drawText(flPos, get_tpms_text(fl));
    p.setPen(get_tpms_color(fr));
    p.drawText(frPos, get_tpms_text(fr));
    p.setPen(get_tpms_color(rl));
    p.drawText(rlPos, get_tpms_text(rl));
    p.setPen(get_tpms_color(rr));
    p.drawText(rrPos, get_tpms_text(rr));
  }
}

// ==========================
// PEDAL ICONS
PedalIcons::PedalIcons(QWidget *parent) : QWidget(parent), scene(uiState()->scene) {
  setFixedSize(btn_size, btn_size);

  brake_pedal_img = loadPixmap("../frogpilot/assets/other_images/brake_pedal.png", QSize(img_size, img_size));
  gas_pedal_img = loadPixmap("../frogpilot/assets/other_images/gas_pedal.png", QSize(img_size, img_size));
}
constexpr float PIVOT_ACCEL = 0.25f;
constexpr float PIVOT_DECEL = -0.25f;

void PedalIcons::updateState() {
  acceleration = scene.acceleration;

  accelerating = acceleration > PIVOT_ACCEL;
  decelerating = acceleration < PIVOT_DECEL;

  if (accelerating || decelerating) {
    update();
  }
}
// Paint event handler for PedalIcons. This is called by the Qt framework to draw the pedal icons.
void PedalIcons::paintEvent(QPaintEvent *event) {
  // QPainter is used for all drawing operations
  QPainter p(this);
  // Set antialiasing to true for smoother drawing
  p.setRenderHint(QPainter::Antialiasing);

  // Increase the img_size
  int newSize = img_size * 1.3;

  // Calculate the total width needed to draw both pedal icons side by side
  int totalWidth = 2 * newSize;
  // Calculate the starting X coordinate to center the icons within the widget
  int startX = (width() - totalWidth) / 2;

  // Calculate X coordinate for the brake pedal icon, centering it within its half of the area
  int brakeX = startX + newSize / 2;
  // Calculate X coordinate for the gas pedal icon, placing it next to the brake
  int gasX = startX + newSize;

  float brakeOpacity = scene.standstill ? 1.0f : decelerating ? std::max(0.25f, std::abs(acceleration)) : 0.25f;
  float gasOpacity = accelerating ? std::max(0.25f, acceleration) : 0.25f;

  // Set the painter's opacity before drawing the brake pedal
  p.setOpacity(brakeOpacity);
  // Draw the brake pedal icon at its calculated position
  p.drawPixmap(brakeX, (height() - newSize) / 2, brake_pedal_img);

  // Set the painter's opacity before drawing the gas pedal
  p.setOpacity(gasOpacity);
  // Draw the gas pedal icon at its calculated position
  p.drawPixmap(gasX, (height() - newSize) / 2, gas_pedal_img);
}



// ==================
// PersonalityButton
PersonalityButton::PersonalityButton(QWidget *parent) : QPushButton(parent), scene(uiState()->scene) {
  setFixedSize(btn_size*1.3, btn_size);

  // Configure the profile vector
  profile_data = {
    {QPixmap("../frogpilot/assets/other_images/aggressive.png"), "Aggressive"},
    {QPixmap("../frogpilot/assets/other_images/standard.png"), "Standard"},
    {QPixmap("../frogpilot/assets/other_images/relaxed.png"), "Relaxed"}
  };

  personalityProfile = params.getInt("LongitudinalPersonality");

  transitionTimer.start();

  connect(this, &QPushButton::clicked, this, &PersonalityButton::handleClick);
}

void PersonalityButton::checkUpdate() {
  // Sync with the steering wheel button
  personalityProfile = params.getInt("LongitudinalPersonality");
  updateState();
  paramsMemory.putBool("PersonalityChangedViaWheel", false);
}

void PersonalityButton::handleClick() {
  int mapping[] = {2, 0, 1};
  personalityProfile = mapping[personalityProfile];

  params.putInt("LongitudinalPersonality", personalityProfile);
  paramsMemory.putBool("PersonalityChangedViaUI", true);

  updateState();
}

void PersonalityButton::updateState() {
  // Start the transition
  transitionTimer.restart();
}

void PersonalityButton::paintEvent(QPaintEvent *) {
  // Declare the constants
  constexpr qreal fadeDuration = 1000.0;  // 1 second
  constexpr qreal textDuration = 2000.0;  // 2 seconds

  QPainter p(this);
  int elapsed = transitionTimer.elapsed();
  qreal textOpacity = qBound(0.0, 1.0 - ((elapsed - textDuration) / fadeDuration), 1.0);
  qreal imageOpacity = qBound(0.0, (elapsed - textDuration) / fadeDuration, 1.0);

  // Enable Antialiasing
  p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

  // Configure the button
  auto &[profile_image, profile_text] = profile_data[personalityProfile];
  QRect rect(0, 0, width(), height()/*+ 95*/);

  // Draw the profile text with the calculated opacity
  if (textOpacity > 0.0) {
    p.setOpacity(textOpacity);
    p.setFont(InterFont(40, QFont::Bold));
    p.setPen(Qt::white);
    p.drawText(rect, Qt::AlignCenter, profile_text);
  }

  // Draw the profile image with the calculated opacity
  if (imageOpacity > 0.0) {
    // Calculate the x and y coordinates to position the icon at the bottom center of the widget
    int x = (width() - profile_image.width()) / 2; // Center horizontally
    int y = height() - profile_image.height();      // Align to bottom

    // The center point for the icon
    QPoint center(x + profile_image.width() / 2, y + profile_image.height() / 2);
    
    drawIcon(p, center, profile_image, Qt::transparent, imageOpacity);
  }
}
// ==========================
// Driver's Face
DriverFaceIcon::DriverFaceIcon(UIState* ui_state, QWidget *parent)
    : QWidget(parent), scene(uiState()->scene), ui_state(ui_state) {
    setFixedSize(btn_size, btn_size);
    ic_driver_face = loadPixmap("../assets/img_driver_face.png", QSize(img_size + 5, img_size + 5));    
}
void DriverFaceIcon::updateState(bool rightHandDM) {
  // update DM icon
  auto dm_state = (*uiState()->sm)["driverMonitoringState"].getDriverMonitoringState();
  dmActive = dm_state.getIsActiveMode();
  // DM icon transition
  dm_fade_state = std::clamp(dm_fade_state+0.2*(0.5-dmActive), 0.0, 1.0);
  rightHandDM_ = rightHandDM;
  update_dmonitoring(ui_state, (*uiState()->sm)["driverStateV2"].getDriverStateV2(), dm_fade_state, rightHandDM_);  // /selfdrive/ui.h

  update(); // if(isVisible() is not necessary. Only repaint if something has changed that requires a repaint.
}
// This is called by the Qt framework to draw.
void DriverFaceIcon::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  // Ensure antialiasing for smoother output
  p.setRenderHint(QPainter::Antialiasing);

  // base icon
  float opacity = dmActive ? 0.65 : 0.2;

  // Calculate the x and y coordinates to position the icon at the bottom center of the widget
  int x = (width() - ic_driver_face.width()) / 2; // Center horizontally
  int y = height() - ic_driver_face.height();      // Align to bottom

  // The center point for the icon
  QPoint center(x + ic_driver_face.width() / 2, y + ic_driver_face.height() / 2);

  // Draw the driver face icon with the specified opacity
  drawIcon(p, center, ic_driver_face, blackColor(70), opacity);

  // Calculate the offset for drawing the facial keypoints and tracking arcs
  int face_icon_center_x = x + ic_driver_face.width() / 2;
  int face_icon_center_y = y + ic_driver_face.height() / 2;

  // Now draw the facial keypoints and tracking arcs using the calculated x and y
  QPointF face_kpts_draw[std::size(default_face_kpts_3d)];
  float kp;
  for (int i = 0; i < std::size(default_face_kpts_3d); ++i) {
      kp = (scene.face_kpts_draw[i].v[2] - 8) / 120 + 1.0;
      // Offset each keypoint by the position of the icon's center
      face_kpts_draw[i] = QPointF(scene.face_kpts_draw[i].v[0] * kp + face_icon_center_x, 
                                  scene.face_kpts_draw[i].v[1] * kp + face_icon_center_y);
  }

  p.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  p.drawPolyline(face_kpts_draw, std::size(default_face_kpts_3d));

  // Tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7;
  const float arc_t_extend = 12.0;
  QColor arc_color = QColor::fromRgbF(0.545 - 0.445 * ui_state->engaged(),
                                      0.545 + 0.4 * ui_state->engaged(),
                                      0.545 - 0.285 * ui_state->engaged(),
                                      0.4 * (1.0 - dm_fade_state));
  float delta_x = -scene.driver_pose_sins[1] * arc_l / 2;
  float delta_y = -scene.driver_pose_sins[0] * arc_l / 2;

  p.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  // Use the center of the driver face icon for the start point of the arc drawing
  QRectF arc_rect_x(std::fmin(face_icon_center_x + delta_x, face_icon_center_x), face_icon_center_y - arc_l / 2, fabs(delta_x) * 2, arc_l);
  p.drawArc(arc_rect_x, (scene.driver_pose_sins[1] > 0 ? 90 : 270) * 16, 180 * 16);

  p.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  QRectF arc_rect_y(face_icon_center_x - arc_l / 2, std::fmin(face_icon_center_y + delta_y, face_icon_center_y), arc_l, fabs(delta_y) * 2);
  p.drawArc(arc_rect_y, (scene.driver_pose_sins[0] > 0 ? 0 : 180) * 16, 180 * 16);
}
