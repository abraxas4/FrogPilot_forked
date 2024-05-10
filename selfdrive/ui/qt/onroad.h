#pragma once

#include <memory>

#include <QPushButton>
#include <QStackedLayout>
#include <QWidget>

#include "common/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

#include "selfdrive/frogpilot/screenrecorder/screenrecorder.h"

const int btn_size = 192;
const int img_size = (btn_size / 4) * 3;

// FrogPilot global variables
static double fps;


// Inline functions for color convenience
inline QColor greenColor(int alpha = 242) { return QColor(23, 134, 68, alpha); }
inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); } // Returns a red color with the specified opacity.
inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); } // Returns a white color with the specified opacity.
inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); } // Returns a black color with the specified opacity.
inline QColor yellowColor(int alpha = 255) { return QColor(255, 255, 0, alpha); }

// ***** onroad widgets *****
class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent), scene(uiState()->scene) {}
  void updateAlert(const Alert &a);

protected:
  void paintEvent(QPaintEvent*) override;

private:
  QColor bg;
  Alert alert = {};

  // FrogPilot variables
  UIScene &scene;
};
// -----------------------------


class BrakeDiscIcons : public QWidget {
    Q_OBJECT

public:
    explicit BrakeDiscIcons(UIState* ui_state, QWidget *parent = nullptr);
    void updateState();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QPixmap ic_brake;
    UIState* ui_state;
};


class TirePressureIcons : public QWidget {
    Q_OBJECT

public:
    explicit TirePressureIcons(UIState* ui_state, QWidget *parent = nullptr);
    void updateState();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QPixmap ic_tire_pressure;
    UIState* ui_state;
};

class ExperimentalButton : public QPushButton {
  Q_OBJECT

public:
  explicit ExperimentalButton(QWidget *parent = 0);
  void updateState(bool leadInfo);

private:
  void paintEvent(QPaintEvent *event) override;
  void changeMode();

  Params params;
  QPixmap engage_img;
  QPixmap experimental_img;
  bool experimental_mode;
  bool engageable;

  // FrogPilot variables
  UIScene &scene;

  std::map<int, QPixmap> wheelImages;

  bool firefoxRandomEventTriggered;
  bool rotatingWheel;
  int steeringAngleDeg;
  int wheelIcon;
  int y_offset;
};

class PedalIcons : public QWidget {
  Q_OBJECT

public:
  explicit PedalIcons(QWidget *parent = 0);
  void updateState();

private:
  void paintEvent(QPaintEvent *event) override;

  QPixmap brake_pedal_img;
  QPixmap gas_pedal_img;

  UIScene &scene;

  bool accelerating;
  bool decelerating;

  float acceleration;
};

class MapSettingsButton : public QPushButton {
  Q_OBJECT

public:
  explicit MapSettingsButton(QWidget *parent = 0);

private:
  void paintEvent(QPaintEvent *event) override;

  QPixmap settings_img;
};

class Compass : public QWidget {
public:
  explicit Compass(QWidget *parent = nullptr);

  void updateState(int bearing_deg);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  int bearingDeg = 0; // Current bearing degree
  QPixmap compassInnerImg; // Image for the inner compass (the needle)
  QPixmap compassOuterImg; // Image for the outer compass (the static part)
  int x, y; // Center coordinates of the compass
};

class PersonalityButton : public QPushButton {
public:
  explicit PersonalityButton(QWidget *parent = 0);

  void checkUpdate();
  void handleClick();
  void updateState();

private:
  void paintEvent(QPaintEvent *event) override;

  Params params;
  Params paramsMemory{"/dev/shm/params"};

  UIScene &scene;

  int personalityProfile = 0;

  QElapsedTimer transitionTimer;

  QVector<std::pair<QPixmap, QString>> profile_data;
};
class DriverFaceIcon : public QWidget {
    Q_OBJECT

  bool dmActive = false; // Flag for driver monitoring system activity
  float dm_fade_state = 1.0; // Opacity state for driver monitoring fade effect
  bool rightHandDM_ = false;

public:
    explicit DriverFaceIcon(UIState* ui_state, QWidget *parent = nullptr);
    void updateState(bool rightHandDM);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QPixmap ic_driver_face;
    UIState* ui_state;
    UIScene &scene;
};

// --------------------------------------------------
// container window for the NVG UI
class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = nullptr);
  void updateState(const UIState &s);

// Define a signal in your AnnotatedCameraWidget header file.
signals:
  void leadInfoUpdated(const QString &info); // Simplified signal

public:
  MapSettingsButton *map_settings_btn;
private:
  QGridLayout* mainLayout;
  QVBoxLayout* bottomLeftMainLayout; // New QVBoxLayout for the bottom left main layout
  QHBoxLayout* bottomLeft1stFloorLayout;
  QVBoxLayout* bottomRightLayout;
  //std::map<QWidget*, QPair<int, int>> widgetPositions;

  // Widgets for corners
  QWidget *topLeftWidget; // Container for top-left corner widgets
  QWidget *topRightWidget; // Container for top-right corner widgets
  QWidget *bottomLeftWidget; // Container for bottom-left corner widgets
  QWidget *bottomRightWidget; // Container for bottom-right corner widgets

  BrakeDiscIcons *brake_disc_icons; // BrakeDisc icon widgets
  TirePressureIcons *tire_pressure_icons; // TirePressure icon widgets
  ScreenRecorder *recorder_btn; // Button to control screen recording
  ExperimentalButton *experimental_btn; // Button for experimental features
  PedalIcons *pedal_icons; // Pedal icon widgets
  Compass *compass_img; // Compass widget
  PersonalityButton *personality_btn; // Button to toggle driving personality
  DriverFaceIcon *driver_face_icon;


  float speed; // Current vehicle speed
  QString speedUnit; // Unit of the speed (e.g., km/h or mph)
  float setSpeed; // Speed set by cruise control
  float speedLimit; // Speed limit detected or set by the system
  bool is_cruise_set = false; // Flag to indicate if cruise control is set
  bool is_metric = false; // Flag for metric system usage for units

  bool hideBottomIcons = false; // Flag to hide or show bottom icons
  bool rightHandDM = false; // Flag for right-hand drive configurations for DM
  bool has_us_speed_limit = false; // Flag for US speed limit detection
  bool has_eu_speed_limit = false; // Flag for EU speed limit detection
  bool v_ego_cluster_seen = false; // Flag indicating if ego vehicle's speed is shown in cluster
  int status = STATUS_DISENGAGED; // Current status of the driving session
  std::unique_ptr<PubMaster> pm; // Publisher for interprocess communication

  int skip_frame_count = 0; // Counter for skipped frames
  bool wide_cam_requested = false; // Flag to indicate if wide camera view is requested
  
  // Methods to position and resize widgets within the container widgets
  void positionWidgets();

  // FrogPilot-specific UI initialization and update functions
  void initializeFrogPilotWidgets();
  void updateFrogPilotWidgets(QPainter &p);

  // Functions to draw lead car info, SLC confirmation, status bar, and turn signals
  void drawLeadInfo(QPainter &p);
  void drawSLCConfirmation(QPainter &p);
  void drawStatusBar(QPainter &p);
  void drawTurnSignals(QPainter &p);

  // FrogPilot variables and additional UI elements
  Params paramsMemory{"/dev/shm/params"}; // In-memory storage for parameters

  UIScene &scene; // Reference to the scene containing vehicle and environment data

  // Flags and settings for various FrogPilot features and UI customizations
  bool alwaysOnLateral;
  bool alwaysOnLateralActive;
  bool blindSpotLeft;
  bool blindSpotRight;
  bool compass;
  bool conditionalExperimental;
  bool experimentalMode;
  bool fullMapOpen;
  bool leadInfo;
  bool mapOpen;
  bool onroadAdjustableProfiles;
  bool pedalsOnUI;
  bool rightHandDrive;
  bool roadNameUI;
  bool showDriverCamera;
  bool showSLCOffset;
  bool slcOverridden;
  bool speedLimitController;
  bool turnSignalLeft;
  bool turnSignalRight;
  bool useViennaSLCSign;
  bool vtscControllingCurve;  
  // Variables for holding configuration and dynamic values for the UI elements
  float cruiseAdjustment;
  float distanceConversion;
  float laneWidthLeft;
  float laneWidthRight;
  float slcSpeedLimit;
  float slcSpeedLimitOffset;
  float speedConversion;

  int cameraView;
  int conditionalSpeed;
  int conditionalSpeedLead;
  int conditionalStatus;
  int customColors;
  int customSignals;
  int obstacleDistance;
  int obstacleDistanceStock;
  int totalFrames = 8;
  // Variables for storing units and labels for UI elements
  QString leadDistanceUnit;
  QString leadSpeedUnit;

  size_t animationFrameIndex; // Index for the current frame in an animation sequence

  // Configuration for theming and UI colors
  std::unordered_map<int, std::tuple<QString, int, QColor, std::map<double, QBrush>>> themeConfiguration;
  std::vector<QPixmap> signalImgVector; // Vector of images for turn signals

  QTimer *animationTimer; // Timer to handle UI animations

protected:
  // Overridden OpenGL functions for initializing and painting the OpenGL context
  void paintGL() override;
  void initializeGL() override;
  void paintEvent(QPaintEvent *event) override; // Overrides the QWidget paint event to handle custom painting of the UI elements.
  // Event handlers and UI update functions
  void showEvent(QShowEvent *event) override; // Called when the widget is shown; can be used to initialize certain UI elements or states.
  void updateFrameMat() override; // Called to update the frame matrix for the camera view, possibly recalculating the projection if necessary.
  // Drawing functions for various UI components related to driving visualization
  void drawLaneLines(QPainter &painter, const UIState *s); // Draws lane lines on the UI based on the current state.
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd); // Draws the lead vehicle information, including distance and speed.
  void paintAdjacentLanesAndBlindSpots(QPainter &painter);
  // Function to draw the entire heads-up display
  void drawHud(QPainter &p);
  // Function to draw the current speed and unit
  void drawSpeed(QPainter &p);
  // Function to draw the speed limit sign
  void drawSpeedLimitSign(QPainter &p);
  // Additional private functions for drawing other components
  void drawHeaderGradient(QPainter &p);
  void drawCarStateOverlay(QPainter &p);
  void updateWidgetAlignment(); // Method to update widget alignment dynamically
  void drawPolygonCoordinates(QPainter &painter, const QPolygonF &polygon, QColor textColor);
  double prev_draw_t = 0; // Holds the previous draw time; useful for calculating frame rates or animation intervals.
  FirstOrderFilter fps_filter; // A filter for smoothing out the frame-per-second calculations.
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }
  void showMapPanel(bool show) { if (map) map->setVisible(show); }

signals:
  void mapPanelRequested();

private:
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent* e) override;
  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;
  QString leadInfoLog; // To store the latest log information

  // FrogPilot widgets
  void updateFPSCounter();

  // FrogPilot variables
  UIScene &scene;
  Params paramsMemory{"/dev/shm/params"};

  double avgFPS;
  double maxFPS = 0.0;
  double minFPS = 99.9;

  QPoint timeoutPoint = QPoint(420, 69);
  QTimer clickTimer;

private slots:
  void offroadTransition(bool offroad);
  void primeChanged(bool prime);
  void updateState(const UIState &s);
  void handleLeadInfo(const QString &info);
};
