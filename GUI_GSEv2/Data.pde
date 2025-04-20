// Data.pde
// Contains global variables, data structures, and initialization functions

// -----------------------------------------
// Global Variables
// -----------------------------------------
PShape rocket;
Serial port;
ControlP5 cp5;
DropdownList portDropdown;
Textarea consoleOutput;
Textarea serialOutput;
PrintWriter dataFile;
String filename;

// Telemetry
float yaw, pitch, roll, altitude, yawServo, pitchServo;

// Pyro 
boolean pyro1Continuity = false;
boolean pyro2Continuity = false;

//dump to sd
int dump = 0; // 0: no dump, 1: dump going, 2: dump finished

// State
int state = 0; // Default: PAD IDLE
String[] stateLabels = {
  "PAD IDLE",
  "CALIBRATION",
  "COUNTDOWN",
  "ASCENT",
  "UNPOWERED ASCENT",
  "DESCENT",
  "ABORT",
  "GROUND IDLE"
};
color[] stateColors = {
  color(150, 150, 150),  // PAD IDLE - gray
  color(100, 100, 255),  // CALIBRATION - blue
  color(255, 200, 0),    // COUNTDOWN - amber
  color(255, 100, 0),    // ASCENT - orange
  color(0, 200, 255),    // UNPOWERED ASCENT - cyan
  color(0, 200, 0),      // DESCENT - green
  color(255, 0, 0),       // ABORT - red
  color(255, 255, 255)   // GROUND IDLE - white
};

// Delta time
float dt = 0; // Time since last state change

// Status Indicators
boolean apogeeReached = false;
boolean parachuteDeployed = false;

// Graph Data
int maxPoints = 300;
float[] yawHistory = new float[maxPoints];
float[] pitchHistory = new float[maxPoints];
float[] rollHistory = new float[maxPoints];
float[] altitudeHistory = new float[maxPoints];
float[] yawServoHistory = new float[maxPoints];
float[] pitchServoHistory = new float[maxPoints];
float[] dtHistory = new float[maxPoints];

int currentIndex = 0;

// 3D Trajectory
ArrayList<PVector> trajectoryPoints = new ArrayList<PVector>();
float cameraDistance = 500;
float cameraAngleX = PI/4;
float cameraAngleY = -PI/4;
boolean showTrajectory = true;
float groundSize = 1000;
boolean trajectory3DPanelHover = false;
PGraphics trajectoryPG;

// Physics Parameters
float momentOfInertia = 0.1;  // Moment of inertia (kg*m²)
float momentArm = 0.49;       // Moment arm from CG (m)
float rocketMass = 1.0;       // Rocket mass (kg) - estimate
float gravity = 9.81;         // Gravity (m/s²)
float dragCoefficient = 0.5;  // Simplified drag coefficient
float rocketArea = 0.01;      // Cross-sectional area (m²) - estimate
float airDensity = 1.225;     // Air density at sea level (kg/m³)

// Thrust Curve (simplified as time and thrust pairs)
float[] thrustTimes = {0, 1, 2, 3, 4, 6};      // seconds
float[] thrustValues = {0, 30, 20, 10, 5, 0};  // Newtons
float currentTime = 0;
float timeStep = 0.1;  // simulation time step in seconds

// Current state
PVector position = new PVector(0, 0, 0);  // x, y, z in meters
PVector velocity = new PVector(0, 0, 0);  // m/s
PVector acceleration = new PVector(0, 0, 0);  // m/s²

// Flight state
boolean flightActive = false;
boolean simulationMode = false;
long flightStartTime = 0;

// Flags / Buffers
boolean commConnected = false;
String[] availablePorts;
String consoleBuffer = "";
String serialBuffer = "";

// UI Layout
int sidebarWidth = 150;
int graphPanelX;
int graphPanelWidth;
color bgColor;
PFont labelFont;
PFont stateLabelFont;
// 3D Trajectory Panel Dimensions
int trajectory3DPanelX;
int trajectory3DPanelY;
int trajectory3DPanelWidth;
int trajectory3DPanelHeight;

// -----------------------------------------
// Initialize Variables
// -----------------------------------------
void initializeVariables() {
  // UI initialization
  bgColor = color(20, 20, 30);
  labelFont = createFont("Arial", 12);
  stateLabelFont = createFont("Arial Bold", 18);
  graphPanelX = sidebarWidth + 20;
  graphPanelWidth = width - graphPanelX - 20;
  
  // Initialize 3D trajectory panel dimensions
  trajectory3DPanelWidth = (int)(width * 0.35);
  trajectory3DPanelHeight = (int)(height * 0.35);
  trajectory3DPanelX = width - trajectory3DPanelWidth - 20;
  trajectory3DPanelY = 70;
  
  // Initialize PGraphics for trajectory
  trajectoryPG = createGraphics(trajectory3DPanelWidth, trajectory3DPanelHeight, P3D);
  
  // Get available serial ports
  availablePorts = Serial.list();
  println("Available Serial Ports:");
  for (int i = 0; i < availablePorts.length; i++) {
    println(i + ": " + availablePorts[i]);
  }
}

// -----------------------------------------
// Initialize Data Logging
// -----------------------------------------
void initializeLogging() {
  String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
  filename = "flight_data_" + timestamp + ".csv";
  dataFile = createWriter(dataPath(filename));
  
  // Write CSV header
  dataFile.println("Timestamp,ElapsedMillis,Yaw,Pitch,Roll,Altitude,YawServo,PitchServo,Pyro1,Pyro2,DeltaTime,State");
  dataFile.flush();
  println("Logging to: " + dataPath(filename));
}