import processing.serial.*;
import controlP5.*;

PShape rocket;
Serial port;
float roll = 0, pitch = 0, yaw = 0;
float altitude = 0;
int yawServo = 0, pitchServo = 0;

// Data arrays for graphs
int maxPoints = 300;
float[] yawHistory = new float[maxPoints];
float[] pitchHistory = new float[maxPoints];
float[] rollHistory = new float[maxPoints];
float[] altitudeHistory = new float[maxPoints];
int currentIndex = 0;

boolean commConnected = false;
String rawSerialData = "";
String[] availablePorts;


ControlP5 cp5;
DropdownList portDropdown;

void setup() {
  // Set default size first
  size(800, 600, P3D); // Provide a default size initially (fixed values)
  
  
  

  // Now set the dynamic size for the sketch window
  surface.setSize(displayWidth * 3 / 4, displayHeight * 3 / 4); // Dynamically resize the window
  
  surface.setResizable(true);

  frameRate(30);

  rocket = loadShape("rocket.obj");
  if (rocket == null) {
    println("Error loading rocket model.");
    exit();
  }

  availablePorts = Serial.list();
  println("Available serial ports:");
  for (int i = 0; i < availablePorts.length; i++) {
    println(i + ": " + availablePorts[i]);
  }

  cp5 = new ControlP5(this);
  portDropdown = cp5.addDropdownList("portDropdown")
    .setPosition(20, 20)
    .setSize(200, 200)
    .setBarHeight(30)
    .setItemHeight(20)
    .setColorBackground(color(50))
    .setColorActive(color(100))
    .setLabel("Select COM Port");

  for (int i = 0; i < availablePorts.length; i++) {
    portDropdown.addItem(availablePorts[i], i);
  }
  
}


void draw() {
  background(0);
  lights();

  if (!commConnected) {
    fill(255);
    textSize(18);
    text("Select a COM port to connect.", width / 2 - 100, 30);
  }

  drawCommStatus();
  drawSerialTerminal();

  pushMatrix();
  translate(width / 2, height / 4, -300);
  
  rotateZ(PI);
  rotateZ(radians(yaw));
  rotateX(-radians(pitch));
  rotateY(radians(roll));
  scale(3);
  shape(rocket);
  popMatrix();

  drawGraphs();
  displayTelemetry();
}

void controlEvent(ControlEvent theEvent) {
  if (theEvent.getController().getName().equals("portDropdown")) {
    int selectedIndex = (int) theEvent.getValue();
    println("Selected port: " + availablePorts[selectedIndex]);
    connectToPort(selectedIndex);
  }
}

void connectToPort(int index) {
  try {
    if (port != null) {
      port.stop();
    }
    port = new Serial(this, availablePorts[index], 9600);
    port.bufferUntil('\n');
    commConnected = true;

    println("Connected to " + availablePorts[index]);
    portDropdown.setLock(true);
  }
  catch (Exception e) {
    println("Error connecting to port: " + availablePorts[index]);
    commConnected = false;
  }
}

void drawCommStatus() {
  fill(commConnected ? color(0, 255, 0) : color(255, 0, 0));
  rect(1000, 20, 150, 40);
  fill(255);
  textAlign(CENTER, CENTER);
  text(commConnected ? "Connected" : "Disconnected", 1075, 38);
}

void drawSerialTerminal() {
  fill(50);
  rect(20, 775, 1160, 200);
  fill(255);
  text("Serial Terminal (Raw Data):", 30, 790);
  text(rawSerialData, 30, 800);
}

void serialEvent(Serial port) {
  String data = port.readStringUntil('\n');
  if (data != null) {
    rawSerialData = data + rawSerialData;
    if (rawSerialData.length() > 5000) {
      rawSerialData = rawSerialData.substring(0, 5000);
    }
    parseTelemetry(data.trim());
  }
}

void parseTelemetry(String data) {
  try {
    String[] components = split(data, ';');
    if (components.length != 3) {
      throw new Exception("Invalid telemetry format");
    }

    // Parse YPR
    if (components[0].startsWith("YPR:")) {
      String[] yprValues = split(components[0].substring(4), ',');
      if (yprValues.length == 3) {
        yaw = float(yprValues[0]);
        pitch = float(yprValues[1]);
        roll = float(yprValues[2]);
      }
    }

    // Parse Altitude
    if (components[1].startsWith("Alt:")) {
      altitude = float(components[1].substring(5));
    }

    // Parse Servo
    if (components[2].startsWith("Servo:")) {
      String[] servoValues = split(components[2].substring(7), ',');
      if (servoValues.length == 2) {
        yawServo = int(servoValues[0]);
        pitchServo = int(servoValues[1]);
      }
    }

    updateGraphData();
  }
  catch (Exception e) {
    println("Error parsing telemetry: " + data);
    println("Exception: " + e.getMessage());
  }
}

void updateGraphData() {
  yawHistory[currentIndex] = yaw;
  pitchHistory[currentIndex] = pitch;
  rollHistory[currentIndex] = roll;
  altitudeHistory[currentIndex] = altitude;
  currentIndex = (currentIndex + 1) % maxPoints;
}

void drawGraphs() {
  int graphWidth = 800;
  int graphHeight = 100;
  int graphX = 200;
  int graphY = 420;

  drawLineGraph(graphX, graphY, graphWidth, graphHeight, yawHistory, color(255, 0, 0), "Yaw", -180, 180);
  drawLineGraph(graphX, graphY + 120, graphWidth, graphHeight, pitchHistory, color(0, 255, 0), "Pitch", -90, 90);
  drawLineGraph(graphX, graphY + 240, graphWidth, graphHeight, rollHistory, color(0, 0, 255), "Roll", -180, 180);
  drawLineGraph(graphX, graphY + 360, graphWidth, graphHeight, altitudeHistory, color(255, 255, 0), "Altitude", -10, 100);
}

void drawLineGraph(int x, int y, int w, int h, float[] data, color c, String label, float minValue, float maxValue) {
  stroke(c);
  noFill();
  rect(x, y, w, h);

  fill(255);
  text(label, x - 40, y + h / 2);

  beginShape();
  for (int i = 0; i < maxPoints; i++) {
    int index = (currentIndex + i) % maxPoints;
    float value = map(data[index], minValue, maxValue, y + h, y);
    vertex(x + map(i, 0, maxPoints, 0, w), value);
  }
  endShape();
}

void displayTelemetry() {
  fill(255);
  text("Yaw: " + nf(yaw, 1, 2), 10, 500);
  text("Pitch: " + nf(pitch, 1, 2), 10, 520);
  text("Roll: " + nf(roll, 1, 2), 10, 540);
  text("Altitude: " + nf(altitude, 1, 2), 10, 560);
  text("Yaw Servo: " + yawServo, 10, 580);
  text("Pitch Servo: " + pitchServo, 10, 600);
}
