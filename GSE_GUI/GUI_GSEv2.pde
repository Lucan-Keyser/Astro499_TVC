import processing.serial.*;
import controlP5.*; // Import ControlP5 for GUI elements

PShape rocket; // 3D rocket model
Serial port;   // Serial connection
float roll = 0, pitch = 0, yaw = 0; // Euler angles

// Data arrays to store graph points
int maxPoints = 300; // Number of points to display on the graph
float[] yawHistory = new float[maxPoints];
float[] pitchHistory = new float[maxPoints];
float[] rollHistory = new float[maxPoints];
int currentIndex = 0;

// Communication state
boolean commConnected = false; // Status of communication
String rawSerialData = "";     // Raw serial data buffer
String[] availablePorts;       // Array to store available ports

ControlP5 cp5;                 // GUI library instance
DropdownList portDropdown;     // Dropdown list for selecting COM port

void setup() {
  size(1200, 1000, P3D);
  frameRate(30);

  // Load the rocket model
  rocket = loadShape("rocket.obj"); // Replace with your model file name
  if (rocket == null) {
    println("Error loading rocket model. Ensure the file is in the data folder.");
    exit();
  }

  // List available serial ports
  availablePorts = Serial.list();
  if (availablePorts.length == 0) {
    println("No serial ports available.");
  }
  println("Available serial ports:");
  for (int i = 0; i < availablePorts.length; i++) {
    println(i + ": " + availablePorts[i]);
  }

  // Initialize ControlP5 and add a dropdown list for port selection
  cp5 = new ControlP5(this);
  portDropdown = cp5.addDropdownList("portDropdown")
                    .setPosition(20, 20)
                    .setSize(200, 200)
                    .setBarHeight(30)
                    .setItemHeight(20)
                    .setColorBackground(color(50))
                    .setColorActive(color(100))
                    .setLabel("Select COM Port");

  // Populate the dropdown list with available ports
  for (int i = 0; i < availablePorts.length; i++) {
    portDropdown.addItem(availablePorts[i], i);
  }

  // Add a listener for port selection
  portDropdown.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction() == ControlP5.ACTION_BROADCAST) {
        int selectedIndex = (int) portDropdown.getValue();
        println("Selected port: " + availablePorts[selectedIndex]);
        connectToPort(selectedIndex);
      }
    }
  });
}

void draw() {
  background(0);

  // Display COM selection instructions if not connected
  if (!commConnected) {
    fill(255);
    textSize(18);
    textAlign(LEFT, TOP);
    text("Select a COM port to connect.", 725, 30);
  }

  lights();

  // Draw communication status
  drawCommStatus();

  // Draw raw serial terminal
  drawSerialTerminal();

  // 3D Visualization
  pushMatrix();
  translate(width / 2, height / 4, -300);

  // Apply initial rotations to make the rocket vertical
  rotateY(-HALF_PI);
  rotateZ(HALF_PI);

  // Apply live rotations (yaw, pitch, roll)
  rotateZ(radians(yaw));
  rotateX(radians(pitch));
  rotateY(radians(roll));

  // Scale the rocket by 3x and draw it
  scale(3);
  shape(rocket);
  popMatrix();

  // Update and draw graphs
  updateGraphs();
  drawGraphs();

  // Display telemetry data
  displayTelemetry();
}

void connectToPort(int index) {
  try {
    if (port != null) {
      port.stop(); // Stop any existing port connection
    }
    port = new Serial(this, availablePorts[index], 9600);
    port.bufferUntil('\n'); // Read until newline character
    commConnected = true;

    // Disable the dropdown after selection
    portDropdown.setLock(true);
  } catch (Exception e) {
    println("Error connecting to port: " + availablePorts[index]);
    commConnected = false;
  }
}

void drawCommStatus() {
  fill(commConnected ? color(0, 255, 0) : color(255, 0, 0)); // Green if connected, red if not
  rect(1000, 20, 150, 40);
  fill(255);
  textSize(15);
  textAlign(CENTER, CENTER);
  text(commConnected ? "Connected" : "Disconnected", 1075, 38);
}

void drawSerialTerminal() {
  fill(50);
  rect(20, 775, 1160, 200);
  fill(255);
  textSize(12);
  textAlign(LEFT, TOP);
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
    parseTelemetry(data);
    commConnected = true;
  }
}

void parseTelemetry(String data) {
  try {
    String[] values = split(trim(data), ','); // Split by commas
    if (values.length == 3) { // Expecting yaw, pitch, roll
      yaw = float(split(values[0], ':')[1]);
      pitch = float(split(values[1], ':')[1]);
      roll = float(split(values[2], ':')[1]);

      // Update graphs
      yawHistory[currentIndex] = yaw;
      pitchHistory[currentIndex] = pitch;
      rollHistory[currentIndex] = roll;
      currentIndex = (currentIndex + 1) % maxPoints;
    }
  } catch (Exception e) {
    println("Error parsing telemetry: " + data);
  }
}

void updateGraphs() {
  // Graphs are updated in parseTelemetry()
}

void drawGraphs() {
  int graphWidth = 800;
  int graphHeight = 100;
  int graphX = 200;
  int graphY = 420;

  drawLineGraph(graphX, graphY, graphWidth, graphHeight, yawHistory, color(255, 0, 0), "Yaw", -180, 180);
  drawLineGraph(graphX, graphY + 120, graphWidth, graphHeight, pitchHistory, color(0, 255, 0), "Pitch", -90, 90);
  drawLineGraph(graphX, graphY + 240, graphWidth, graphHeight, rollHistory, color(0, 0, 255), "Roll", -180, 180);
}

void drawLineGraph(int x, int y, int w, int h, float[] data, color c, String label, float minValue, float maxValue) {
  stroke(c);
  noFill();
  rect(x, y, w, h);

  fill(255);
  textSize(12);
  textAlign(LEFT, CENTER);
  text(label, x - 40, y + h / 2);
  text(nf(maxValue, 0, 0), x - 30, y + 10);
  text(nf(minValue, 0, 0), x - 30, y + h - 5);
  text("0", x - 20, y + h / 2 + 5);

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
  textSize(15);
  textAlign(LEFT, TOP);
  text("Yaw: " + nf(yaw, 1, 2), 10, 500);
  text("Pitch: " + nf(pitch, 1, 2), 10, 520);
  text("Roll: " + nf(roll, 1, 2), 10, 540);
}
