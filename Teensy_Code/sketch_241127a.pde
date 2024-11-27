PShape rocket; // 3D rocket model
float roll = 0, pitch = 0, yaw = 0;

// Data arrays to store graph points
int maxPoints = 300; // Number of points to display on the graph
float[] yawHistory = new float[maxPoints];
float[] pitchHistory = new float[maxPoints];
float[] rollHistory = new float[maxPoints];
int currentIndex = 0;

void setup() {
  size(1000, 800, P3D); // Increased window size to fit graphs
  frameRate(30);

  // Load the rocket model
  rocket = loadShape("rocket.obj"); // Replace with your model file name
  if (rocket == null) {
    println("Error loading rocket model. Ensure the file is in the data folder.");
    exit();
  }
}

void draw() {
  background(0);
  lights();

  // Simulate incoming data
  simulateData();

  // 3D Visualization
  pushMatrix();
  translate(width / 2, height / 2, -200);

  // Apply initial rotations to make the rocket vertical
  rotateY(-HALF_PI); // Rotate 90 degrees around the Y-axis
  rotateZ(HALF_PI); // Rotate 90 degrees around the Z-axis

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

void simulateData() {
  // Randomly adjust angles to simulate real-time updates
  roll += random(-2, 2);
  pitch += random(-2, 2);
  yaw += random(-2, 2);

  // Keep angles within a realistic range (-180 to 180 degrees)
  roll = constrain(roll, -180, 180);
  pitch = constrain(pitch, -90, 90);
  yaw = constrain(yaw, -180, 180);
}

void updateGraphs() {
  // Update the data arrays with the latest values
  yawHistory[currentIndex] = yaw;
  pitchHistory[currentIndex] = pitch;
  rollHistory[currentIndex] = roll;

  // Move the index forward, looping back if necessary
  currentIndex = (currentIndex + 1) % maxPoints;
}

void drawGraphs() {
  // Graph layout
  int graphWidth = 800;
  int graphHeight = 100;
  int graphX = 100;
  int graphY = 600; // Starting position for the first graph

  // Draw Yaw graph
  drawLineGraph(graphX, graphY, graphWidth, graphHeight, yawHistory, color(255, 0, 0), "Yaw", -180, 180);

  // Draw Pitch graph
  drawLineGraph(graphX, graphY + 120, graphWidth, graphHeight, pitchHistory, color(0, 255, 0), "Pitch", -90, 90);

  // Draw Roll graph
  drawLineGraph(graphX, graphY + 240, graphWidth, graphHeight, rollHistory, color(0, 0, 255), "Roll", -180, 180);
}

void drawLineGraph(int x, int y, int w, int h, float[] data, color c, String label, float minValue, float maxValue) {
  stroke(c);
  noFill();
  rect(x, y, w, h); // Draw graph border

  // Draw Y-axis labels
  fill(255);
  textSize(12);
  text(label, x - 40, y + h / 2); // Label on the left
  text(nf(maxValue, 0, 0), x - 30, y + 10); // Top label
  text(nf(minValue, 0, 0), x - 30, y + h - 5); // Bottom label
  text("0", x - 20, y + h / 2 + 5); // Center label

  // Draw the data line
  beginShape();
  for (int i = 0; i < maxPoints; i++) {
    int index = (currentIndex + i) % maxPoints;
    float value = map(data[index], minValue, maxValue, y + h, y); // Map data to graph height
    vertex(x + map(i, 0, maxPoints, 0, w), value);
  }
  endShape();
}

void displayTelemetry() {
  // Display simulated orientation data on screen
  fill(255);
  textSize(15);
  text("Yaw: " + nf(yaw, 1, 2), 10, 20);
  text("Pitch: " + nf(pitch, 1, 2), 10, 40);
  text("Roll: " + nf(roll, 1, 2), 10, 60);
}
