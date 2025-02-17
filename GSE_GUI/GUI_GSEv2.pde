import processing.serial.*;
import controlP5.*;
import java.io.OutputStream;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
PrintWriter dataFile;
boolean fileInitialized = false;
String filename;

// -----------------------------------------
// Global Variables
// -----------------------------------------
PShape rocket;
Serial port;
ControlP5 cp5;
DropdownList portDropdown;
Textarea consoleOutput;
Textarea serialOutput;

// Telemetry
float yaw, pitch, roll, altitude, yawServo, pitchServo;

//Pyro 
boolean pyro1Continuity = false;
boolean pyro2Continuity = false;



// Graph Data
int maxPoints = 300;
float[] yawHistory = new float[maxPoints];
float[] pitchHistory = new float[maxPoints];
float[] rollHistory = new float[maxPoints];
float[] altitudeHistory = new float[maxPoints];
int currentIndex = 0;

// Flags / Buffers
boolean commConnected = false;
String[] availablePorts;
String consoleBuffer = "";
String serialBuffer = "";

// -----------------------------------------
// Setup
// -----------------------------------------
void setup() {
  size(800, 600, P3D);
  surface.setSize(displayWidth * 3/4, displayHeight * 3/4);
  surface.setResizable(true);
  frameRate(30);

  // Initialize ControlP5
  cp5 = new ControlP5(this);

  // --- Populate Serial Ports ---
  availablePorts = Serial.list();
  println("Available Serial Ports:");
  for (int i = 0; i < availablePorts.length; i++) {
    println(i + ": " + availablePorts[i]);
  }

  // Create DropdownList for COM ports
  portDropdown = cp5.addDropdownList("portDropdown")
    .setPosition(20, 20)
    .setSize(200, 200)
    .setBarHeight(30)
    .setItemHeight(20)
    .setColorBackground(color(50))
    .setColorActive(color(100))
    .setLabel("Select COM Port");

  // Populate dropdown with the serial ports
  for (int i = 0; i < availablePorts.length; i++) {
    portDropdown.addItem(availablePorts[i], i);
  }

  // Console Output UI
  consoleOutput = cp5.addTextarea("consoleOutput")
    .setPosition((width - width/2) + 10, height - 150)
    .setSize((width/2) - 30, 130)
    .setFont(createFont("Arial", 12))
    .setLineHeight(14)
    .setColor(color(255))
    .setColorBackground(color(50))
    .setColorForeground(color(200));

  // Serial Output UI
  serialOutput = cp5.addTextarea("serialOutput")
    .setPosition(10, height - 150)
    .setSize((width/2) - 30, 130)
    .setFont(createFont("Arial", 12))
    .setLineHeight(14)
    .setColor(color(255))
    .setColorBackground(color(50))
    .setColorForeground(color(200));

  // Buttons
  cp5.addButton("armRocket")
    .setPosition(20, 150)
    .setSize(120, 40)
    .setLabel("Arm Rocket");

  cp5.addButton("launchRocket")
    .setPosition(20, 200)
    .setSize(120, 40)
    .setLabel("Launch");

  cp5.addButton("manualSeparation")
    .setPosition(20, 250)
    .setSize(120, 40)
    .setLabel("Separate");

  // Disconnect button
  cp5.addButton("disconnectPort")
    .setPosition(20, 300)
    .setSize(120, 40)
    .setLabel("Disconnect");

  // Toggles
  cp5.addToggle("apogeeReached")
    .setPosition(20, 380)
    .setSize(120, 40)
    .setLabel("Apogee Reached")
    .setValue(false);

  cp5.addToggle("parachuteDeployed")
    .setPosition(20, 430)
    .setSize(120, 40)
    .setLabel("Parachute Deployed")
    .setValue(false);

  // Load the 3D model
  try {
    rocket = loadShape("rocket.obj");
    if (rocket == null) {
      throw new RuntimeException("Failed to load rocket.obj");
    }
  } catch (Exception e) {
    println("Error: " + e.getMessage());
    exit(); // Exit if the model cannot be loaded
  }

  // Redirect console output to GUI
  redirectConsoleToGUI();

  // Print initial message
  println("Setup complete. Waiting for actions...");
  
  
  String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
  filename = "flight_data_" + timestamp + ".csv";
  dataFile = createWriter(dataPath(filename));
  
  // Write CSV header
  dataFile.println("Timestamp,ElapsedMillis,Yaw,Pitch,Roll,Altitude,YawServo,PitchServo");
  dataFile.flush(); // Force write to disk
  println("Logging to: " + dataPath(filename));

}

// -----------------------------------------
// Draw
// -----------------------------------------
void draw() {
  background(0);
  lights();

  // If no COM port selected/connected yet
  if (!commConnected) {
    fill(255);
    textSize(18);
    text("Select a COM port to connect.", width - width / 10 - 200, 38);
  }

  // Draw connection status
  drawCommStatus();

  // Adjust console/serial panel positions if window resized
  resizeComponents();

  // Draw rocket model with current orientation
  pushMatrix();
    translate(width/2, height/4, -300);
    rotateZ(PI);
    rotateZ(radians(yaw));
    rotateX(-radians(pitch));
    rotateY(radians(roll));
    scale(3);
    shape(rocket);
  popMatrix();

  // Draw telemetry line graphs
  drawGraphs();

  // Draw textual telemetry readout
  displayTelemetry();

  // Disable depth test, draw CP5, re-enable
  hint(DISABLE_DEPTH_TEST);
  cp5.draw(); 
  hint(ENABLE_DEPTH_TEST);
}

// -----------------------------------------
// ControlP5 Events (Buttons/Dropdown/Toggles)
// -----------------------------------------
void controlEvent(ControlEvent theEvent) {
  String eventName = theEvent.getController().getName();

  // 1) Dropdown selection
  if (eventName.equals("portDropdown")) {
    int selectedIndex = (int) theEvent.getValue();
    println("Selected port: " + availablePorts[selectedIndex]);
    connectToPort(selectedIndex);

    //  A) Make sure the dropdown closes after selection
    portDropdown.close();

    //  B) Optionally, consider removing portDropdown.setLock(true) altogether
    //     or do it here (AFTER it's closed). For now let's skip it.
  }

  // 2) Buttons
  else if (eventName.equals("armRocket")) {
    println("Rocket armed");
    if (port != null) {
      port.write("ARM\n");
    }
  }
  else if (eventName.equals("launchRocket")) {
    println("Rocket launched");
    if (port != null) {
      port.write("LAUNCH\n");
    }
  }
  else if (eventName.equals("manualSeparation")) {
    println("Manual separation triggered");
    if (port != null) {
      port.write("SEPARATE\n");
    }
  }
  else if (eventName.equals("disconnectPort")) {
    disconnectFromPort();
  }
}

// -----------------------------------------
// Connect / Disconnect from Serial
// -----------------------------------------
void connectToPort(int index) {
  try {
    if (port != null) {
      port.stop();
    }
    port = new Serial(this, availablePorts[index], 9600);
    port.bufferUntil('\n');
    commConnected = true;

    println("Connected to " + availablePorts[index]);
    
    // We used to lock the dropdown here, but let's skip it for now,
    // or do it only after we close it to avoid it stuck in "open" state:
    // portDropdown.setLock(true);
  }
  catch (Exception e) {
    println("Error connecting to port: " + availablePorts[index]);
    commConnected = false;
  }
}

void disconnectFromPort() {
  if (port != null) {
    println("Disconnecting from port...");
    port.stop();
    port = null;
  }
  commConnected = false;
  // Let the user pick another port if needed
  portDropdown.setLock(false);
  println("Disconnected.");
}

// -----------------------------------------
// Serial Event (Modified)
// -----------------------------------------
void serialEvent(Serial port) {
  try {
    String rawData = port.readStringUntil('\n');
    if (rawData != null) {
      rawData = rawData.trim();
      String[] parts = split(rawData, ',');
      
      if (parts.length == 6) {
        // Parse values
        yaw = float(parts[0]);
        pitch = float(parts[1]);
        roll = float(parts[2]);
        altitude = float(parts[3]);
        yawServo = float(parts[4]);
        pitchServo = float(parts[5]);
        
        // Update display
        updateGraphData();
        
        // Log to file
        String timestamp = new SimpleDateFormat("HH:mm:ss.SSS").format(new Date());
        String dataLine = String.join(",",
          timestamp,
          str(millis()),
          parts[0],  // Yaw
          parts[1],  // Pitch
          parts[2],  // Roll
          parts[3],   // Altitude
          parts[4],
          parts[5]
        );
        
        dataFile.println(dataLine);
        dataFile.flush(); // Ensure data is written immediately
      }
    }
  } catch (Exception e) {
    println("Error: " + e);
  }
}




// -----------------------------------------
// Display Telemetry Text
// -----------------------------------------
void displayTelemetry() {
  fill(255);
  text("Yaw: " + nf(yaw, 1, 2) + "°",      60, 500);
  text("Pitch: " + nf(pitch, 1, 2) + "°", 63, 520);
  text("Roll: " + nf(roll, 1, 2) + "°",   60, 540);
  text("Altitude: " + nf(altitude, 1, 2), 73, 560);
  text("Yaw Servo: " + yawServo + "°",    73, 580);
  text("Pitch Servo: " + pitchServo + "°",75, 600);
    // Continuity Status
  fill(pyro1Continuity ? color(0, 255, 0) : color(255, 0, 0));
  text("Pyro1 Continuity: " + (pyro1Continuity ? "OK" : "NO"), 100, 620);

  fill(pyro2Continuity ? color(0, 255, 0) : color(255, 0, 0));
  text("Pyro2 Continuity: " + (pyro2Continuity ? "OK" : "NO"), 100, 640);
}

// -----------------------------------------
// Update Toggles Programmatically
// -----------------------------------------
void updateIndicators(boolean apogee, boolean parachute) {
  cp5.get(Toggle.class, "apogeeReached").setValue(apogee);
  cp5.get(Toggle.class, "parachuteDeployed").setValue(parachute);
}

// -----------------------------------------
// Graph Helpers
// -----------------------------------------
void updateGraphData() {
  yawHistory[currentIndex]      = yaw;
  pitchHistory[currentIndex]    = pitch;
  rollHistory[currentIndex]     = roll;
  altitudeHistory[currentIndex] = altitude;
  currentIndex = (currentIndex + 1) % maxPoints;
}

void drawGraphs() {
  int graphWidth   = width / 6;
  int graphHeight  = height / 12;
  int graphX       = width - width / 5;
  int graphY       = height / 10;
  int graphSpacing = height / 10;

  drawLineGraph(graphX, graphY,             graphWidth, graphHeight,
                yawHistory,   color(255, 0, 0),   "Yaw °",   -180, 180);

  drawLineGraph(graphX, graphY + graphSpacing,   graphWidth, graphHeight,
                pitchHistory, color(0, 255, 0),   "Pitch °", -90, 90);

  drawLineGraph(graphX, graphY + graphSpacing*2, graphWidth, graphHeight,
                rollHistory,  color(0, 0, 255),   "Roll °",  -180, 180);

  drawLineGraph(graphX, graphY + graphSpacing*3, graphWidth, graphHeight,
                altitudeHistory, color(255, 255, 0), "Altitude", -10, 100);
}

void drawLineGraph(int x, int y, int w, int h,
                   float[] data, color c, String label,
                   float minValue, float maxValue) {
  stroke(c);
  noFill();
  rect(x, y, w, h);

  fill(255);
  text(label, x - 40, y + h / 2);

  beginShape();
  for (int i = 0; i < maxPoints; i++) {
    int index = (currentIndex + i) % maxPoints;
    float mappedY = map(data[index], minValue, maxValue, y + h, y);
    float mappedX = map(i, 0, maxPoints, x, x + w);
    vertex(mappedX, mappedY);
  }
  endShape();
}

// -----------------------------------------
// UI / Layout Helpers
// -----------------------------------------
void drawCommStatus() {
  fill(commConnected ? color(0, 255, 0) : color(255, 0, 0));
  rect((width - width/10) - 75, 20, 150, 40);
  fill(255);
  textAlign(CENTER, CENTER);
  text(commConnected ? "Connected" : "Disconnected", width - width/10, 38);
}

void resizeComponents() {
  consoleOutput.setPosition((width - width/2) + 10, height - 150);
  consoleOutput.setSize((width/2) - 30, 130);

  serialOutput.setPosition(10, height - 150);
  serialOutput.setSize((width/2) - 30, 130);
}

// -----------------------------------------
// Redirect Console to GUI
// -----------------------------------------
void redirectConsoleToGUI() {
  PrintStream customStream = new PrintStream(new OutputStream() {
    public void write(int b) {
      consoleBuffer += (char) b;

      // Keep console buffer size reasonable
      if (consoleBuffer.length() > 8000) {
        consoleBuffer = consoleBuffer.substring(consoleBuffer.length() - 8000);
      }

      consoleOutput.setText(consoleBuffer);

      // Auto-scroll
      int totalLines = consoleBuffer.split("\n").length;
      consoleOutput.scroll(totalLines);
    }
  });

  System.setOut(customStream);
  System.setErr(customStream);
}
