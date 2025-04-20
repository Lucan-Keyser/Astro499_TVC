// UI.pde
// User Interface Components and Display Functions

// -----------------------------------------
// Initialize UI Components
// -----------------------------------------
void initializeUI() {
  // Initialize ControlP5
  cp5 = new ControlP5(this);
  
  // Create DropdownList for COM ports
  portDropdown = cp5.addDropdownList("portDropdown")
    .setPosition(10, 30)
    .setSize(sidebarWidth, 200)
    .setBarHeight(30)
    .setItemHeight(20)
    .setColorBackground(color(40, 40, 60))
    .setColorActive(color(60, 60, 100))
    .setColorForeground(color(60, 60, 80))
    .setLabel("SELECT PORT");

  // Populate dropdown with the serial ports
  for (int i = 0; i < availablePorts.length; i++) {
    portDropdown.addItem(availablePorts[i], i);
  }

  // Console & Serial Output UI
  consoleOutput = cp5.addTextarea("consoleOutput")
    .setPosition(width/2, height - 150)
    .setSize(width/2 - 20, 130)
    .setFont(createFont("Consolas", 11))
    .setLineHeight(13)
    .setColor(color(220))
    .setColorBackground(color(30, 30, 40))
    .setColorForeground(color(40, 40, 60))
    .setLabel("SYSTEM LOG");

  serialOutput = cp5.addTextarea("serialOutput")
    .setPosition(10, height - 150)
    .setSize(width/2 - 20, 130)
    .setFont(createFont("Consolas", 11))
    .setLineHeight(13)
    .setColor(color(220))
    .setColorBackground(color(30, 30, 40))
    .setColorForeground(color(40, 40, 60))
    .setLabel("SERIAL DATA");

  // Command Buttons
  int buttonY = 80;
  int buttonSpacing = 45;
  
  cp5.addButton("Reset")
    .setPosition(10, buttonY)
    .setSize(sidebarWidth, 35)
    .setLabel("RESET")
    .setColorBackground(color(60, 60, 100));
  
  cp5.addButton("calibration")
    .setPosition(10, buttonY + buttonSpacing)
    .setSize(sidebarWidth, 35)
    .setLabel("CALIBRATION")
    .setColorBackground(color(60, 60, 150));
  
  cp5.addButton("launchRocket")
    .setPosition(10, buttonY + buttonSpacing*2)
    .setSize(sidebarWidth, 35)
    .setColorBackground(color(200, 30, 30))
    .setLabel("LAUNCH");

  cp5.addButton("manualSeparation")
    .setPosition(10, buttonY + buttonSpacing*3)
    .setSize(sidebarWidth, 35)
    .setColorBackground(color(60, 60, 100))
    .setLabel("SEPARATE");
    
  cp5.addButton("abortMission")
    .setPosition(10, buttonY + buttonSpacing*4)
    .setSize(sidebarWidth, 35)
    .setColorBackground(color(255, 0, 0))
    .setLabel("ABORT");

  cp5.addButton("disconnectPort")
    .setPosition(10, buttonY + buttonSpacing*5)
    .setSize(sidebarWidth, 35)
    .setColorBackground(color(60, 60, 100))
    .setLabel("DISCONNECT");
    
  // Simulation Toggle
  cp5.addButton("toggleSimulation")
    .setPosition(10, buttonY + buttonSpacing*6)
    .setSize(sidebarWidth, 35)
    .setColorBackground(color(100, 60, 60))
    .setLabel("SIMULATION OFF");
}

// -----------------------------------------
// Draw Main Telemetry View
// -----------------------------------------
void drawTelemetryView() {
  lights();

  // Draw rocket model with current orientation
  pushMatrix();
    translate(width/2, height/3, 0);
    rotateZ(PI);
    rotateZ(radians(yaw));
    rotateX(-radians(pitch));
    rotateY(radians(roll));
    scale(4);
    shape(rocket);
  popMatrix();

  // Draw telemetry line graphs
  drawGraphs();

  // Draw textual telemetry readout
  displayTelemetry();
  
  // Draw status indicators
  drawStatusIndicators();
}

// -----------------------------------------
// Draw State Display
// -----------------------------------------
void drawStateDisplay() {
  int stateDisplayX = sidebarWidth + 30;
  int stateDisplayY = trajectory3DPanelY + trajectory3DPanelHeight + 20;
  int stateDisplayWidth = 300;
  int stateDisplayHeight = 80;
  
  // State panel background
  stroke(80, 80, 80);
  strokeWeight(2);
  fill(stateColors[state], 180);
  rect(stateDisplayX, stateDisplayY, stateDisplayWidth, stateDisplayHeight, 10);
  
  // State label
  textFont(stateLabelFont);
  fill(255);
  textAlign(CENTER, CENTER);
  text("STATE: " + stateLabels[state], stateDisplayX + stateDisplayWidth/2, stateDisplayY + 25);
  
  // Delta time
  textFont(labelFont);
  textAlign(CENTER, CENTER);
  text("TIME IN STATE: " + nf(dt, 1, 2) + " ms", stateDisplayX + stateDisplayWidth/2, stateDisplayY + 55);
}

// -----------------------------------------
// Display Telemetry Text
// -----------------------------------------
void displayTelemetry() {
  fill(255);
  textAlign(LEFT, CENTER);
  textFont(labelFont);
  
  // Telemetry panel
  int telX = sidebarWidth + 30;
  int telY = height - 250;
  int colWidth = 130;
  
  fill(40, 40, 60, 150);
  noStroke();
  rect(telX - 10, telY - 20, 300, 190, 5);
  
  // Column 1
  fill(255);
  text("YAW:", telX, telY);
  text("PITCH:", telX, telY + 25);
  text("ROLL:", telX, telY + 50);
  
  // Column 1 Values
  fill(255, 100, 100);
  text(nf(yaw, 1, 2) + "°", telX + 70, telY);
  fill(100, 255, 100);
  text(nf(pitch, 1, 2) + "°", telX + 70, telY + 25);
  fill(100, 100, 255);
  text(nf(roll, 1, 2) + "°", telX + 70, telY + 50);
  
  // Column 2
  fill(255);
  text("ALTITUDE:", telX + colWidth, telY);
  text("YAW SERVO:", telX + colWidth, telY + 25);
  text("PITCH SERVO:", telX + colWidth, telY + 50);
  
  // Column 2 Values
  fill(255, 255, 100);
  text(nf(altitude, 1, 2) + " m", telX + colWidth + 100, telY);
  fill(255, 160, 100);
  text(nf(yawServo, 1, 2) + "°", telX + colWidth + 100, telY + 25);
  fill(160, 100, 255);
  text(nf(pitchServo, 1, 2) + "°", telX + colWidth + 100, telY + 50);
}

// -----------------------------------------
// Status Indicators
// -----------------------------------------
void drawStatusIndicators() {
  int indicatorX = 10;
  int indicatorY = 400;
  int indicatorSize = 20;
  int textOffsetX = 35;
  int spacing = 30;
  
  // Apogee Indicator
  noStroke();
  fill(apogeeReached ? color(0, 255, 0) : color(255, 30, 30));
  ellipse(indicatorX + indicatorSize/2, indicatorY + indicatorSize/2, indicatorSize, indicatorSize);
  fill(255);
  textAlign(LEFT, CENTER);
  text("APOGEE DETECTED", indicatorX + textOffsetX, indicatorY + indicatorSize/2);
  
  // Parachute Indicator
  fill(parachuteDeployed ? color(0, 255, 0) : color(255, 30, 30));
  ellipse(indicatorX + indicatorSize/2, indicatorY + spacing + indicatorSize/2, indicatorSize, indicatorSize);
  fill(255);
  text("PARACHUTE DEPLOYED", indicatorX + textOffsetX, indicatorY + spacing + indicatorSize/2);
  
  // Pyro Continuity Indicators
  fill(pyro1Continuity ? color(0, 255, 0) : color(255, 30, 30));
  ellipse(indicatorX + indicatorSize/2, indicatorY + spacing*2 + indicatorSize/2, indicatorSize, indicatorSize);
  fill(255);
  text("PYRO 1 CONTINUITY", indicatorX + textOffsetX, indicatorY + spacing*2 + indicatorSize/2);
  
  fill(pyro2Continuity ? color(0, 255, 0) : color(255, 30, 30));
  ellipse(indicatorX + indicatorSize/2, indicatorY + spacing*3 + indicatorSize/2, indicatorSize, indicatorSize);
  fill(255);
  text("PYRO 2 CONTINUITY", indicatorX + textOffsetX, indicatorY + spacing*3 + indicatorSize/2);
  

  //sd dump indicator
    fill(dump == 1 ? color(0, 255, 0) : color(255, 30, 30));
    ellipse(indicatorX + indicatorSize/2, indicatorY + spacing*5 + indicatorSize/2, indicatorSize, indicatorSize);
    fill(255);
    text("SD DUMP GOING", indicatorX + textOffsetX, indicatorY + spacing*5 + indicatorSize/2);
    

    fill(dump == 2 ? color(0, 255, 0) : color(255, 30, 30));
    ellipse(indicatorX + indicatorSize/2, indicatorY + spacing*6 + indicatorSize/2, indicatorSize, indicatorSize);
    fill(255);
    text("SD DUMP FINISHED", indicatorX + textOffsetX, indicatorY + spacing*6 + indicatorSize/2);

// Flight Active Indicator right side of screen
  fill(flightActive ? color(0, 255, 0) : color(255, 30, 30));
  ellipse(indicatorX + indicatorSize/2, indicatorY + spacing*4 + indicatorSize/2, indicatorSize, indicatorSize);
  fill(255);
  text("FLIGHT ACTIVE", indicatorX + textOffsetX, indicatorY + spacing*4 + indicatorSize/2);

}

// -----------------------------------------
// Connection Status Display
// -----------------------------------------
void drawCommStatus() {
  int statusX = width - 160;
  int statusY = 30;
  int statusWidth = 140;
  int statusHeight = 35;
  
  // Background
  noStroke();
  fill(commConnected ? color(40, 120, 40) : color(120, 40, 40));
  rect(statusX, statusY, statusWidth, statusHeight, 5);
  
  // Text
  fill(255);
  textAlign(CENTER, CENTER);
  textFont(labelFont);
  text(commConnected ? "CONNECTED" : "DISCONNECTED", statusX + statusWidth/2, statusY + statusHeight/2);
}

// -----------------------------------------
// Resize UI Components
// -----------------------------------------
void resizeComponents() {
  // Update text areas
  consoleOutput.setPosition(width/2, height - 150);
  consoleOutput.setSize(width/2 - 20, 130);

  serialOutput.setPosition(10, height - 150);
  serialOutput.setSize(width/2 - 20, 130);
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

// Add data to the serial output text area
void addToSerialOutput(String data) {
  serialBuffer += data + "\n";
  
  // Keep buffer size reasonable
  if (serialBuffer.length() > 8000) {
    serialBuffer = serialBuffer.substring(serialBuffer.length() - 8000);
  }
  
  serialOutput.setText(serialBuffer);
  
  // Auto-scroll
  int totalLines = serialBuffer.split("\n").length;
  serialOutput.scroll(totalLines);
}