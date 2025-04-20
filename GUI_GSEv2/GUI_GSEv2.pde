// RocketTelemetryGUI.pde
// Main file for the Rocket Telemetry GUI
// Handles setup and main draw loop

import processing.serial.*;
import controlP5.*;
import java.io.OutputStream;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.ArrayList;

// -----------------------------------------
// Setup
// -----------------------------------------
void setup() {
  size(1024, 768, P3D);
  surface.setResizable(true);
  frameRate(30);
  
  initializeVariables();
  initializeUI();
  initializeLogging();
  
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
  
  println("Setup complete. Waiting for connection...");
  
  // Initialize trajectory with starting point
  trajectoryPoints.add(new PVector(0, 0, 0));
}

// -----------------------------------------
// Draw
// -----------------------------------------
void draw() {
  background(bgColor);
  
  // Update simulation if active
  if (simulationMode && flightActive) {
    updateSimulation();
  }
  
  // Update 3D trajectory panel position for resizing
  updatePanelSizes();
  
  // Draw main telemetry view
  drawTelemetryView();
  
  // Draw 3D trajectory in a smaller panel
  draw3DTrajectoryPanel();

  // Draw connection status
  drawCommStatus();
  
  // Draw state display
  drawStateDisplay();

  // Adjust UI components if window resized
  resizeComponents();
  
  // Check if mouse is over 3D trajectory panel
  trajectory3DPanelHover = mouseX >= trajectory3DPanelX && mouseX <= trajectory3DPanelX + trajectory3DPanelWidth &&
                           mouseY >= trajectory3DPanelY && mouseY <= trajectory3DPanelY + trajectory3DPanelHeight;
  
  // Disable depth test for UI components
  hint(DISABLE_DEPTH_TEST);
  cp5.draw(); 
  hint(ENABLE_DEPTH_TEST);
  
  // Check for parachute deployment based on state
  if (state == 5 || state == 6) { // DESCENT or ABORT
    parachuteDeployed = true;
  }
}

// Event handlers for mouse interaction
void mouseDragged() {
  if (trajectory3DPanelHover) {
    cameraAngleY += (mouseX - pmouseX) * 0.01;
    cameraAngleX += (mouseY - pmouseY) * 0.01;
    
    // Limit pitch to avoid gimbal lock
    cameraAngleX = constrain(cameraAngleX, -PI/2 + 0.1, PI/2 - 0.1);
  }
}

void mouseWheel(MouseEvent event) {
  if (trajectory3DPanelHover) {
    float e = event.getCount();
    cameraDistance += e * 20;
    cameraDistance = constrain(cameraDistance, 100, 2000);
  }
}

// Update panel sizes on window resize
void updatePanelSizes() {
  trajectory3DPanelWidth = (int)(width * 0.35);
  trajectory3DPanelHeight = (int)(height * 0.35);
  trajectory3DPanelX = width - trajectory3DPanelWidth - 20;
  trajectory3DPanelY = 70;
  
  graphPanelX = sidebarWidth + 20;
  graphPanelWidth = width - graphPanelX - 20;
}