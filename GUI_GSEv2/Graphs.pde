// Graphs.pde
// Graph Drawing and Data Visualization Functions

// -----------------------------------------
// Update Graph Data
// -----------------------------------------
void updateGraphData() {
  yawHistory[currentIndex] = yaw;
  pitchHistory[currentIndex] = pitch;
  rollHistory[currentIndex] = roll;
  altitudeHistory[currentIndex] = altitude;
  yawServoHistory[currentIndex] = yawServo;
  pitchServoHistory[currentIndex] = pitchServo;
  dtHistory[currentIndex] = dt;
  currentIndex = (currentIndex + 1) % maxPoints;
}

// -----------------------------------------
// Draw All Graphs
// -----------------------------------------
void drawGraphs() {
  int graphWidth = (int)(width * 0.25) - 20;
  int graphHeight = 80;
  int graphY = 30;
  int graphSpacing = graphHeight + 20;
  int leftColX = sidebarWidth + 20;
  int rightColX = leftColX + graphWidth + 20;
  
  // Attitude graphs
  drawLineGraph(leftColX, graphY, graphWidth, graphHeight,
                yawHistory, color(255, 100, 100), "YAW (°)", -180, 180);

  drawLineGraph(leftColX, graphY + graphSpacing, graphWidth, graphHeight,
                pitchHistory, color(100, 255, 100), "PITCH (°)", -90, 90);

  drawLineGraph(leftColX, graphY + graphSpacing*2, graphWidth, graphHeight,
                rollHistory, color(100, 100, 255), "ROLL (°)", -180, 180);

  // Other telemetry graphs  
  drawLineGraph(rightColX, graphY, graphWidth, graphHeight,
                altitudeHistory, color(255, 255, 100), "ALTITUDE (m)", 0, max(100, altitude * 1.2));
  
  drawLineGraph(rightColX, graphY + graphSpacing, graphWidth, graphHeight,
                yawServoHistory, color(255, 160, 100), "YAW SERVO (°)", -90, 90);
                
  drawLineGraph(rightColX, graphY + graphSpacing*2, graphWidth, graphHeight,
                pitchServoHistory, color(160, 100, 255), "PITCH SERVO (°)", -90, 90);

  drawLineGraph(rightColX, graphY + graphSpacing*3, graphWidth, graphHeight,
                dtHistory, color(255, 200, 50), "DELTA TIME (ms)", 0, max(50, dt * 1.5));
}

// -----------------------------------------
// Draw Single Line Graph
// -----------------------------------------
void drawLineGraph(int x, int y, int w, int h,
                   float[] data, color c, String label,
                   float minValue, float maxValue) {
  // Graph background
  fill(40, 40, 60, 150);
  noStroke();
  rect(x, y, w, h, 5);
  
  // Title
  fill(255);
  textAlign(LEFT, CENTER);
  textFont(labelFont);
  text(label, x + 10, y - 10);
  
  // Y-axis labels
  textAlign(RIGHT, CENTER);
  textSize(10);
  text(nf(maxValue, 1, 0), x - 5, y + 10);
  text(nf(minValue, 1, 0), x - 5, y + h - 10);
  
  // Grid lines
  stroke(100, 100, 100, 80);
  strokeWeight(1);
  // Horizontal grid lines
  for (int i = 1; i < 4; i++) {
    float yPos = y + (h * i / 4.0);
    line(x, yPos, x + w, yPos);
  }
  // Vertical grid lines
  for (int i = 1; i < 4; i++) {
    float xPos = x + (w * i / 4.0);
    line(xPos, y, xPos, y + h);
  }
  
  // Draw graph line
  stroke(c);
  strokeWeight(2);
  noFill();
  beginShape();
  for (int i = 0; i < maxPoints; i++) {
    int index = (currentIndex + i) % maxPoints;
    float mappedY = map(data[index], minValue, maxValue, y + h - 10, y + 10);
    float mappedX = map(i, 0, maxPoints, x + 10, x + w - 10);
    vertex(mappedX, mappedY);
  }
  endShape();
  
  // Current value display
  int lastIndex = (currentIndex - 1 + maxPoints) % maxPoints;
  float currentValue = data[lastIndex];
  fill(c);
  textAlign(RIGHT, CENTER);
  text(nf(currentValue, 1, 1), x + w - 10, y + 20);
}

// -----------------------------------------
// Draw 3D Trajectory Panel
// -----------------------------------------
void draw3DTrajectoryPanel() {
  // Panel background
  stroke(80, 80, 80);
  strokeWeight(2);
  fill(40, 40, 60, 200);
  rect(trajectory3DPanelX, trajectory3DPanelY, trajectory3DPanelWidth, trajectory3DPanelHeight, 5);
  
  // Panel title
  fill(255);
  textAlign(LEFT, CENTER);
  textFont(labelFont);
  text("3D TRAJECTORY", trajectory3DPanelX + 10, trajectory3DPanelY - 10);
  
  // Draw in a separate PGraphics
  trajectoryPG.beginDraw();
  trajectoryPG.background(30, 30, 40);
  
  // Set up lighting
  trajectoryPG.lights();
  
  // Position camera for the 3D view
  trajectoryPG.translate(trajectoryPG.width/2, trajectoryPG.height/2, 0);
  trajectoryPG.rotateX(cameraAngleX);
  trajectoryPG.rotateY(cameraAngleY);
  
  // Draw coordinate system
  trajectoryPG.stroke(255, 0, 0);
  trajectoryPG.line(0, 0, 0, 50, 0, 0);  // X-axis
  trajectoryPG.stroke(0, 255, 0);
  trajectoryPG.line(0, 0, 0, 0, 50, 0);  // Y-axis
  trajectoryPG.stroke(0, 0, 255);
  trajectoryPG.line(0, 0, 0, 0, 0, 50);  // Z-axis
  
  // Draw ground grid
  trajectoryPG.stroke(80, 80, 80);
  trajectoryPG.strokeWeight(1);
  for (int i = -5; i <= 5; i++) {
    float linePos = i * (groundSize/20);
    trajectoryPG.line(-groundSize/4, 0, linePos, groundSize/4, 0, linePos);
    trajectoryPG.line(linePos, 0, -groundSize/4, linePos, 0, groundSize/4);
  }
  
  // Draw trajectory
  trajectoryPG.noFill();
  trajectoryPG.strokeWeight(2);
  trajectoryPG.stroke(255, 255, 0);
  trajectoryPG.beginShape();
  for (PVector point : trajectoryPoints) {
    trajectoryPG.vertex(point.x * 5, -point.y * 5, point.z * 5);  // Scale and invert Y for display
  }
  trajectoryPG.endShape();
  
  // Draw current position
  trajectoryPG.pushMatrix();
    trajectoryPG.translate(position.x * 5, -position.y * 5, position.z * 5);
    
    // Orient rocket according to flight dynamics
    float heading = atan2(velocity.z, velocity.x) + PI/2;  // Heading angle in the XZ plane
    float elevation = 0;
    if (velocity.mag() > 0.1) {  // Only calculate if there's significant velocity
      elevation = -asin(velocity.y / velocity.mag());  // Elevation angle
    }
    
    trajectoryPG.rotateY(heading);
    trajectoryPG.rotateX(elevation);
    
    // Draw rocket
    trajectoryPG.scale(1.5);
    trajectoryPG.shape(rocket);
  trajectoryPG.popMatrix();
  
  // Draw text info
  trajectoryPG.camera();
  trajectoryPG.fill(255);
  trajectoryPG.textAlign(LEFT, TOP);
  trajectoryPG.textSize(10);
  trajectoryPG.text("ALT: " + nf(position.y, 1, 1) + "m", 10, 10);
  trajectoryPG.text("VEL: " + nf(velocity.mag(), 1, 1) + "m/s", 10, 25);
  trajectoryPG.text("STATE: " + stateLabels[state], 10, 40);
  
  trajectoryPG.endDraw();
  
  // Draw the PGraphics onto the main canvas
  image(trajectoryPG, trajectory3DPanelX, trajectory3DPanelY);
}