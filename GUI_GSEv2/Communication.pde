// Communication.pde
// Serial Communication and Data Handling

// -----------------------------------------
// Control Event Handler for UI
// -----------------------------------------
void controlEvent(ControlEvent theEvent) {
  String eventName = theEvent.getController().getName();

  if (eventName.equals("portDropdown")) {
    int selectedIndex = (int) theEvent.getValue();
    println("Selected port: " + availablePorts[selectedIndex]);
    connectToPort(selectedIndex);
    portDropdown.close();
  }
  else if (eventName.equals("Reset")) {
    println("Rocket reset");
    resetFlight();
    if (port != null) {
      port.write("RESET\n");
    }
  }
  else if (eventName.equals("calibration")) {
    println("Calibration initiated");
    if (port != null) {
      port.write("CALIBRATE\n");
    }
    // If in simulation mode, set state to calibration
    if (simulationMode) {
      state = 1;
      dt = 0;
    }
  }
  else if (eventName.equals("launchRocket")) {
    println("Rocket launched");
    startFlight();
    if (port != null) {
      port.write("LAUNCH\n");
    }
  }
  else if (eventName.equals("manualSeparation")) {
    println("Manual separation triggered");
    if (port != null) {
      port.write("SEPARATE\n");
    }
    parachuteDeployed = true;
  }
  else if (eventName.equals("abortMission")) {
    println("ABORT MISSION triggered");
    if (port != null) {
      port.write("ABORT\n");
    }
    // Set state to ABORT
    state = 6;
    parachuteDeployed = true;
    dt = 0;
  }
  else if (eventName.equals("disconnectPort")) {
    disconnectFromPort();
  }
  else if (eventName.equals("toggleSimulation")) {
    simulationMode = !simulationMode;
    if (simulationMode) {
      cp5.getController("toggleSimulation").setLabel("SIMULATION ON");
      cp5.getController("toggleSimulation").setColorBackground(color(60, 100, 60));
      println("Simulation mode activated");
    } else {
      cp5.getController("toggleSimulation").setLabel("SIMULATION OFF");
      cp5.getController("toggleSimulation").setColorBackground(color(100, 60, 60));
      println("Simulation mode deactivated");
    }
  }
}

// -----------------------------------------
// Serial Connection Management
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
  portDropdown.setLock(false);
  println("Disconnected.");
}

// -----------------------------------------
// Serial Event Handler
// -----------------------------------------
void serialEvent(Serial port) {
  try {
    String rawData = port.readStringUntil('\n');
    if (rawData != null) {
      rawData = rawData.trim();
      
      // Add to serial output box
      addToSerialOutput(rawData);
      
      String[] parts = split(rawData, ',');
      
      // Check for proper data format: 10 values for full telemetry with pyro, dt and state
      if (parts.length == 11) {
        // Parse values
        yaw = float(parts[0]);
        pitch = float(parts[1]);
        roll = float(parts[2]);
        altitude = float(parts[3]);
        yawServo = float(parts[4]);
        pitchServo = float(parts[5]);
        pyro1Continuity = float(parts[6]) > 0.5;
        pyro2Continuity = float(parts[7]) > 0.5;
        dt = float(parts[8]);
        state = int(float(parts[9]));
        dump = int(float(parts[10]));
        
        // Check state limits
        state = constrain(state, 0, 7);
        
        // If we're receiving real data and in flight, update the trajectory
        if (flightActive && !simulationMode) {
          // Convert from telemetry to 3D coordinates (simplified)
          float x = sin(radians(yaw)) * altitude * 0.1;
          float z = cos(radians(yaw)) * altitude * 0.1;
          position = new PVector(x, altitude, z);
          trajectoryPoints.add(position.copy());
          
          if (trajectoryPoints.size() > 1000) {
            trajectoryPoints.remove(0);
          }
        }
        
        // Update display
        updateGraphData();
        
        // Automatically detect parachute deployment based on state
        if (state == 5 || state == 6) { // DESCENT or ABORT
          parachuteDeployed = true;
        }
        
        // Automatically detect apogee based on state
        if (state >= 4) { // UNPOWERED ASCENT, DESCENT, or ABORT
          apogeeReached = true;
        }
        
        // Log to file
        String timestamp = new SimpleDateFormat("HH:mm:ss.SSS").format(new Date());
        String dataLine = String.join(",",
          timestamp,
          str(millis()),
          parts[0],   // Yaw
          parts[1],   // Pitch
          parts[2],   // Roll
          parts[3],   // Altitude
          parts[4],   // YawServo
          parts[5],   // PitchServo
          parts[6],   // Pyro1
          parts[7],   // Pyro2
          parts[8],   // DeltaTime
          parts[9]    // State
        );
        
        dataFile.println(dataLine);
        dataFile.flush(); // Ensure data is written immediately
      }
      // Previous format with 8 values, just in case
      else if (parts.length == 8) {
        yaw = float(parts[0]);
        pitch = float(parts[1]);
        roll = float(parts[2]);
        altitude = float(parts[3]);
        yawServo = float(parts[4]);
        pitchServo = float(parts[5]);
        pyro1Continuity = float(parts[6]) > 0.5;
        pyro2Continuity = float(parts[7]) > 0.5;
        
        updateGraphData();
        
        String timestamp = new SimpleDateFormat("HH:mm:ss.SSS").format(new Date());
        String dataLine = String.join(",",
          timestamp,
          str(millis()),
          parts[0], parts[1], parts[2], parts[3], parts[4], parts[5], parts[6], parts[7], "0", "0"
        );
        
        dataFile.println(dataLine);
        dataFile.flush();
      }
      // Even older format
      else if (parts.length == 6) {
        yaw = float(parts[0]);
        pitch = float(parts[1]);
        roll = float(parts[2]);
        altitude = float(parts[3]);
        yawServo = float(parts[4]);
        pitchServo = float(parts[5]);
        
        updateGraphData();
        
        String timestamp = new SimpleDateFormat("HH:mm:ss.SSS").format(new Date());
        String dataLine = String.join(",",
          timestamp,
          str(millis()),
          parts[0], parts[1], parts[2], parts[3], parts[4], parts[5], "0", "0", "0", "0"
        );
        
        dataFile.println(dataLine);
        dataFile.flush();
      }
    }
  } catch (Exception e) {
    println("Error: " + e);
  }
}