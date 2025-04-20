// Simulation.pde
// Physics Simulation and Flight Dynamics

// -----------------------------------------
// Simulation Methods
// -----------------------------------------
void updateSimulation() {
  if (!flightActive) return;
  
  // Calculate time since launch
  float elapsedTime;
  if (simulationMode) {
    currentTime += timeStep;
    elapsedTime = currentTime;
  } else {
    elapsedTime = (millis() - flightStartTime) / 1000.0;
  }
  
  // Get thrust from thrust curve
  float thrust = interpolateThrust(elapsedTime);
  
  // Calculate forces
  // 1. Thrust force (along rocket's local y-axis)
  PVector thrustForce = new PVector(0, thrust, 0);
  
  // Apply rotation to thrust vector based on rocket orientation
  float yawRad = radians(yaw);
  float pitchRad = radians(pitch);
  float rollRad = radians(roll);
  
  // Convert orientation angles to a direction vector
  // (This is a simplification - for more accuracy, you'd use a rotation matrix)
  PVector thrustDirection = new PVector(
    sin(pitchRad) * cos(yawRad),
    cos(pitchRad),
    sin(pitchRad) * sin(yawRad)
  );
  thrustDirection.normalize();
  
  // Scale by thrust magnitude
  thrustDirection.mult(thrust);
  
  // 2. Gravity force (always in global -y direction)
  PVector gravityForce = new PVector(0, -gravity * rocketMass, 0);
  
  // 3. Drag force (opposite to velocity)
  PVector dragForce = velocity.copy();
  if (dragForce.mag() > 0) {
    dragForce.normalize();
    float dragMagnitude = 0.5 * airDensity * velocity.magSq() * dragCoefficient * rocketArea;
    dragForce.mult(-dragMagnitude);
  }
  
  // Sum forces
  PVector totalForce = new PVector();
  totalForce.add(thrustDirection);
  totalForce.add(gravityForce);
  totalForce.add(dragForce);
  
  // Calculate acceleration (F = ma)
  acceleration = PVector.div(totalForce, rocketMass);
  
  // Update velocity (v = v0 + at)
  velocity.add(PVector.mult(acceleration, timeStep));
  
  // Update position (p = p0 + vt)
  position.add(PVector.mult(velocity, timeStep));
  
  // Ensure rocket doesn't go below ground
  if (position.y < 0) {
    position.y = 0;
    velocity.y = 0;
    
    // If we hit the ground, stop the flight
    if (velocity.mag() < 0.1 && elapsedTime > 5) {
      flightActive = false;
    }
  }
  
  // Calculate angular moments for pitch and yaw based on servo positions
  // This is a simplified model - in reality, these would depend on aerodynamic forces
  // Yaw and pitch servos influence the control surface deflection, which creates moments
  float yawMoment = yawServo * 0.01 * thrust * momentArm;  // Simplified
  float pitchMoment = pitchServo * 0.01 * thrust * momentArm;  // Simplified
  
  // Angular acceleration = Moment / Moment of Inertia
  float yawAccel = yawMoment / momentOfInertia;
  float pitchAccel = pitchMoment / momentOfInertia;
  
  // Update angular velocities
  float yawVelocity = yawAccel * timeStep;
  float pitchVelocity = pitchAccel * timeStep;
  
  // Update angles
  yaw += yawVelocity * timeStep;
  pitch += pitchVelocity * timeStep;
  
  // Add position to trajectory
  trajectoryPoints.add(position.copy());
  if (trajectoryPoints.size() > 1000) {
    trajectoryPoints.remove(0);  // Remove oldest point to prevent excessive memory use
  }
  
  // Update altitude for displays
  altitude = position.y;
  
  // Update graph data
  updateGraphData();
  
  // Update flight states based on simulation
  updateFlightState(elapsedTime, thrust);
}

// -----------------------------------------
// Update Flight State
// -----------------------------------------
void updateFlightState(float elapsedTime, float thrust) {
  // State transitions for simulation mode
  if (simulationMode) {
    if (state == 0 && flightActive) {
      // Move from PAD IDLE to CALIBRATION
      state = 1;
      dt = 0;
    } 
    else if (state == 1 && elapsedTime > 2) {
      // Move from CALIBRATION to COUNTDOWN
      state = 2;
      dt = 0;
    }
    else if (state == 2 && elapsedTime > 4) {
      // Move from COUNTDOWN to ASCENT
      state = 3;
      dt = 0;
    }
    else if (state == 3 && thrust < 1.0) {
      // Motor has burned out, move to UNPOWERED ASCENT
      state = 4;
      dt = 0;
    }
    else if (state == 4 && velocity.y < 0) {
      // We've reached apogee, move to DESCENT
      state = 5;
      parachuteDeployed = true;
      dragCoefficient = 2.0; // Increase drag for parachute
      dt = 0;
      
      // Also set apogee reached
      apogeeReached = true;
    }
    
    // Update time in current state
    dt += timeStep * 1000; // Convert to milliseconds
  }
}

// -----------------------------------------
// Thrust Interpolation
// -----------------------------------------
float interpolateThrust(float time) {
  // If time is beyond the thrust curve, return 0
  if (time >= thrustTimes[thrustTimes.length - 1]) {
    return 0;
  }
  
  // Find the segment in the thrust curve
  int i = 0;
  while (i < thrustTimes.length - 1 && time > thrustTimes[i + 1]) {
    i++;
  }
  
  // Interpolate within the segment
  float t = (time - thrustTimes[i]) / (thrustTimes[i + 1] - thrustTimes[i]);
  return lerp(thrustValues[i], thrustValues[i + 1], t);
}

// -----------------------------------------
// Flight Control Functions
// -----------------------------------------
void resetFlight() {
  // Reset flight parameters
  flightActive = false;
  apogeeReached = false;
  parachuteDeployed = false;
  currentTime = 0;
  
  // Reset position and motion
  position = new PVector(0, 0, 0);
  velocity = new PVector(0, 0, 0);
  acceleration = new PVector(0, 0, 0);
  
  // Reset drag coefficient (will increase when parachute deploys)
  dragCoefficient = 0.5;
  
  // Reset state
  state = 0; // PAD IDLE
  dt = 0;
  
  // Clear trajectory
  trajectoryPoints.clear();
  trajectoryPoints.add(new PVector(0, 0, 0));
  
  println("Flight reset");
}

void startFlight() {
  resetFlight();
  flightActive = true;
  flightStartTime = millis();
  println("Flight started");
}