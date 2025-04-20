#include <Arduino.h>
#include <RH_RF95.h>

// Radio configuration
#define RFM95_CS 10
#define RFM95_RST 22
#define RFM95_INT 1
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Telemetry data structure with pyro continuity as floats
#pragma pack(push, 1)
struct TelemetryData {
  float roll;
  float pitch;
  float yaw;
  float altitude;
  float pitchServo;
  float yawServo;
  float pyro1Continuity;  
  float pyro2Continuity;  
  float dt;
  float state;
  float dump;
};
#pragma pack(pop)

void setup() {
  Serial.begin(9600);  // Match Processing baud rate
  
  // Initialize LoRa
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1);
  }

  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);
  rf95.setSpreadingFactor(9);
  rf95.setSignalBandwidth(500000);
  rf95.setCodingRate4(5);
  Serial.println("GSE ready");
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len) && len >= sizeof(TelemetryData)) {
      Serial.print("Received packet size (bytes): ");
      Serial.println(len);  // This prints the received message length

      TelemetryData data;
      memcpy(&data, buf, sizeof(TelemetryData));
      
      // Output in format expected by Processing:

      char output[128];
      sprintf(output, "%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.0f,%.0f,%.4f,%.0f,%.0f", 
              data.roll, data.pitch, data.yaw, data.altitude, 
              data.yawServo, data.pitchServo, 
              data.pyro1Continuity, data.pyro2Continuity,
              data.dt,
              data.state,
              data.dump);
      Serial.println(output);
    }
  }

  // Forward commands to rocket
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.println(command);
      rf95.send((uint8_t*)command.c_str(), command.length());
      rf95.waitPacketSent();
    }
  }
  
  delay(5);
}