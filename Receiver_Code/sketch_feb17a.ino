#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 22
#define RFM95_INT 1
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

#pragma pack(push, 1)
struct TelemetryData {
  float yaw;
  float pitch;
  float roll;
  float altitude;
  float yawServo;
  float pitchServo;
};
#pragma pack(pop)

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection

  // LoRa initialization
  pinMode(RFM95_RST, OUTPUT);
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

  Serial.println("GSE_READY");
}

void loop() {
  // Receive and process telemetry
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      if (len == sizeof(TelemetryData)) {
        TelemetryData data;
        memcpy(&data, buf, len);
        
        // Convert to CSV string for Processing
        char output[60];
        snprintf(output, sizeof(output), "%.2f,%.2f,%.2f,%.1f,%.2f,%.2f", 
                 data.yaw, data.pitch, data.roll, data.altitude, data.yawServo, data.pitchServo);
        
        Serial.println(output); // Send to Processing
      }
    }
  }

  // Forward commands to rocket
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      rf95.send((uint8_t*)command.c_str(), command.length());
      rf95.waitPacketSent();
    }
  }
}