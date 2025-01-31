#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 22
#define RFM95_INT 1

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  Serial.begin(9600);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1);
  }
  Serial.println("LoRa init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Set frequency failed");
    while (1);
  }
  rf95.setTxPower(23, false);
}

void loop() {
  // Receive data from Flight Controller and forward to Processing
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.println((char *)buf);
    }
  }

  // Forward commands from Processing to Flight Controller via LoRa
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    Serial.print("GSE Sending via LoRa: ");
    Serial.println(command);

    rf95.send((uint8_t *)command.c_str(), command.length());
    rf95.waitPacketSent();
  }

}