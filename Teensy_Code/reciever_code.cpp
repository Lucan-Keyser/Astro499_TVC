#include <SPI.h>
#include <RH_RF95.h>

// Define pins for LoRa module
#define RFM95_CS    10
#define RFM95_INT   1
#define RFM95_RST   23
#define RF95_FREQ   915.0 // Frequency for your region

// LoRa module instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
    Serial.begin(9600); // Start the serial communication
    while (!Serial);    // Wait for serial connection

    // Initialize LoRa module
    if (!rf95.init()) {
        Serial.println("LoRa init failed!");
        while (1);
    }

    // Set frequency
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("Set frequency failed!");
        while (1);
    }

    // Configure LoRa parameters

    Serial.println("LoRa receiver ready.");
}

void loop() {
    // Check if there's a LoRa packet available
    if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; // Buffer to hold the incoming packet
        uint8_t len = sizeof(buf);

        // Receive the packet
        if (rf95.recv(buf, &len)) {
            buf[len] = '\0'; // Null-terminate the string
            Serial.println((char*)buf); // Forward the received packet to the serial monitor

            // Debug information
            Serial.print("RSSI: ");
            Serial.println(rf95.lastRssi());
        } else {
            Serial.println("Receive failed.");
        }
    }
}
