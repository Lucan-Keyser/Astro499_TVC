#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <RH_RF95.h>

// Pin Definitions
#define RFM95_CS 10
#define RFM95_INT 1
#define RFM95_RST 23
#define RF95_FREQ 915.0

// Initialize LoRa and IMU
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Quaternion Data
double quatData[4]; // [q_w, q_x, q_y, q_z]

// Function Prototypes
void readEulerAndConvertToQuaternion();
void transmitQuaternion();

// Setup Function
void setup() {
    Serial.begin(9600);

    // Initialize IMU
    if (!bno.begin()) {
        Serial.println("BNO055 initialization failed! Check wiring.");
        while (1);
    }

    // Initialize LoRa
    if (!rf95.init()) {
        Serial.println("LoRa initialization failed! Check wiring.");
        while (1);
    }
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("LoRa frequency setup failed!");
        while (1);
    }

    Serial.println("Setup complete.");
}

void loop() {
    readEulerAndConvertToQuaternion();
    transmitQuaternion();
    delay(10); // Send every 1 second
}

// Function to Read Euler Angles and Convert to Quaternion
void readEulerAndConvertToQuaternion() {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    // Read Euler angles
    double yaw = orientationData.orientation.x; // Degrees
    double pitch = orientationData.orientation.y; // Degrees
    double roll = orientationData.orientation.z; // Degrees

    // Convert degrees to radians
    yaw = yaw * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    roll = roll * M_PI / 180.0;

    // Compute Quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    quatData[0] = cr * cp * cy + sr * sp * sy; // q_w
    quatData[1] = sr * cp * cy - cr * sp * sy; // q_x
    quatData[2] = cr * sp * cy + sr * cp * sy; // q_y
    quatData[3] = cr * cp * sy - sr * sp * cy; // q_z

    // Debug Output
    Serial.print("Quaternion: ");
    Serial.print("w="); Serial.print(quatData[0], 4);
    Serial.print(", x="); Serial.print(quatData[1], 4);
    Serial.print(", y="); Serial.print(quatData[2], 4);
    Serial.print(", z="); Serial.println(quatData[3], 4);
}

// Function to Transmit Quaternion Data
void transmitQuaternion() {
    char message[64];
    snprintf(message, sizeof(message), "w:%.4f,x:%.4f,y:%.4f,z:%.4f",
             quatData[0], quatData[1], quatData[2], quatData[3]);

    Serial.println("Sending: " + String(message));

    if (rf95.send((uint8_t *)message, strlen(message))) {
        if (rf95.waitPacketSent()) {
            Serial.println("Message sent successfully.");
        } else {
            Serial.println("Error: Failed to confirm message transmission.");
        }
    } else {
        Serial.println("Error: Failed to send message.");
    }
}
