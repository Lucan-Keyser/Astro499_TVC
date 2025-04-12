/** sensors.cpp
* ===========================================================
* Project: Flight Controller
* Purpose: Sensor data acquisition and processing
* Version: 1.1
* ===========================================================
*/

#include "../include/sensors.h"

// Global variables

bool initializeSensors(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp) {
    bool success = true;
    
    // Initialize BNO085 IMU on Wire2
    Wire.begin();
    Wire.setClock(400000); // Set to 400kHz I2C speed


    // In your initializeSensors function
    if (!bno->begin_I2C(0x4A, &Wire)) {
        Serial.println("No BNO085 sensor detected!");
        success = false;
    } else {
        Serial.println("BNO085 detected successfully!");

    }
    
    Wire1.begin();
    Wire1.setClock(400000);
        
    // Initialize BMP390 using Wire1
    if (!bmp->begin_I2C(0x77, &Wire1)) {
        Serial.println("Trying alternative BMP390 address...");
    } else {
        Serial.println("BMP390 detected at address 0x77 on Wire1!");
    }
    
    // Configure BMP sensor if successfully initialized
    if (success) {
        //STILL NEED TO CHANGE/UPDATE 
        // bmp->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        // bmp->setPressureOversampling(BMP3_OVERSAMPLING_4X);
        // bmp->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3); // this is a infinite impulse response filter, this works by averaging the last 3 readings and smoothing the data
        // bmp->setOutputDataRate(BMP3_ODR_50_HZ); // Set output data rate to 50 Hz
        
        // Enable gyroscope reports on the BNO085
        if (!bno->enableReport(SH2_GYROSCOPE_CALIBRATED)) {
            Serial.println("Could not enable gyroscope reports");
            success = false;
        }
    }
    
    return success;
}

void updateIMU(Adafruit_BNO08x* bno, double* gyroRates, double* quants) {
    // Record start time
    unsigned long startTime = micros();
    
    // Get sensor data from the BNO085
    sh2_SensorValue_t sensorValue;
    
    if (bno->getSensorEvent(&sensorValue)) {
        // Only process if we have gyroscope data
        if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            // Apply coordinate transform (roll = -x, pitch = y, yaw = -z)
            double transformedRates[3] = {
                -sensorValue.un.gyroscope.x,  // Roll (negative x)
                sensorValue.un.gyroscope.y,   // Pitch (y)
                -sensorValue.un.gyroscope.z   // Yaw (negative z)
            };
            
            // Copy to gyroRates for use elsewhere in the code
            gyroRates[0] = transformedRates[0];
            gyroRates[1] = transformedRates[1];
            gyroRates[2] = transformedRates[2];
            
            // Calculate time delta
            static unsigned long lastTime = micros();
            double deltaTime = (micros() - lastTime) / 1000000.0; // Convert to seconds
            lastTime = micros();
            
            // Create quaternion from current orientation
            imu::Quaternion currentQuat(
                quants[0],  // w
                quants[1],  // x
                quants[2],  // y
                quants[3]   // z
            );
            
            // Calculate magnitude of angular velocity
            double magnitude = sqrt(
                transformedRates[0] * transformedRates[0] + 
                transformedRates[1] * transformedRates[1] + 
                transformedRates[2] * transformedRates[2]
            );
            
            if (magnitude > 0.0001) {  // Avoid division by zero
                // Create rotation quaternion
                double halfAngle = magnitude * deltaTime * 0.5;
                double sinHalfAngle = sin(halfAngle);
                
                imu::Quaternion rotationQuat(
                    cos(halfAngle),  // w
                    sinHalfAngle * transformedRates[0] / magnitude,  // x (roll)
                    sinHalfAngle * transformedRates[1] / magnitude,  // y (pitch)
                    sinHalfAngle * transformedRates[2] / magnitude   // z (yaw)
                );
                
                // Apply rotation: multiply current quaternion by rotation quaternion
                imu::Quaternion newQuat = currentQuat * rotationQuat;
                
                // Normalize the result
                newQuat.normalize();
                
                // Update quaternion array
                quants[0] = newQuat.w();
                quants[1] = newQuat.x();
                quants[2] = newQuat.y();
                quants[3] = newQuat.z();
            }
            
            // Convert quaternion to Euler angles (in radians) using atan2 approach
            double roll = atan2(2*(quants[0]*quants[1] + quants[2]*quants[3]), 
                               1 - 2*(quants[1]*quants[1] + quants[2]*quants[2]));
            
            double pitch = asin(2*(quants[0]*quants[2] - quants[3]*quants[1]));
            
            double yaw = atan2(2*(quants[0]*quants[3] + quants[1]*quants[2]), 
                              1 - 2*(quants[2]*quants[2] + quants[3]*quants[3]));
            
            // Convert to degrees for display
            double rollDeg = roll * RAD_TO_DEG;
            double pitchDeg = pitch * RAD_TO_DEG;
            double yawDeg = yaw * RAD_TO_DEG;
            
            // Print quaternion and Euler angles
            Serial.printf("Quaternion: w=%.4f, x=%.4f, y=%.4f, z=%.4f\n", 
                         quants[0], quants[1], quants[2], quants[3]);
            Serial.printf("Euler angles (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", 
                         rollDeg, pitchDeg, yawDeg);
        }
    }
    
    // Calculate execution time for performance monitoring
    unsigned long endTime = micros();
    double processingTime = (endTime - startTime) / 1000000.0;
    
    // Optional: track average processing time
    static double totalTime = 0.0;
    static int numSamples = 0;
    numSamples++;
    totalTime += processingTime;
    double avgTime = totalTime / numSamples;
    
    Serial.printf("IMU update time: %.6f s, Avg: %.6f s\n", processingTime, avgTime);
}

bool updateAltimeter(Adafruit_BMP3XX* bmp, double altData[3], float refPressure) {
    // Try multiple attempts
    for (int attempts = 0; attempts < 3; attempts++) {
        // Read data from the BMP sensor
        if (bmp->performReading()) {
            // Update altitude data array
            altData[0] = bmp->readAltitude(refPressure);  // Altitude based on reference pressure
            altData[1] = bmp->pressure;                   // Raw pressure
            altData[2] = bmp->temperature;                // Temperature
            return true;
        }
    }
    
    // If we get here, all attempts failed
    Serial.println("Failed to perform altimeter reading after multiple attempts");
    return false;
}

void returnData(sensors_event_t* event, double data[3]) {
    // Initialize with invalid values to detect problems
    data[0] = -1000;
    data[1] = -1000;
    data[2] = -1000;
    
    // Extract only gyroscope data
    if (event->type == SENSOR_TYPE_GYROSCOPE) {
        data[0] = event->gyro.x; // Gyroscope X-axis data in degrees/sec
        data[1] = event->gyro.y; // Gyroscope Y-axis data in degrees/sec
        data[2] = event->gyro.z; // Gyroscope Z-axis data in degrees/sec
    }
}

void resetSensors(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, double altData[3], float* refPressure, double gyroOffsets[3]) {
    // Reset both IMU and altimeter
    zeroIMU(bno, gyroOffsets);  // Reset IMU data
    zeroAltimeter(bmp, altData, refPressure);  // Reset altitude reference
}

//NO NEED CURRENTLY, BUT MAY BE NEEDED LATER


// void zeroIMU(Adafruit_BNO08x* bno, double gyroOffsets[3]) {
//     sh2_SensorValue_t sensorValue;
//     double tempOffsets[3] = {0.0, 0.0, 0.0};
//     int numSamples = 100; // Number of samples to average
//     unsigned long sampleDelay = 10; // Delay between samples in milliseconds
//     int validSamples = 0; // Counter for valid samples

//     Serial.println("Starting uncalibrated gyroscope calibration...");
    
//     // Collect multiple samples to calculate offsets
//     for (int i = 0; i < numSamples; i++) {
//         if (bno->getSensorEvent(&sensorValue)) {
//             if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
//                 tempOffsets[0] += sensorValue.un.gyroscopeUncal.x;
//                 tempOffsets[1] += sensorValue.un.gyroscopeUncal.y;
//                 tempOffsets[2] += sensorValue.un.gyroscopeUncal.z;
//                 validSamples++;
                
//                 if (i % 20 == 0) {
//                     Serial.print(".");
//                 }
//             }
//         } else {
//             Serial.println("Failed to retrieve gyroscope data during calibration!");
//         }
//         delay(sampleDelay); // Wait between samples
//     }
    
//     Serial.println();

//     // Calculate average offsets if valid samples were collected
//     if (validSamples > 0) {
//         gyroOffsets[0] = tempOffsets[0] / validSamples;
//         gyroOffsets[1] = tempOffsets[1] / validSamples;
//         gyroOffsets[2] = tempOffsets[2] / validSamples;
//         Serial.printf("IMU zeroed with %d samples\n", validSamples);
//         Serial.printf("Gyro Offsets: X=%.6f, Y=%.6f, Z=%.6f rad/s\n", 
//                      gyroOffsets[0], gyroOffsets[1], gyroOffsets[2]);
//     } else {
//         Serial.println("No valid IMU samples during calibration!");
//     }
// }


//still producing weird values, need to fix this, incorrest staritng alt
// void zeroAltimeter(Adafruit_BMP3XX* bmp, double altData[3], float* refPressure) {
//     // Try to get a good pressure reading
//     bool readingSuccess = false;
//     float currentPressure = 0;
    
//     // Try multiple times to get a reliable reading
//     for (int attempts = 0; attempts < 5; attempts++) {
//         if (bmp->performReading()) {
//             currentPressure = bmp->pressure / 100.0;  // Convert Pa to hPa
//             readingSuccess = true;
//             break;
//         }
//         delay(10);
//     }
    
//     if (readingSuccess) {
//         // Use current pressure as reference (this sets current location as zero altitude)
//         *refPressure = currentPressure;
        
//         Serial.print("Altitude reference set! Current pressure: ");
//         Serial.print(*refPressure);
//         Serial.println(" hPa");
        
//         // Take a new reading with this reference to verify
//         if (updateAltimeter(bmp, altData, *refPressure)) {
//             Serial.print("Calibrated altitude: ");
//             Serial.print(altData[0]);
//             Serial.println(" m (should be close to 0.0)");
//         }
//     } else {
//         Serial.println("Failed to get pressure reading for calibration");
//         // Don't change the reference pressure if we couldn't get a reading
//     }
// }