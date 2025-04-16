/** sensors.cpp
* ===========================================================
* Project: Flight Controller
* Purpose: Sensor data acquisition and processing
* Version: 1.1
* ===========================================================
*/

#include "../include/sensors.h"
#include "../include/config.h"


double gyroOffsets[3] = {0.0, 0.0, 0.0}; // Gyro offsets for calibration


bool initializeSensors(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, double& refPressure) {
    double tempAltData[3] = {0.0, 0.0, 0.0}; // Altitude data [altitude, pressure, temperature], just within this function
    double sumPressure = 0.0; // Sum of pressure readings for calibration
    bool success = true;
    
    // Initialize BNO085 IMU on Wire2
    Wire.begin();
    Wire.setClock(400000); // Set to  400kHz I2C speed


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
        success = false;
    } else {
        Serial.println("BMP390 detected at address 0x77 on Wire1!");
    }

    for (int i = 0; i < ALTIMETER_CALIBRATION_COUNT; i++) {
        // Read the raw pressure and temperature values
        updateAltimeter(bmp, tempAltData, refPressure);
        sumPressure += tempAltData[1]; // Add the pressure reading to the sum
        delay(ALTIMETER_CALIBRATION_DELAY);
    }

    refPressure = sumPressure / (ALTIMETER_CALIBRATION_COUNT * 100); // Calculate the average pressure for calibration and convert to HPA
    
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
        if (!bno->enableReport(SH2_ACCELEROMETER)){
            Serial.println("Could not enable accelerometer reports");
            success = false;
        }
    }
    
    return success;
}


void initializeQuaternions(Adafruit_BNO08x* bno,  double* quaternions, double* accelerometer) {
    //Initialize quaternion
    sh2_SensorValue_t sensorValue;

    int n = 100;
    double xSum = 0;
    double ySum = 0;
    double zSum = 0;

    for (int i = 0 ; i < n; i++) {
        if (bno->getSensorEvent(&sensorValue)) {
            // Serial.println("lmao");
            //get accelerometer data
            if (sensorValue.sensorId == SH2_ACCELEROMETER) { //coordinate transform
                accelerometer[0] = sensorValue.un.accelerometer.x;
                accelerometer[1] = -sensorValue.un.accelerometer.y;
                accelerometer[2] = sensorValue.un.accelerometer.z;
            }
        }

        xSum += accelerometer[0];
        ySum += accelerometer[1];
        zSum += accelerometer[2];

        delay(10);
    }
    double xAvg = xSum/n;
    double yAvg = ySum/n;
    double zAvg = zSum/n;

    double pitch = atan(zAvg/xAvg); //pitch angle on pad, tan^-1(z/x)
    double yaw = -atan(yAvg/xAvg);//yaw angle on pad, -tan^-1(y/x)
    double roll = 0;

    // double pitch = RAD_TO_DEG*atan(accelerometer[2]/accelerometer[0]);
    // double yaw = RAD_TO_DEG*atan(accelerometer[1]/accelerometer[0]);//pitch angle on pad, tan^-1(y/x)
    // double roll = 0;

    // Compute trigonometric functions
    double cx = cos(roll * 0.5);
    double sx = sin(roll * 0.5);
    double cy = cos(pitch * 0.5);
    double sy = sin(pitch * 0.5);
    double cz = cos(yaw * 0.5);
    double sz = sin(yaw * 0.5);
    // Compute quaternion components
    quaternions[0] = cx * cy * cz + sx * sy * sz;
    quaternions[1]= sx * cy * cz - cx * sy * sz;
    quaternions[2] = cx * sy * cz + sx * cy * sz;
    quaternions[3] = cx * cy * sz - sx * sy * cz;

    // Serial.printf("pitch: %.6f, yaw: %.6f\n", pitch, yaw);

    
}

void updateIMU(Adafruit_BNO08x* bno, double* gyroRates, double* quaternions, double* eulerAngles, double* accelerometer, double dt) {
    // Record start time
    sh2_SensorValue_t sensorValue;
    
    if (bno->getSensorEvent(&sensorValue)) {
            // Process calibrated gyroscope data
            if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
                // Calibrated gyroscope data in rad/s
                // Apply coordinate transform (roll = -x, pitch = y, yaw = -z) based on imu to rocket frame 
                //NASA standard (x = roll, y = pitch, z = yaw, +x = up out of nose cone, +y = right, +z = toward viewer)
                gyroRates[0] = -sensorValue.un.gyroscope.x; // Roll (x-axis) in rad/s
                gyroRates[1] = sensorValue.un.gyroscope.y; // Pitch (y-axis) in rad/s
                gyroRates[2] = -sensorValue.un.gyroscope.z; // Yaw (z-axis) in rad/s
                
            }
        }


    if (bno->getSensorEvent(&sensorValue)) {
        // Serial.println("lmao");
        //get accelerometer data
        if (sensorValue.sensorId == SH2_ACCELEROMETER) { //coordinate transform
            accelerometer[0] = sensorValue.un.accelerometer.x;
            accelerometer[1] = -sensorValue.un.accelerometer.y;
            accelerometer[2] = sensorValue.un.accelerometer.z;
            }
        }
    // Calculate time delta
    
    double q0 = quaternions[0], q1 = quaternions[1], q2 = quaternions[2], q3 = quaternions[3];
    double w1 = gyroRates[0], w2 = gyroRates[1], w3 = gyroRates[2];

    double dq0 = 0.5 * (-w1 * q1 - w2 * q2 - w3 * q3);
    double dq1 = 0.5 * ( w1 * q0 + w3 * q2 - w2 * q3);
    double dq2 = 0.5 * ( w2 * q0 - w3 * q1 + w1 * q3);
    double dq3 = 0.5 * ( w3 * q0 + w2 * q1 - w1 * q2);

    double q0_new = q0 + dq0 * dt;
    double q1_new = q1 + dq1 * dt;
    double q2_new = q2 + dq2 * dt;
    double q3_new = q3 + dq3 * dt;

    // Normalize quaternion
    double norm = sqrt(q0_new * q0_new + q1_new * q1_new + q2_new * q2_new + q3_new * q3_new);
    if (norm > 0) {
        quaternions[0] = q0_new / norm;
        quaternions[1] = q1_new / norm;
        quaternions[2] = q2_new / norm;
        quaternions[3] = q3_new / norm;
    } else {
        Serial.println("Quaternion normalization failed!");
    }

    //roll
    eulerAngles[0] = RAD_TO_DEG * (atan2(2 * (quaternions[0] * quaternions[1] + quaternions[2] * quaternions[3]),
                      1 - 2 * (quaternions[1] * quaternions[1] + quaternions[2] * quaternions[2])));
    //pitch
    eulerAngles[1] = RAD_TO_DEG * asin(2 * (quaternions[0] * quaternions[2] - quaternions[3] * quaternions[1]));
    //yaw
    eulerAngles[2] = RAD_TO_DEG * (atan2(2 * (quaternions[0] * quaternions[3] + quaternions[1] * quaternions[2]),
                     1 - 2 * (quaternions[2] * quaternions[2] + quaternions[3] * quaternions[3]))); 


}

bool updateAltimeter(Adafruit_BMP3XX* bmp, double altData[3], double& refPressure) {
    // Read data from the BMP sensor
    if (bmp->performReading()) {
            // Update altitude data array
            altData[0] = bmp->readAltitude(refPressure);  // Altitude based on reference pressure
            altData[1] = bmp->pressure;                   // Raw pressure
            altData[2] = bmp->temperature;                // Temperature
            return true;
    } else {
        Serial.println("Failed to read BMP390 sensor data!");
        return false;
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