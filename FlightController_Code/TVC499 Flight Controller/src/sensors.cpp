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
    unsigned long mathtime = micros(); // Initialize current time
     
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

    // Calculate time delta
    unsigned long endTime = micros();
    double dt = (endTime - startTime) / 1000000.0; // Convert microseconds to seconds
    //Serial.printf("dt_updateIMU: %f\n", dt); // Print time delta
        


    // find magnitude of the transformed rates (sqrt(x^2 + y^2 + z^2)), units are rads/sec
    // this is used to normalize the quaternion rotation
    double magnitude = sqrt(gyroRates[0]*gyroRates[0] + gyroRates[1]*gyroRates[1] + gyroRates[2]*gyroRates[2]);

    //identity quaternion for initialization, why? because we are going to multiply it by the rotation quaternion
    // this is the quaternion that represents no rotation, so we start with it and then apply the rotation quaternion to it
    double quant_w = quants[0];
    double quant_x = quants[1];
    double quant_y = quants[2];
    double quant_z = quants[3];

    if (magnitude > 0.0001) { //avoid division by zero

        // Create rotation quaternion
        // uses the formula q = (cos(theta/2), sin(theta/2)*u_x, sin(theta/2)*u_y, sin(theta/2)*u_z) 
        //we will get unit vector u in next part, this is just the half angle part
        double halfAngle = magnitude * dt * 0.5; //go from radians/sec to radians and then divide by 2 for the half angle 
        double sinHalfAngle = sin(halfAngle); // sin(theta/2) used to find the x, y, z components of the quaternion (the vector part)
        double cosHalfAngle = cos(halfAngle); // cos(theta/2) used to find the w component of the quaternion (the scalar part)
        
        //normalize the rotation quaternion, getting our unit vector
        double rotation_w = cosHalfAngle; // w component of the quaternion (scalar part) simple cos(theta/2)
        double rotation_x = sinHalfAngle * gyroRates[0] / magnitude; // x component of the quaternion (vector part) sin(theta/2)*u_x
        // u_x is the x component of the transformed rates, we divide by the magnitude to normalize it (make it unit vector)
        // the units on this look like rads/sec (units for gyro rates) / rads/sec (units for magnitude) = 1 
        double rotation_y = sinHalfAngle * gyroRates[1] / magnitude; // y component of the quaternion (vector part) sin(theta/2)*u_y
        double rotation_z = sinHalfAngle * gyroRates[2] / magnitude; // z component of the quaternion (vector part) sin(theta/2)*u_z

        // A quaternion q = (w, x, y, z) consists of:
        //  A scalar (real) part w
        //  Vector (imaginary) parts x, y, z corresponding to bases i, j, k
        
        // Multiplication formula comes from the algebra of these components:
        //      i² = j² = k² = -1 (squares of imaginary units are negative)
        
        //      i*j = k, j*k = i, k*i = j (cyclic products are positive)  
        //   think of cross product of x vector and y vector = z (calc 3)
        //   these are called "cyclic products" because they follow a clockwise
        //   circular pattern when i,j,k are arranged in a circle. Multiplying
        //   two adjacent elements clockwise gives the third element with a positive sign.     
        
        //      j*i = -k, k*j = -i, i*k = -j (anti-cyclic products are negative) 
        //   think of cross product of y vector and x vector = -z (calc 3)
        //   these are called "anti-cyclic products" because they go against the
        //   clockwise circular pattern. Multiplying two adjacent elements
        //   counterclockwise gives the third element with a negative sign.
        
        //This cyclic/anti-cyclic pattern is what makes quaternion multiplication
        // non-commutative (q1*q2 ≠ q2*q1), which correctly models how
        // 3D rotations work (rotating around X then Y ≠ Y then X).

        // We multiply (not add) quaternions because rotation is non-linear.
        // The specific pattern of positive and negative signs in the multiplication
        // formula directly comes from these cyclic and anti-cyclic relationships.
        double new_w = quant_w * rotation_w - quant_x * rotation_x - quant_y * rotation_y - quant_z * rotation_z;
        double new_x = quant_w * rotation_x + quant_x * rotation_w + quant_y * rotation_z - quant_z * rotation_y;
        double new_y = quant_w * rotation_y - quant_x * rotation_z + quant_y * rotation_w + quant_z * rotation_x;
        double new_z = quant_w * rotation_z + quant_x * rotation_y - quant_y * rotation_x + quant_z * rotation_w;
        
        //update orientation with the new values
        quant_w = new_w;
        quant_x = new_x;
        quant_y = new_y;
        quant_z = new_z;
        
        // Normalize the quaternion (ensure unit length)
        double orient_magnitude = sqrt(quant_w * quant_w + quant_x * quant_x + quant_y * quant_y + quant_z * quant_z);
        
        if (orient_magnitude > 0.0001) {
            quant_w /= orient_magnitude;
            quant_x /= orient_magnitude;
            quant_y /= orient_magnitude;
            quant_z /= orient_magnitude;
        }
    }

    quants[0] = quant_w;
    quants[1] = quant_x;
    quants[2] = quant_y;
    quants[3] = quant_z;

    unsigned long endMathTime = micros();
    double mathTime = (endMathTime - mathtime) / 1000000.0; // Convert microseconds to seconds

    //print avg math time delta
    static double avgTime = 0.0;
    static int numSamples = 0;
    static double totalTime = 0.0;
    numSamples++;
    totalTime += mathTime;
    avgTime = totalTime / numSamples;
    Serial.printf("Avg dt: %f\n", avgTime); // Print avg time delta
    Serial5.printf("Avg dt: %f\n", avgTime); // Print avg time delta
    
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