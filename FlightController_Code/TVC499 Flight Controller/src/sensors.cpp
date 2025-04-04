/** sensors.cpp
* ===========================================================
* Name: Flight Controller Sensors Implementation
* Section: TVC499
* Project: Flight Controller
* Purpose: Sensor data acquisition and processing
* ===========================================================
*/

#include "../include/sensors.h"

bool initializeSensors(Adafruit_BNO055* bno, Adafruit_BMP3XX* bmp) {
    bool success = true;
    
    // Initialize BNO055 IMU
    if (!bno->begin()) {
        Serial.println("No BNO055 detected!");
        success = false;
    }
    
    // Initialize BMP390 altimeter on Wire1
    Wire1.begin();
    if (!bmp->begin_I2C(0x77, &Wire1)) {
        Serial.println("No BMP3 sensor detected!");
        success = false;
    }
    
    // Configure BMP sensor if successfully initialized
    if (success) {
        bmp->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp->setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3); // this is a infinite impulse response filter, this works by averaging the last 3 readings and smoothing the data
        bmp->setOutputDataRate(BMP3_ODR_50_HZ); // Set output data rate to 50 Hz
    }
    
    return success;
}

void updateIMU(Adafruit_BNO055* bno, double gyroRates[3], double gyroOffsets[3]) {
    sensors_event_t angVelocityData;


    // Record start time
    unsigned long startTime = micros();

    // Get gyroscope data with offsets
    if (bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
        // Subtract offsets to get calibrated gyroscope rates
        gyroRates[0] = angVelocityData.gyro.x - gyroOffsets[0];
        gyroRates[1] = angVelocityData.gyro.y - gyroOffsets[1];
        gyroRates[2] = angVelocityData.gyro.z - gyroOffsets[2];
    } else {
        Serial.println("Failed to retrieve gyroscope data!");
    }

    // //get gyro data with no offsets
    // if (bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
    //     gyroRates[0] = angVelocityData.gyro.x;
    //     gyroRates[1] = angVelocityData.gyro.y;
    //     gyroRates[2] = angVelocityData.gyro.z;
    // } else {
    //     Serial.println("Failed to retrieve gyroscope data!");
    // }

    // Calculate time delta

    unsigned long endTime = micros();
    double dt = (endTime - startTime) / 1000000.0; // Convert microseconds to seconds
    // Print time delta
    printf("dt_updateIMU: %f\n", dt); // Print time delta


    // Serial.print("Gyro Rates (calibrated): ");
    // Serial.print(gyroRates[0]); Serial.print(", ");
    // Serial.print(gyroRates[1]); Serial.print(", ");
    // Serial.println(gyroRates[2]);
}

bool updateAltimeter(Adafruit_BMP3XX* bmp, double altData[3], float refPressure) {


    // Record start time
    unsigned long startTime = micros();


    // Read data from the BMP sensor
    if (!bmp->performReading()) {
        Serial.println("Failed to perform altimeter reading");
        return false;
    }
    
    // Update altitude data array
    altData[0] = bmp->readAltitude(refPressure);  // Altitude based on reference pressure
    altData[1] = bmp->pressure;                   // Raw pressure
    altData[2] = bmp->temperature;                // Temperature

    // Calculate time delta of function execution
    unsigned long endTime = micros();
    double dt = (endTime - startTime) / 1000000.0; // Convert microseconds to seconds
    printf("dt_updateAltimeter: %f\n", dt); // Print time delta

    return true;
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

void resetSensors(Adafruit_BNO055* bno, Adafruit_BMP3XX* bmp, double altData[3], float* refPressure, double gyroOffsets[3]) {
    // Reset both IMU and altimeter
    zeroIMU(bno, gyroOffsets);  // Reset IMU data
    zeroAltimeter(bmp, altData, refPressure);  // Reset altitude reference
}

void zeroIMU(Adafruit_BNO055* bno, double gyroOffsets[3]) {
    sensors_event_t angVelocityData;
    double tempOffsets[3] = {0.0, 0.0, 0.0};
    int numSamples = 10; // Number of samples to average
    unsigned long sampleDelay = 5; // Delay between samples in milliseconds

    // Collect multiple samples to calculate offsets
    for (int i = 0; i < numSamples; i++) {
        if (bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
            tempOffsets[0] += angVelocityData.gyro.x;
            tempOffsets[1] += angVelocityData.gyro.y;
            tempOffsets[2] += angVelocityData.gyro.z;
        } else {
            Serial.println("Failed to retrieve gyroscope data during calibration!");
            return;
        }
        delay(sampleDelay); // Wait between samples
    }

    // Calculate average offsets
    gyroOffsets[0] = tempOffsets[0] / numSamples;
    gyroOffsets[1] = tempOffsets[1] / numSamples;
    gyroOffsets[2] = tempOffsets[2] / numSamples;

    // Serial.println("IMU gyroscope calibrated. Offsets:");
    // Serial.print("X Offset: "); Serial.println(gyroOffsets[0]);
    // Serial.print("Y Offset: "); Serial.println(gyroOffsets[1]);
    // Serial.print("Z Offset: "); Serial.println(gyroOffsets[2]);
}



void zeroAltimeter(Adafruit_BMP3XX* bmp, double altData[3], float* refPressure) {
    // Read current pressure and update reference pressure
    if (updateAltimeter(bmp, altData, *refPressure)) {
        *refPressure = altData[1] / 100.0;  // Convert Pa to hPa
    }
}