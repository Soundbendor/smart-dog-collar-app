#include "../inc/sensors.h"

// Sets up the IMU and finds the xsample rate
bool Sensors::setupIMU(tflite::ErrorReporter *error_reporter) 
{
    // Try to start up the IMU
    if (!IMU.begin()) 
    {
        error_reporter->Report("Failed to initialize IMU");
        return 0;
    }

    // Make sure we are pulling measurements into a FIFO.
    // If you see an error on this line, make sure you have at least v1.1.0 of the
    // Arduino_LSM9DS1 library installed.
    IMU.setContinuousMode();

    // Find the sample rate of our sensors
    acceleration_sample_rate = IMU.accelerationSampleRate();
    gyroscope_sample_rate = IMU.gyroscopeSampleRate();

    #ifdef SMART_DOG_COLLAR_DEBUG
    float rate_frac;
    float rate_int;
    rate_frac = modf(acceleration_sample_rate, &rate_int);
    TF_LITE_REPORT_ERROR(error_reporter, "Acceleration sample rate %d.%d Hz",
                        static_cast<int32_t>(rate_int),
                        static_cast<int32_t>(rate_frac * 100));
    rate_frac = modf(gyroscope_sample_rate, &rate_int);
    TF_LITE_REPORT_ERROR(error_reporter, "Gyroscope sample rate %d.%d Hz",
                        static_cast<int32_t>(rate_int),
                        static_cast<int32_t>(rate_frac * 100));

    Serial.println("Data from setupIMU()")
    Serial.print(gyroscope_sample_rate);
    Serial.print('\t');
    Serial.print(acceleration_sample_rate);
    #endif

    return 1;
}

// Reads in samples from the accelerometer and gyroscope if data is available
// and stores it in a FIFO buffer
void Sensors::readAccelerometerAndGyroscope(tflite::ErrorReporter *error_reporter) 
{ 
    // Get new samples
    while (IMU.accelerationAvailable()) 
    {
        const int gyroscope_index = (gyroscope_data_index % GYROSCOPE_DATA_LENGTH);
        gyroscope_data_index += 3;
        float* current_gyroscope_data = &gyroscope_data[gyroscope_index];

        // Read gyroscope sample
        if (!IMU.readGyroscope(current_gyroscope_data[0],
                            current_gyroscope_data[1],
                            current_gyroscope_data[2])) 
        {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to read gyroscope data");
        break;
        }

        const int acceleration_index = (acceleration_data_index % ACCELERATION_DATA_LENGTH);
        acceleration_data_index += 3;
        float* current_acceleration_data = &acceleration_data[acceleration_index];

        // Read acceleration sample
        if (!IMU.readAcceleration(current_acceleration_data[0],
                                current_acceleration_data[1],
                                current_acceleration_data[2])) 
        {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to read acceleration data");
        break;
        }

        #ifdef SMART_DOG_COLLAR_DEBUG
        // Check what is being read from the IMU
        Serial.println("Data from readAccelerometerAndGyroscope()");
        Serial.print(current_acceleration_data[0]);
        Serial.print('\t');
        Serial.print(current_acceleration_data[1]);
        Serial.print('\t');
        Serial.print(current_acceleration_data[2]);
        Serial.print('\t');
        Serial.print(current_gyroscope_data[0]);
        Serial.print('\t');
        Serial.print(current_gyroscope_data[1]);
        Serial.print('\t');
        Serial.println(current_gyroscope_data[2]);
        #endif
    }
}