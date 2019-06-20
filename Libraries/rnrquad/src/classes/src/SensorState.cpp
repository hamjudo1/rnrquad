//
// Created by Florian VanKampen on 2019-06-17.
//

#include "Lidar.h"
#include "SensorState.h"

SensorState::SensorState()
    {
        batteryVoltage = 0;

        // Flow sensor
        flowX = 0;
        flowY = 0;
        flowQ = 0;

        // Gyro sensor
        gyroX = 0;
        gyroY = 0;
        gyroZ = 0;

        // Accelerometer
        accelX = 0;
        accelY = 0;
        accelZ = 0;

        // Lidars
        lidarFront = Lidar();
        lidarTop = Lidar();
        lidarLeft = Lidar();
        lidarRight = Lidar();
        lidarBottom = Lidar();
    };