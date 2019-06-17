//
// Created by Florian VanKampen on 2019-06-17.
//

#include "Lidar.h"

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
        lidarFront = new Lidar();
        lidarTop = new Lidar();
        lidarLeft = new Lidar();
        lidarRight = new Lidar();
        lidarBottom = new Lidar();
    };