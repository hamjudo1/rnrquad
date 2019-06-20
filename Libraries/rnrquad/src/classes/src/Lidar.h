//
// Created by Florian VanKampen on 2019-06-17.
//

#ifndef RNRQUAD_LIDAR_H
#define RNRQUAD_LIDAR_H

class Lidar {
public:
  Lidar();

  float timeStamp; // Time of last Measurement
  float value;
};

#endif
