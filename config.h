#pragma once

#include "calibration.h"


struct Config {
  int magic;
  int version;
  CalibrationPoint servo_calibration[4][3][2];
};

extern Config config;

void ReadConfig();
void StoreConfig();
