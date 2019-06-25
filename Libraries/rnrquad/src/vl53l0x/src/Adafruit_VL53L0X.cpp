#include "Adafruit_VL53L0X.h"

#define VERSION_REQUIRED_MAJOR  1
#define VERSION_REQUIRED_MINOR  0
#define VERSION_REQUIRED_BUILD  1

#define STR_HELPER(x) #x
#define STR(x)        STR_HELPER(x)


bool Adafruit_VL53L0X::begin(uint8_t i2c_addr, bool debug)
{
  int32_t status_int;
  int32_t init_done = 0;

  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;

  // Initialize Comms
  pMyDevice->I2cDevAddr = VL53L0X_I2C_ADDR;  // default
  pMyDevice->comms_type = 1;
  pMyDevice->comms_speed_khz = 400;
  errorCount = 0;

  Wire.begin();     // VL53L0X_i2c_init();
  if (debug)
  {
    Serial.println(F("debug on"));
  }
  Serial.print("Adafruit_VL53L0X ");
  Serial.println(__LINE__);

  // unclear if this is even needed:
  if (VL53L0X_IMPLEMENTATION_VER_MAJOR != VERSION_REQUIRED_MAJOR ||
      VL53L0X_IMPLEMENTATION_VER_MINOR != VERSION_REQUIRED_MINOR ||
      VL53L0X_IMPLEMENTATION_VER_SUB != VERSION_REQUIRED_BUILD)
  {
    if (debug)
    {
      Serial.println(
          F("Found " STR(VL53L0X_IMPLEMENTATION_VER_MAJOR) "." STR(VL53L0X_IMPLEMENTATION_VER_MINOR) "."  STR(
              VL53L0X_IMPLEMENTATION_VER_SUB) " rev " STR(VL53L0X_IMPLEMENTATION_VER_REVISION)));
      Serial.println(
          F("Requires " STR(VERSION_REQUIRED_MAJOR) "." STR(VERSION_REQUIRED_MINOR) "." STR(VERSION_REQUIRED_BUILD)));
    }
    Serial.print("Adafruit_VL53L0X ");
    Serial.println(__LINE__);
    Status = VL53L0X_ERROR_NOT_SUPPORTED;

    return false;
  }
  Serial.print("Adafruit_VL53L0X ");
  Serial.println(__LINE__);
  Status = VL53L0X_DataInit(&MyDevice);         // Data initialization
  Serial.print("Adafruit_VL53L0X ");
  Serial.println(__LINE__);
  if (!setAddress(i2c_addr))
  {
    return false;
  }
  Serial.print("Adafruit_VL53L0X ");
  Serial.println(__LINE__);
  Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
  Serial.print("Adafruit_VL53L0X ");
  Serial.println(__LINE__);
  if (Status == VL53L0X_ERROR_NONE)
  {
    if (debug)
    {
      Serial.println(F("VL53L0X Info:"));
      Serial.print(F("Device Name: "));
      Serial.print(DeviceInfo.Name);
      Serial.print(F(", Type: "));
      Serial.print(DeviceInfo.Type);
      Serial.print(F(", ID: "));
      Serial.println(DeviceInfo.ProductId);

      Serial.print(F("Rev Major: "));
      Serial.print(DeviceInfo.ProductRevisionMajor);
      Serial.print(F(", Minor: "));
      Serial.println(DeviceInfo.ProductRevisionMinor);
    }
  }

  if (Status == VL53L0X_ERROR_NONE)
  {
    if (debug)
    {
      Serial.println(F("VL53L0X: StaticInit"));
    }

    Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
  }

  if (Status == VL53L0X_ERROR_NONE)
  {
    if (debug)
    {
      Serial.println(F("VL53L0X: PerformRefSpadManagement"));
    }

    Status = VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads); // Device Initialization

    if (debug)
    {
      Serial.print(F("refSpadCount = "));
      Serial.print(refSpadCount);
      Serial.print(F(", isApertureSpads = "));
      Serial.println(isApertureSpads);
    }
  }

  if (Status == VL53L0X_ERROR_NONE)
  {
    if (debug)
    {
      Serial.println(F("VL53L0X: PerformRefCalibration"));
    }

    Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal);           // Device Initialization
  }

  if (Status == VL53L0X_ERROR_NONE)
  {
    // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
    if (debug)
    {
      Serial.println(F("VL53L0X: SetDeviceMode"));
    }

    Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);        // Setup in single ranging mode
  }

  // Enable/Disable Sigma and Signal check
  if (Status == VL53L0X_ERROR_NONE)
  {
    Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
  }

  if (Status == VL53L0X_ERROR_NONE)
  {
    Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  }

  if (Status == VL53L0X_ERROR_NONE)
  {
    Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
  }

  if (Status == VL53L0X_ERROR_NONE)
  {
    Status = VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                        (FixPoint1616_t) (1.5 * 0.023 * 65536));
  }

  if (Status == VL53L0X_ERROR_NONE)
  {
    return true;
  } else
  {
    if (debug)
    {
      Serial.print(F("VL53L0X Error: "));
      Serial.println(Status);
    }

    return false;
  }
}

bool Adafruit_VL53L0X::setAddress(uint8_t newAddr)
{
  newAddr &= 0x7F;

  VL53L0X_SetDeviceAddress(pMyDevice, newAddr * 2); // 7->8 bit

  delay(10);

  if (Status == VL53L0X_ERROR_NONE)
  {
    pMyDevice->I2cDevAddr = newAddr;  // 7 bit addr
    return true;
  }
  return false;
}

VL53L0X_Error Adafruit_VL53L0X::startRanging()
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  VL53L0X_RangingMeasurementData_t measure;
  Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

  if (Status != VL53L0X_ERROR_NONE)
  {
    errorCount++;
    if (errorCount < 5)
    {
      Serial.println("Status borked in call to VL53L0X_SetDeviceMode()");
    }
    return Status;
  }
  Status = VL53L0X_StartMeasurement(pMyDevice);
  if (Status != VL53L0X_ERROR_NONE)
  {
    errorCount++;
    if (errorCount < 5)
    {
      Serial.println("Status borked in call to VL53L0X_StartMeasurement()");
    }
  }
  return Status;
}

bool Adafruit_VL53L0X::checkRangeReady()
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint8_t mReady;
  // Function to get a bit that tells if a measurement is ready, without waiting.
  Status = VL53L0X_GetMeasurementDataReady(pMyDevice, &mReady);
  if (Status != VL53L0X_ERROR_NONE)
  {
    errorCount++;
    if (errorCount < 5)
    {
      Serial.println("Status borked in call to VL53L0X_GetMeasurementDataReady()");
    }
  }
  return (bool) mReady;
}

bool Adafruit_VL53L0X::updateRangeInMeters(float *range)
{
  bool success = false;
  VL53L0X_RangingMeasurementData_t measure;
  if (checkRangeReady())
  {
    Status = VL53L0X_GetRangingMeasurementData(pMyDevice, &measure);
    if (Status == VL53L0X_ERROR_NONE && measure.RangeStatus != 255)
    {
      if (measure.RangeStatus == 0 || measure.RangeStatus == 2)
      {
        *range = (float) measure.RangeMilliMeter * 0.001;
        if (*range > 1.3)
        {
          *range = 1.3; // Sometimes with RangeStatus == 2, the range is reported as 8192.
        }
        success = true;
      } else if (measure.RangeStatus == 4)
      {
        *range = 1.3;
        success = true;
      } else
      {
        // Serial.print("RS:");
        // Serial.println(measure.RangeStatus);
      }
      Status = VL53L0X_ClearInterruptMask(pMyDevice, 0);
      if (Status != 0)
      {
        Serial.print(Status);
        Serial.print(" updateRangeInMeters() ");
        Serial.println(measure.RangeStatus);
      }
      if (Status != VL53L0X_ERROR_NONE)
      {
        Status = startRanging();
      }
    }
  }
  return success;
}

float Adafruit_VL53L0X::getRangeInMeters()
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  VL53L0X_RangingMeasurementData_t measure;
  float range = 1.3;

  // Read the measurement from a device that is ready.
  Status = VL53L0X_GetRangingMeasurementData(pMyDevice, &measure);
  if (Status == VL53L0X_ERROR_NONE)
  {
    if (measure.RangeStatus != 4 && measure.RangeStatus != 255)
    {
      range = (float) measure.RangeMilliMeter * 0.001;
    } else
    {
      return range;
    }
  } else
  {
    Serial.print(__FILE__);
    Serial.print(__LINE__);
    Serial.print(Status);
    Serial.println();
  }
  // Call this to clear the interrupt, or in our case the measurementDataReady bit.
  Status = VL53L0X_ClearInterruptMask(pMyDevice, 0);
  if (Status != 0)
  {
    Serial.print(Status);
    Serial.println(" getRangeInMeters()");
  }
  startRanging();
  return range;
}

VL53L0X_Error
Adafruit_VL53L0X::getSingleRangingMeasurement(VL53L0X_RangingMeasurementData_t *RangingMeasurementData, bool debug)
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  FixPoint1616_t LimitCheckCurrent;
  /*
      Step  4 : Test ranging mode
  */

  if (Status == VL53L0X_ERROR_NONE)
  {
    if (debug)
    {
      Serial.println(F("sVL53L0X: PerformSingleRangingMeasurement"));
    }
    Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice, RangingMeasurementData);

    if (debug)
    {
      printRangeStatus(RangingMeasurementData);
    }

    if (debug)
    {
      VL53L0X_GetLimitCheckCurrent(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

      Serial.print(F("RANGE IGNORE THRESHOLD: "));
      Serial.println((float) LimitCheckCurrent / 65536.0);

      Serial.print(F("Measured distance: "));
      Serial.println(RangingMeasurementData->RangeMilliMeter);
    }
  }

  return Status;
}


void Adafruit_VL53L0X::printRangeStatus(VL53L0X_RangingMeasurementData_t *pRangingMeasurementData)
{
  char buf[VL53L0X_MAX_STRING_LENGTH];
  uint8_t RangeStatus;

  /*
     New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
  */

  RangeStatus = pRangingMeasurementData->RangeStatus;

  VL53L0X_GetRangeStatusString(RangeStatus, buf);

  Serial.print(F("Range Status: "));
  Serial.print(RangeStatus);
  Serial.print(F(" : "));
  Serial.println(buf);

}

