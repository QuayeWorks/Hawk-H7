/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "power.h"
#include "settings.h"
#include "flight_state.h"
#include "imu.h"        // your MPU6050 driver
#include "compass.h"    // your QMC5883/MC5883 driver
#include "gps.h"
#include "rc_input.h"   // RC/PPM reading library
#include "motor.h"      // ESC/PWM library
#include "servo.h"
#include "telemetry.h"
#include "geofence.h"
#include "failsafe.h"
#include "led.h"
#include "attitude.h"
#include "cpu.h"
#include "buzzer.h"
#include "ekf.h"
#include "sonar.h"
#include "baro.h"
#include "battery.h"
#include "debug_menu.h"
#include "sensor_diag.h"
#include "at7456e.h"
#include "bt_link.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define BUZZER_ERROR_MIN_INTERVAL_MS 1000u
#define GPS_DROPPED_BYTES_PER_200MS_THRESHOLD 8u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/** UART receive complete callback inserted in user callbacks **/


static void DebugMsg(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

static uint32_t buzzerErrorLastMs[TONE_COUNT] = {0u};

static void Buzzer_PlayErrorRateLimited(Buzzer_ToneID tone, uint32_t now_ms)
{
    if (tone >= TONE_COUNT) {
        return;
    }
    if (buzzerErrorLastMs[tone] == 0u ||
        (now_ms - buzzerErrorLastMs[tone]) >= BUZZER_ERROR_MIN_INTERVAL_MS) {
        buzzerErrorLastMs[tone] = now_ms;
        Buzzer_PlayTone(tone);
    }
}

static volatile SensorRateReport g_sensorRates = {0};

void SensorDiag_GetRates(SensorRateReport *out)
{
    if (out == NULL) {
        return;
    }

    __disable_irq();
    *out = g_sensorRates;
    __enable_irq();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_ADC3_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  Buzzer_Config bzCfg = {
      .htim    = &htim12,
      .channel = TIM_CHANNEL_1,
      .port    = NULL,  // PWM-only mode
      .pin     = 0
  };
  Buzzer_Init(&bzCfg);
  AT7456_Init(&hspi4, OSD_CS_GPIO_Port, OSD_CS_Pin);
  DebugMsg("AT7456 init done\r\n");
  Power_Init();
  // Keep avionics rails enabled during bring-up so sensors on buck rails are powered.
  Power_SetBuckEnable(0u, 1u);
  Power_SetBuckEnable(1u, 1u);
  Power_SetBuckEnable(2u, 1u);
  Power_SetBuckEnable(3u, 1u);
  HAL_Delay(20);
  DebugMsg("Power init done\r\n");
  // MX_FATFS_Init() already links SD_Driver to SDPath.

  // Mount the SD card file system.
  const char *sdPath = (SDPath[0] != '\0') ? SDPath : "0:/";
  FRESULT res = f_mount(&SDFatFS, sdPath, 1);
  if (res == FR_NOT_READY) {
      HAL_Delay(50);
      res = f_mount(&SDFatFS, sdPath, 1);
  }
  if (res != FR_OK) {
      char mountMsg[48];
      int mountLen = snprintf(mountMsg, sizeof(mountMsg),
                              "ERROR: f_mount(%s) fr=%d\r\n", sdPath, (int)res);
      if (mountLen > 0) {
          HAL_UART_Transmit(&huart1, (uint8_t*)mountMsg, (uint16_t)mountLen, HAL_MAX_DELAY);
      }
  } else {
      // Now “0:/” is valid. Initialize settings (maybe creates file):
      if (!Settings_Init("0:/settings.ini")) {
          HAL_UART_Transmit(&huart1, (uint8_t*)"ERROR: Settings_Init failed.\r\n", 28, HAL_MAX_DELAY);
      }
      else {
          HAL_UART_Transmit(&huart1, (uint8_t*)"OK: settings.ini created/loaded.\r\n", 32, HAL_MAX_DELAY);
      }
  }

#if BT_USE_OLD_BOARD_PE8_PE9
  if (BTLink_Init(NULL, 9600u)) {
      DebugMsg("BT link init done\r\n");
  } else {
      DebugMsg("WARN: BT link init failed\r\n");
  }
#endif

  // 3) Initialize all sensor buses & devices
  IMU_Init(&hi2c1);        // begin MPU6050 DMP or raw-data streaming
  if (!IMU_IsIdentityOK()) {
      // Some builds wire IMU_EN active-low; try fallback polarity once.
      HAL_GPIO_WritePin(MPU6050_EN_GPIO_Port, MPU6050_EN_Pin, GPIO_PIN_RESET);
      HAL_Delay(20);
      IMU_Init(&hi2c1);
      if (!IMU_IsIdentityOK()) {
          HAL_GPIO_WritePin(MPU6050_EN_GPIO_Port, MPU6050_EN_Pin, GPIO_PIN_SET);
          HAL_Delay(20);
          IMU_Init(&hi2c1);
      }
  }
  DebugMsg("IMU init done\r\n");
  Compass_Init(&hi2c1);    // memory magnetometer
  DebugMsg("Compass init done\r\n");
  GPS_Init(&huart6);      // USART6
  DebugMsg("GPS init done\r\n");
  RC_Input_Init();   // PPM input via TIM2 interrupts
  DebugMsg("RC input init done\r\n");
  Sonar_Init();      // e.g. HC‐SR04 trigger/echo with TIM3
  DebugMsg("Sonar init done\r\n");
  Battery_Init(&hi2c1, &hadc3, Settings_GetINA219ShuntOhm());    // INA219 via I2C1 and ADC3 on PH2
  DebugMsg("Battery init done\r\n");
  if (Settings_GetBaroEnabled()) {
      Baro_Init(&hi2c1, 0u); // auto-probe 0x76/0x77 on I2C1
      DebugMsg("Baro init done\r\n");
  } else {
      DebugMsg("Baro disabled in settings\r\n");
  }

  EKF_Init();              // your EKF/UKF library initialization
  DebugMsg("EKF init done\r\n");

  DebugMenu_Init(&huart1);
  DebugMsg("Initialization complete\r\n");
  DebugMenu_SetActionMask(DEBUG_MENU_NONE);

  Motor_Init();      // ESC PWM outputs
  Servo_Init();




#ifndef DEBUG_BYPASS_HEALTH
  // 4) Pre‐arm Calibration (Power ON → IMU level & bias)

  //    This block runs only once at startup, before any arming is allowed.
  // Declare locals to hold the computed biases
  float imu_bx, imu_by, imu_bz;

  if (Settings_GetCalibrateOnBoot()) {
          bool imu_ok = IMU_IsIdentityOK() && IMU_CalibrateOnBoot(
                          Settings_GetCalibSamples(),
                          Settings_GetAccelTolG(),
                          &imu_bx, &imu_by, &imu_bz);

      if (imu_ok) {
    	  // save them back into settings (and IMU’s internal bias storage):
    	  Settings_SetAccelBiases(imu_bx, imu_by, imu_bz);
          FlightState_SetHealth(FS_HEALTH_IMU_OK_BIT);
      } else {
          FlightState_ClearHealth(FS_HEALTH_IMU_OK_BIT);
          // If you want an immediate failure tone:
          Buzzer_PlayErrorRateLimited(TONE_ERROR_IMU, HAL_GetTick());
      }
  } else {
      // Skip calibration on boot; assume old biases from Settings are already correct
      if (IMU_IsIdentityOK()) {
          FlightState_SetHealth(FS_HEALTH_IMU_OK_BIT);
      } else {
          FlightState_ClearHealth(FS_HEALTH_IMU_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_IMU, HAL_GetTick());
      }
  }
  DebugMsg("IMU calibration done\r\n");

  // 5) Load any compass offsets from settings
  if (Compass_IsStub()) {
      FlightState_ClearHealth(FS_HEALTH_COMPASS_OK_BIT);
      DebugMsg("Compass stub detected\r\n");
  } else if (Settings_GetCompassEnabled()) {
      Compass_LoadCalibration(
          Settings_GetMagSoftIronX(),
          Settings_GetMagSoftIronY(),
          Settings_GetMagSoftIronZ(),
          Settings_GetMagHardIronX(),
          Settings_GetMagHardIronY(),
          Settings_GetMagHardIronZ());

      // Immediately check one sample to see if compass is sane:
      int16_t  mx, my, mz;
      bool comp_ok = Compass_ReadRaw(&mx, &my, &mz);
      if (comp_ok) {
    	  // Compute heading if you need it for display or telemetry:
    	  //float heading = Compass_ComputeHeading(mx, my, mz);

    	  // Mahalanobis‐distance check: only pass raw counts + threshold
    	  float mahaThresh = Settings_GetMagMahaThreshold();
    	  if (Compass_CheckMahalanobis(mx, my, mz, mahaThresh)) {
    	          FlightState_SetHealth(FS_HEALTH_COMPASS_OK_BIT);
    	      } else {
    	          FlightState_ClearHealth(FS_HEALTH_COMPASS_OK_BIT);
    	          Buzzer_PlayErrorRateLimited(TONE_ERROR_COMPASS, HAL_GetTick());
    	      }
      } else {
          FlightState_ClearHealth(FS_HEALTH_COMPASS_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_COMPASS, HAL_GetTick());
      }
  } else {
      // If the user disabled compass in settings, consider it “OK” by default
      FlightState_SetHealth(FS_HEALTH_COMPASS_OK_BIT);
  }
  DebugMsg("Compass check done\r\n");

  // 6) Barometer pre-check (if used)
  if (Settings_GetBaroEnabled()) {
      bool baro_ok = false;
      uint32_t t0 = HAL_GetTick();
      if (Baro_IsIdentityOK()) {
          while ((HAL_GetTick() - t0) < 600U) {
              uint32_t now_ms = HAL_GetTick();
              float tolerance = Settings_GetBaroToleranceHpa();
              float pressureRef = Settings_GetBaroPressureOffsetHpa();
              if (Baro_Update(now_ms)) {
                  EKF_PublishBaro(Baro_GetAltitudeMeters());
                  if (tolerance <= 0.0f ||
                      fabsf(Baro_GetPressureHpa() - pressureRef) <= tolerance) {
                      baro_ok = true;
                      break;
                  }
              }
              HAL_Delay(20);
          }
      }

      if (baro_ok) {
          FlightState_SetHealth(FS_HEALTH_BARO_OK_BIT);
      } else {
          FlightState_ClearHealth(FS_HEALTH_BARO_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_BARO, HAL_GetTick());
      }
  } else {
      FlightState_SetHealth(FS_HEALTH_BARO_OK_BIT);
  }
  DebugMsg("Barometer pre-check done\r\n");



  // 7) Sonar pre‐check (if used)
  if (Settings_GetSonarEnabled()) {
      Sonar_TriggerAll();
      HAL_Delay(60);
      bool anyOk = false;
      for (uint8_t idx = 0; idx < 3; idx++) {
          float sr = Sonar_ReadDistance(idx);
          if (sr >= Settings_GetSonarMinDistance() &&
              sr <= Settings_GetSonarMaxDistance())
          {
              anyOk = true;
              break;
          }
      }
      if (anyOk) {
          FlightState_SetHealth(FS_HEALTH_SONAR_OK_BIT);
      } else {
          FlightState_ClearHealth(FS_HEALTH_SONAR_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_SONAR, HAL_GetTick());
      }
  } else {
      FlightState_SetHealth(FS_HEALTH_SONAR_OK_BIT);
  }
  DebugMsg("Sonar pre-check done\r\n");

  // 8) GPS pre‐check
  if (Settings_GetGPSEnabled()) {
      // Try to wait up to Settings_GetGPSRequiredTimeSec() seconds for satellites
      bool gps_ok = false;
      uint32_t t0 = HAL_GetTick();
      while ((HAL_GetTick() - t0) < (Settings_GetGPSRequiredTimeSec()*1000UL)) {
          GPS_Update();
          if ((GPS_GetSatCount() >= Settings_GetGPSMinSatellites()) &&
              (GPS_GetHDOP() <= Settings_GetGPSMaxHDOP()) &&
              (GPS_CheckDriftOK(Settings_GetGPSHoldAfterLossSec()))) {
              gps_ok = true;
              break;
          }
          HAL_Delay(100);  // check every 100 ms
      }
      if (gps_ok) {
          FlightState_SetHealth(FS_HEALTH_GPS_OK_BIT);
          // Save home position from current GPS fix
          Settings_SetHomeLatitude(GPS_GetLatitude());
          Settings_SetHomeLongitude(GPS_GetLongitude());
          Settings_SetHomeAltitude(GPS_GetAltitude());
      } else {
          FlightState_ClearHealth(FS_HEALTH_GPS_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_GPS, HAL_GetTick());
      }
  } else {
      FlightState_SetHealth(FS_HEALTH_GPS_OK_BIT);
  }
  DebugMsg("GPS pre-check done\r\n");

  // 9) RC link pre‐check
  {
      // Example: RSSI read from ADC or UART, channel jitter from PPM input, sticks reading
      uint16_t rssi = RC_GetRSSI();
      uint16_t rssiMin = Settings_GetRCRSSIMin();
      bool rssi_ok = (rssiMin == 0U) || (rssi >= rssiMin);
      bool channels_ok = RC_AllChannelsStable();
      bool sticks_low_yaw_centered =
          (RC_GetChannel(Settings_GetRCThrottleChannel()) < Settings_GetRCArmStickThreshold()) &&
          (abs(RC_GetChannel(Settings_GetRCRollChannel()) - 1500) < Settings_GetRCCenterThreshold()) &&
          (abs(RC_GetChannel(Settings_GetRCYawChannel())  - 1500) < Settings_GetRCCenterThreshold());

      if (rssi_ok && channels_ok && sticks_low_yaw_centered) {
          FlightState_SetHealth(FS_HEALTH_RC_OK_BIT);
      } else {
          FlightState_ClearHealth(FS_HEALTH_RC_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_RC, HAL_GetTick());
      }
    }
    DebugMsg("RC link pre-check done\r\n");

    // 10) Battery pre‐check under no load (just check voltage per cell)
  {
      float packVolts = Battery_ReadPackVoltage();
      float perCell = Battery_ReadPerCellVoltage();
      uint8_t cellCount = Settings_GetBatteryCellCount();
      if ((packVolts / cellCount) >= Settings_GetBatteryWarningVolts()) {
          FlightState_SetHealth(FS_HEALTH_BATT_OK_BIT);
      } else {
          FlightState_ClearHealth(FS_HEALTH_BATT_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_BATTERY, HAL_GetTick());
      }

      if (perCell >= Settings_GetBatteryWarningVolts()) {
          FlightState_SetHealth(FS_HEALTH_BATT_OK_BIT);
      } else {
          FlightState_ClearHealth(FS_HEALTH_BATT_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_BATTERY, HAL_GetTick());
      }

  }

  DebugMsg("Battery pre-check done\r\n");

  // —— Pre‐arm Calibration & Health Checks ——
  // (IMU level/bias, compass Mahalanobis, baro, sonar, GPS, RC, battery)
  // After you have read and set each FS_HEALTH_XXX_BIT, **add this:**

  {
      // Example: assume you have ax,ay,az,gx,gy,gz from IMU_ReadRaw()/Get*
      float ax, ay, az, gx, gy, gz;
      IMU_GetAccelMps2(&ax, &ay, &az);
      IMU_GetGyroDPS(&gx, &gy, &gz);

      // 1) update the simple attitude filter for control/failsafes:
      Attitude_Update(ax, ay, az, gx, gy, gz);

      // 2) publish into the EKF for full state estimation & health gating:
      EKF_PublishIMU(ax, ay, az, gx, gy, gz);

      // assume mx,my,mz and heading
      int16_t mx,my,mz;
      Compass_ReadRaw(&mx,&my,&mz);
      float heading = Compass_ComputeHeading(mx, my, mz);
      EKF_PublishMag(heading);


      // assume lat,lon,alt from GPS
      double lat = GPS_GetLatitude();
      double lon = GPS_GetLongitude();
      float  alt = GPS_GetAltitude();
      EKF_PublishGPS(lat, lon, alt);
      if (Settings_GetBaroEnabled() && Baro_HasSample()) {
          EKF_PublishBaro(Baro_GetAltitudeMeters());
      }

      EKF_UpdateSensors();

      if (EKF_AllInnovationGatesOK() && EKF_CovariancesConverged()) {
          FlightState_SetHealth(FS_HEALTH_EKF_OK_BIT);
      } else {
          FlightState_ClearHealth(FS_HEALTH_EKF_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_EKF, HAL_GetTick());
      }
  }
  DebugMsg("EKF sensor publish done\r\n");
  // —————————————————————————————————————————

  // 11) EKF health pre‐check (if you have a ready‐made EKF routine)
  {
      EKF_UpdateSensors(); // do one update from IMU+GPS+Mag
      if (EKF_AllInnovationGatesOK() && EKF_CovariancesConverged()) {
          FlightState_SetHealth(FS_HEALTH_EKF_OK_BIT);
      } else {
          FlightState_ClearHealth(FS_HEALTH_EKF_OK_BIT);
          Buzzer_PlayErrorRateLimited(TONE_ERROR_EKF, HAL_GetTick());
      }
  }
  DebugMsg("EKF health pre-check done\r\n");

  // At this point, FlightState_Update() has already been called by every SetHealth()/ClearHealth().
  // If all health bits are set, FS_READY_BIT will be set.  Otherwise, you are stuck “un‐ready.”

  // 12) Motor spin‐up & throttle calibration (only if Ready==true, but not yet armed)
  if (FlightState_IsReady()) {
      // Pulse each ESC at a low throttle and watch the RPM or current response:
      // (This is just pseudo‐code; adapt to your ESC/telemetry setup)
      for (uint8_t i = 0; i < Settings_GetMotorCount(); i++) {
          uint16_t pwmMin = Settings_GetMotorPWMMinUs();
          Motor_SetPWM(i, pwmMin + (Settings_GetMotorPWMMaxUs() - pwmMin) / 10);
          HAL_Delay(200);
          if (!Motor_EscResponds(i)) {
              // Mark motor health as failed—this could be an additional bit in fsState,
              // or you could lump it under a generic “motor health” flag. For now, we’ll
              // disarm immediately and treat it as a failsafe.
              FlightState_Disarm();
              Buzzer_PlayErrorRateLimited(TONE_ERROR_MOTOR, HAL_GetTick());
          }
          Motor_SetPWM(i, pwmMin);  // set back to idle
          HAL_Delay(100);
      }
      // If all ESCs responded, you may choose to write back the discovered PWM min/max
      // into settings.ini (if dynamic) via Settings_SetMotorPWMMinUs() etc.
      Buzzer_PlayTone(TONE_READY);
  }
  DebugMsg("Motor spin-up calibration done\r\n");

  // 13) Motor orientation auto‐config (even‐count props only)
  if (Settings_GetMotorAutoDetect()) {
      // Implement a routine that pings each motor with a short pulse and uses a spin‐sensory
      // method (e.g. back‐EMF) to figure out actual rotation direction. If the detected
      // direction differs from Settings_GetMotorDir(i), flip it and then call
      // Settings_SetMotorDir(i, newDir).
      Motor_AutoDetectOrientation();
  }
  DebugMsg("Motor orientation auto-config done\r\n");

  // 14) Now we wait for the actual “arming command” from the pilot →
  //     (stick positions or a dedicated switch) → but only if Ready==true
  //     We assume RC_GetChannel(…) returns a 1000–2000us PPM value, center=1500.
  bool prearmOK = false;
  if (FlightState_IsReady()) {
      // You can allow either a 3-position switch (Settings_GetPrearmSwitchEnabled)
      // or the “sticks crossed” method (roll left + yaw right) to arm.
      if (Settings_GetPrearmSwitchEnabled()) {
          uint16_t switchVal = RC_GetChannel(Settings_GetPrearmSwitchChannel());
          if (switchVal >= Settings_GetPrearmSwitchHigh()) {
              prearmOK = true;
          }
      } else {
          // “sticks crossed” method: roll left (e.g. <1100) AND yaw right (e.g. >1900)
          uint16_t roll = RC_GetChannel(Settings_GetRCRollChannel());
          uint16_t yaw  = RC_GetChannel(Settings_GetRCYawChannel());
          if (roll < Settings_GetRCDisarmStickThreshold() &&
              yaw  > Settings_GetRCArmStickThreshold()) {
              prearmOK = true;
          }
      }

  if (prearmOK) {
      FlightState_Arm();
      HAL_UART_Transmit(&huart1, (uint8_t*)"OK: Armed!\r\n", 12, HAL_MAX_DELAY);
  }
}
  DebugMsg("Pre-arm sequence complete\r\n");

#endif // DEBUG_BYPASS_HEALTH

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

        uint32_t now = HAL_GetTick();
    static uint32_t lastImuMs = 0;
    static uint32_t lastMagMs = 0;
    static uint32_t lastBaroMs = 0;
    static uint32_t lastSonarMs = 0;
    static uint32_t lastGpsMs = 0;
    static uint32_t lastGpsParseMs = 0;
    static uint32_t lastRcMs = 0;
    static uint32_t lastBatteryMs = 0;
    static uint32_t lastEkfMs = 0;
    static uint32_t lastRatesMs = 0;

    static uint32_t imuSamples = 0;
    static uint32_t magSamples = 0;
    static uint32_t baroSamples = 0;
    static uint32_t sonarSamples = 0;
    static uint32_t batterySamples = 0;
    static uint32_t gpsCountPrev = 0;
    static uint32_t gpsDropPrev = 0;
    static uint32_t gpsOvfLastLogMs = 0;
    static uint32_t rcFramePrev = 0;

    // Dedicated sensor scheduler to avoid stale values and jittery reads.
    if ((now - lastImuMs) >= 2U) { // 500 Hz
        int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
        lastImuMs = now;
        if (!IMU_IsIdentityOK()) {
            FlightState_ClearHealth(FS_HEALTH_IMU_OK_BIT);
            Buzzer_PlayErrorRateLimited(TONE_ERROR_IMU, now);
        } else if (IMU_ReadRaw(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz)) {
            float ax, ay, az, gx, gy, gz;
            IMU_GetAccelMps2(&ax, &ay, &az);
            IMU_GetGyroDPS(&gx, &gy, &gz);
            Attitude_Update(ax, ay, az, gx, gy, gz);
            EKF_PublishIMU(ax, ay, az, gx, gy, gz);
            imuSamples++;
            if (IMU_CheckPlausibility()) {
                FlightState_SetHealth(FS_HEALTH_IMU_OK_BIT);
            } else {
                FlightState_ClearHealth(FS_HEALTH_IMU_OK_BIT);
                Buzzer_PlayErrorRateLimited(TONE_ERROR_IMU, now);
            }
        } else {
            FlightState_ClearHealth(FS_HEALTH_IMU_OK_BIT);
            Buzzer_PlayErrorRateLimited(TONE_ERROR_IMU, now);
        }
    }

    if ((now - lastMagMs) >= 20U) { // 50 Hz
        lastMagMs = now;
        if (Compass_IsStub()) {
            FlightState_ClearHealth(FS_HEALTH_COMPASS_OK_BIT);
        } else if (Settings_GetCompassEnabled()) {
            int16_t mx, my, mz;
            if (Compass_ReadRaw(&mx, &my, &mz)) {
                float mahaThresh = Settings_GetMagMahaThreshold();
                if (Compass_CheckMahalanobis(mx, my, mz, mahaThresh)) {
                    float heading = Compass_ComputeHeading(mx, my, mz);
                    EKF_PublishMag(heading);
                    magSamples++;
                    FlightState_SetHealth(FS_HEALTH_COMPASS_OK_BIT);
                } else {
                    FlightState_ClearHealth(FS_HEALTH_COMPASS_OK_BIT);
                    Buzzer_PlayErrorRateLimited(TONE_ERROR_COMPASS, now);
                }
            } else {
                FlightState_ClearHealth(FS_HEALTH_COMPASS_OK_BIT);
                Buzzer_PlayErrorRateLimited(TONE_ERROR_COMPASS, now);
            }
        } else {
            FlightState_SetHealth(FS_HEALTH_COMPASS_OK_BIT);
        }
    }

    if ((now - lastBaroMs) >= 20U) { // 50 Hz
        lastBaroMs = now;
        if (Settings_GetBaroEnabled()) {
            if (!Baro_IsIdentityOK()) {
                FlightState_ClearHealth(FS_HEALTH_BARO_OK_BIT);
                Buzzer_PlayErrorRateLimited(TONE_ERROR_BARO, now);
            } else if (Baro_Update(now)) {
                float pressure = Baro_GetPressureHpa();
                float pressureRef = Settings_GetBaroPressureOffsetHpa();
                float pressureTol = Settings_GetBaroToleranceHpa();
                bool pressureOk = (pressureTol <= 0.0f) ||
                                  (fabsf(pressure - pressureRef) <= pressureTol);
                EKF_PublishBaro(Baro_GetAltitudeMeters());
                baroSamples++;
                if (pressureOk) {
                    FlightState_SetHealth(FS_HEALTH_BARO_OK_BIT);
                } else {
                    FlightState_ClearHealth(FS_HEALTH_BARO_OK_BIT);
                    Buzzer_PlayErrorRateLimited(TONE_ERROR_BARO, now);
                }
            } else {
                FlightState_ClearHealth(FS_HEALTH_BARO_OK_BIT);
                Buzzer_PlayErrorRateLimited(TONE_ERROR_BARO, now);
            }
        } else {
            FlightState_SetHealth(FS_HEALTH_BARO_OK_BIT);
        }
    }

    if ((now - lastGpsParseMs) >= 2U) { // 500 Hz sentence parsing from RX ring buffer
        lastGpsParseMs = now;
        GPS_Update();
    }

    {
        uint16_t sonarRateHz = Settings_GetSonarUpdateRateHz();
        uint32_t sonarIntervalMs;
        if (sonarRateHz == 0U) {
            sonarRateHz = 10U;
        }
        sonarIntervalMs = 1000U / (uint32_t)sonarRateHz;
        if (sonarIntervalMs == 0U) {
            sonarIntervalMs = 1U;
        }

    if ((now - lastSonarMs) >= sonarIntervalMs) { // configurable, typically 10–20 Hz
        lastSonarMs = now;
        if (Settings_GetSonarEnabled()) {
            bool anyOk = false;
            Sonar_TriggerAll();
            for (uint8_t idx = 0; idx < SONAR_COUNT; idx++) {
                float d = Sonar_ReadDistance(idx);
                if (d >= Settings_GetSonarMinDistance() &&
                    d <= Settings_GetSonarMaxDistance()) {
                    anyOk = true;
                }
            }
            if (anyOk) {
                FlightState_SetHealth(FS_HEALTH_SONAR_OK_BIT);
                sonarSamples++;
            } else {
                FlightState_ClearHealth(FS_HEALTH_SONAR_OK_BIT);
                Buzzer_PlayErrorRateLimited(TONE_ERROR_SONAR, now);
            }
        } else {
            FlightState_SetHealth(FS_HEALTH_SONAR_OK_BIT);
        }
    }
    }

    if ((now - lastGpsMs) >= 200U) { // 5 Hz GPS health gating (data parsed in main loop)
        lastGpsMs = now;
        if (Settings_GetGPSEnabled()) {
            char gpsOvfMsg[32];
            uint32_t gpsDropNow = GPS_GetDroppedByteCount();
            uint32_t gpsDropDelta = gpsDropNow - gpsDropPrev;
            bool gps_overflow_ok = (gpsDropDelta <= GPS_DROPPED_BYTES_PER_200MS_THRESHOLD);
            gpsDropPrev = gpsDropNow;
            if (!FlightState_IsArmed() &&
                !gps_overflow_ok &&
                ((now - gpsOvfLastLogMs) >= 1000U)) {
                snprintf(gpsOvfMsg, sizeof(gpsOvfMsg), "WARN code=GPS_RX_OVF delta=%lu\r\n",
                         (unsigned long)gpsDropDelta);
                DebugMsg(gpsOvfMsg);
                gpsOvfLastLogMs = now;
            }
            bool gps_ok = GPS_HasFix() &&
                          (GPS_GetSatCount() >= Settings_GetGPSMinSatellites()) &&
                          (GPS_GetHDOP() <= Settings_GetGPSMaxHDOP()) &&
                          GPS_CheckDriftOK(Settings_GetGPSHoldAfterLossSec()) &&
                          gps_overflow_ok;
            if (gps_ok) {
                FlightState_SetHealth(FS_HEALTH_GPS_OK_BIT);
                EKF_PublishGPS(GPS_GetLatitude(), GPS_GetLongitude(), GPS_GetAltitude());
            } else {
                FlightState_ClearHealth(FS_HEALTH_GPS_OK_BIT);
                Buzzer_PlayErrorRateLimited(TONE_ERROR_GPS, now);
            }
        } else {
            FlightState_SetHealth(FS_HEALTH_GPS_OK_BIT);
        }
    }

    if ((now - lastRcMs) >= 10U) { // 100 Hz housekeeping, frames arrive on EXTI
        uint16_t rssiMin;
        bool rssi_low;
        lastRcMs = now;
        RC_Input_Update(now);
        rssiMin = Settings_GetRCRSSIMin();
        rssi_low = (rssiMin > 0U) && (RC_GetRSSI() < rssiMin);
        if (RC_ChannelsAreStale(500U) || rssi_low) {
            FlightState_ClearHealth(FS_HEALTH_RC_OK_BIT);
            Buzzer_PlayErrorRateLimited(TONE_ERROR_RC, now);
        } else {
            FlightState_SetHealth(FS_HEALTH_RC_OK_BIT);
        }
    }

    if ((now - lastBatteryMs) >= 20U) { // 50 Hz
        float perCell;
        lastBatteryMs = now;
        Battery_Tick(now);
        batterySamples++;
        perCell = Battery_ReadPerCellVoltage();
        if (perCell >= Settings_GetBatteryWarningVolts()) {
            FlightState_SetHealth(FS_HEALTH_BATT_OK_BIT);
        } else {
            FlightState_ClearHealth(FS_HEALTH_BATT_OK_BIT);
            if (FlightState_IsArmed()) {
                Buzzer_PlayTone(TONE_WARN_BATT_LOW);
            }
        }
    }

    if ((now - lastEkfMs) >= 5U) { // 200 Hz
        lastEkfMs = now;
        EKF_UpdateSensors();
        if (EKF_AllInnovationGatesOK() && EKF_CovariancesConverged()) {
            FlightState_SetHealth(FS_HEALTH_EKF_OK_BIT);
        } else {
            FlightState_ClearHealth(FS_HEALTH_EKF_OK_BIT);
            if (FlightState_IsArmed()) {
                Buzzer_PlayErrorRateLimited(TONE_ERROR_EKF, now);
            }
        }
    }

    if ((now - lastRatesMs) >= 1000U) {
        uint32_t gpsCount = GPS_GetUpdateCount();
        uint32_t rcFrames = RC_GetFrameCount();

        g_sensorRates.imu_hz = (uint16_t)imuSamples;
        g_sensorRates.mag_hz = (uint16_t)magSamples;
        g_sensorRates.baro_hz = (uint16_t)baroSamples;
        g_sensorRates.sonar_hz = (uint16_t)sonarSamples;
        g_sensorRates.battery_hz = (uint16_t)batterySamples;
        g_sensorRates.gps_hz = (uint16_t)(gpsCount - gpsCountPrev);
        g_sensorRates.rc_hz = (uint16_t)(rcFrames - rcFramePrev);

        imuSamples = 0;
        magSamples = 0;
        baroSamples = 0;
        sonarSamples = 0;
        batterySamples = 0;
        gpsCountPrev = gpsCount;
        rcFramePrev = rcFrames;
        lastRatesMs = now;
    }

    FlightState_Update();

    // b) If armed, start sending throttle to motors. Otherwise keep motors at idle:
    if (FlightState_IsArmed()) {
        uint16_t thr = RC_GetChannel(Settings_GetRCThrottleChannel());
        // Scale from PPM value (1000–2000) to your ESC range (motor_pwmMinUs … motor_pwmMaxUs)
        uint16_t pwmOut = Motor_ThrottleToPWM(thr);
        Motor_SetAllPWM(pwmOut);
    } else {
        // Disarmed ⇒ set motors to idle
        for (uint8_t i = 0; i < Settings_GetMotorCount(); i++) {
            Motor_SetPWM(i, Settings_GetMotorPWMMinUs());
        }
    }

    // Additional plausibility check: if baro/gps altitude diverges too much, mark baro unhealthy.
    if (Settings_GetBaroEnabled() && Baro_HasSample() && GPS_HasFix()) {
        float altDelta = fabsf(Baro_GetAltitudeMeters() - GPS_GetAltitude());
        if (altDelta > 80.0f) {
            FlightState_ClearHealth(FS_HEALTH_BARO_OK_BIT);
            Buzzer_PlayErrorRateLimited(TONE_ERROR_BARO, now);
        }
    }

    // c) Continually re-check critical health bits in flight:
    if (!CPU_CheckTimingConstraints()) {
        FlightState_ClearHealth(FS_HEALTH_EKF_OK_BIT);
        Buzzer_PlayErrorRateLimited(TONE_ERROR_CPU, now);
    }

    // 16) Failsafe checks (only run if armed)
    if (FlightState_IsArmed()) {
        float navAlt;
        uint16_t rcLossSec;

        // a) Attitude limits
        if (Attitude_GetTiltAngle() > Settings_GetMaxTiltDeg() ||
            Attitude_GetAngularRate() > Settings_GetMaxRollRateDPS()) {
            Buzzer_PlayTone(TONE_WARN_ATTITUDE);
            CommenceAutoLand();
        }

        // b) Geofence (prefer baro altitude when healthy)
        navAlt = (Settings_GetBaroEnabled() && Baro_IsHealthy()) ?
                 Baro_GetAltitudeMeters() : GPS_GetAltitude();
        if (Geofence_Breached(GPS_GetLatitude(), GPS_GetLongitude(), navAlt,
                              Settings_GetGeofenceCenterLat(),
                              Settings_GetGeofenceCenterLon(),
                              Settings_GetGeofenceCenterAlt(),
                              Settings_GetGeofenceRadiusM(),
                              Settings_GetGeofenceAltMaxM())) {
            Buzzer_PlayTone(TONE_WARN_GEOFENCE);
            CommenceReturnToHome();
        }

        // c) RC/Telemetry link lost
        rcLossSec = (uint16_t)Settings_GetFSRCLossDelaySec();
        if (rcLossSec == 0U) {
            rcLossSec = 1U;
        }
        if (RC_LinkLostForSeconds(rcLossSec)) {
            Buzzer_PlayTone(TONE_WARN_RCLINK);
            CommenceAutoLand();  // or switch to Alt‐Hold
        }

        // d) EKF fails mid-flight
        if (!EKF_CovariancesConverged()) {
            Buzzer_PlayErrorRateLimited(TONE_ERROR_EKF, now);
            CommenceAutoLand();
        }

        // e) Overcurrent/thermal
        if (Battery_OverCurrent() || MCU_OverTemp() || ESC_OverTemp()) {
            Buzzer_PlayTone(TONE_WARN_THERMAL);
            CommenceAutoLand();
        }

        // f) Battery under load
        {
            float perCell = Battery_ReadPerCellVoltage();
            float remPerc = Battery_GetRemainingPercent();
            if (perCell < Settings_GetBatteryCriticalVolts()) {
                FlightState_ClearHealth(FS_HEALTH_BATT_OK_BIT);
                Buzzer_PlayTone(TONE_WARN_BATT_CRIT);
                CommenceAutoLand();
            } else if (remPerc < 5.0f) {
                Buzzer_PlayTone(TONE_WARN_BATT_CRIT);
                CommenceAutoLand();
            }
        }

        // g) If both GPS and baro are unavailable, hold mode fallback.
        if (!GPS_HasFix() && !(Settings_GetBaroEnabled() && Baro_IsHealthy())) {
            Buzzer_PlayErrorRateLimited(TONE_ERROR_GPS, now);
            CommenceAltHold();
        }
    }

    // Update servos using RC channels and gimbal mode
    Servo_Task(DebugMenu_GetActionMask());

    // 17) Telemetry & LED updates
    if (Settings_GetTelemetryEnabled()) {
        static uint32_t lastTelemetryMs = 0;
        uint16_t rateHz = Settings_GetTelemetryStreamRateHz();
        uint32_t intervalMs = (rateHz > 0U) ? (1000U / rateHz) : 0U;
        if (rateHz > 0U && intervalMs == 0U) {
            intervalMs = 1U;
        }

        if ((rateHz == 0U) || ((now - lastTelemetryMs) >= intervalMs)) {
            Telemetry_SendHealth(FlightState_GetStateMask());
            Telemetry_SendAttitude(Attitude_GetAngles());
            lastTelemetryMs = now;
        }
    }
    if (Settings_GetLEDEnabled()) {
        LED_UpdateStatus(FlightState_GetStateMask());
    }
    //DebugMsg("telemetry/LED done\r\n");

    // Process deferred settings writes outside IRQ context (rate-limited in settings.c).
    Settings_ProcessDeferredSave(now, !FlightState_IsArmed());

    // Sleep until next loop iteration (run at ~200 Hz for attitude, ~1 kHz for IMU)
    DebugMenu_Task();
    //DebugMsg("loop end\r\n");
    HAL_Delay(1);  // crude scheduler base tick (~1 kHz loop cadence).

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x009034B6;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 8;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 199;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 399;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 20000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 57600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TRIG1_Pin|TRIG2_Pin|TRIG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|HC_05_EN_Pin|MPU6050_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OSD_CS_GPIO_Port, OSD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(YAW_Buck_EN_GPIO_Port, YAW_Buck_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BMP388_EN_Pin|QMC5883_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARM_Buck_EN_Pin|PITCH_Buck_EN_Pin|ROL_Buck_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ECHO1_Pin ECHO2_Pin ECHO3_Pin */
  GPIO_InitStruct.Pin = ECHO1_Pin|ECHO2_Pin|ECHO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PPM_Pin */
  GPIO_InitStruct.Pin = PPM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PPM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG1_Pin TRIG2_Pin TRIG3_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin|TRIG2_Pin|TRIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 HC_05_EN_Pin MPU6050_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|HC_05_EN_Pin|MPU6050_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC_CD_Pin Motor_Swtich_1_Pin Motor_Switch_2_Pin */
  GPIO_InitStruct.Pin = SDMMC_CD_Pin|Motor_Swtich_1_Pin|Motor_Switch_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OSD_CS_Pin */
  GPIO_InitStruct.Pin = OSD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OSD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : YAW_Buck_EN_Pin */
  GPIO_InitStruct.Pin = YAW_Buck_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(YAW_Buck_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMP388_EN_Pin QMC5883_EN_Pin */
  GPIO_InitStruct.Pin = BMP388_EN_Pin|QMC5883_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARM_Buck_EN_Pin PITCH_Buck_EN_Pin ROL_Buck_EN_Pin */
  GPIO_InitStruct.Pin = ARM_Buck_EN_Pin|PITCH_Buck_EN_Pin|ROL_Buck_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ECHO1_EXTI_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ECHO1_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// RC‐PPM on PF9
	if (GPIO_Pin == GPIO_PIN_9) {
	    RC_Input_EXTI_Callback();
	    return;
	}

	// Sonar echos on PF6, PF7, PF8
	if (GPIO_Pin == GPIO_PIN_6) {
	    Sonar_EchoCallback(0, HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6));
	}
	else if (GPIO_Pin == GPIO_PIN_7) {
	    Sonar_EchoCallback(1, HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7));
	}
	else if (GPIO_Pin == GPIO_PIN_8) {
	    Sonar_EchoCallback(2, HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8));
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
