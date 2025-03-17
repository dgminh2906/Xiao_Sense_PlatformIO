#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include "Motion-detection-with-XIAO-Sense_inferencing.h"
#include "nrf52840.h"
// Library for using QSPI
#include "nrfx_qspi.h"
#include "app_util_platform.h"

// BLE Setting
#define BLENAME "XIAO_SENSE_ACTIVITY_TRACKER"
#define SERVICE_UUID "4D7D1101-EE27-40B2-836C-17505C1044D7"
#define TX_PRED_CHAR_UUID "4D7D1108-EE27-40B2-836C-17505C1044D7"
#define TX_STEP_CHAR_UUID "4D7D1109-EE27-40B2-836C-17505C1044D7"
#define RX_CHAR_UUID "4D7D1110-EE27-40B2-836C-17505C1044D7"

// Macro for IMU data
#define ACCELERATION_DUE_TO_GRAVITY 9.81f
#define GYRO_ANGLE_TO_RADIAN 3.141f / 180.0f

// Macro for step counting
#define CLEAR_STEP true
#define NOT_CLEAR_STEP false

// Macro for using QSPI
#define QSPI_STD_CMD_WRSR 0x01
#define QSPI_STD_CMD_RSTEN 0x66
#define QSPI_STD_CMD_RST 0x99
#define QSPI_DPM_ENTER 0x0003 // 3 x 256 x 62.5ns = 48ms
#define QSPI_DPM_EXIT 0x0003

// Macro for motion data
#define MAX_MOTION_ENTRIES 100
#define MAX_STRING_LENGTH 128

#define LED_RED_PORT 0
#define LED_RED_PIN 26
#define IMU_INT1_PORT 0
#define IMU_INT1_PIN 11
uint32_t lsm6ds3_interrupt_count = 0;

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C device address 0x6A

// Initialize BLE
BLEService bleService(SERVICE_UUID); // Bluetooth Low Energy LED Service
BLEStringCharacteristic txPredCharacteristic(TX_PRED_CHAR_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txStepCharacteristic(TX_STEP_CHAR_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic rxCharacteristic(RX_CHAR_UUID, BLEWrite, 1024);

// Variable for QSPI
static uint32_t *QSPI_Status_Ptr = (uint32_t *)0x40029604; // Setup for the SEEED XIAO BLE - nRF52840
static nrfx_qspi_config_t QSPIConfig;
static nrf_qspi_cinstr_conf_t QSPICinstr_cfg;
static const uint32_t MemToUse = 64 * 1024; // Alter this to create larger read writes, 64Kb is the size of the Erase
static bool QSPIWait = false;

// Variable for normal mode
static bool receiving = false;
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

// Motion data storage using array of strings
char **motionData = NULL;                  // Pointer to array of strings
int motionCount = 0;                       // Number of motion entries
int maxMotionEntries = MAX_MOTION_ENTRIES; // Maximum number of entries

String pre_motion = "idle";
uint8_t dataByte = 0;
uint16_t step = 0;

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr)
{
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

// Clear motion data
void clearMotionData()
{
  if (motionData != NULL)
  {
    for (int i = 0; i < motionCount; i++)
    {
      if (motionData[i] != NULL)
      {
        free(motionData[i]);
        motionData[i] = NULL;
      }
    }
    motionCount = 0;
  }
}

// Initialize motion data storage
void initMotionData()
{
  motionData = (char **)malloc(maxMotionEntries * sizeof(char *));
  for (int i = 0; i < maxMotionEntries; i++)
  {
    motionData[i] = NULL;
  }
  motionCount = 0;
}

// Add a motion entry
void addMotionEntry(const char *motion)
{
  if (motionCount >= maxMotionEntries)
  {
    // If array is full, remove the oldest entry
    free(motionData[0]);
    for (int i = 0; i < maxMotionEntries - 1; i++)
    {
      motionData[i] = motionData[i + 1];
    }
    motionCount--;
  }

  // Add new entry
  motionData[motionCount] = (char *)malloc(MAX_STRING_LENGTH * sizeof(char));
  if (motionData[motionCount] != NULL)
  {
    strncpy(motionData[motionCount], motion, MAX_STRING_LENGTH - 1);
    motionData[motionCount][MAX_STRING_LENGTH - 1] = '\0'; // Ensure null termination
    motionCount++;
  }
}

void blePeripheralConnectHandler(BLEDevice central)
{
  // central connected event handler
  Serial.print("Connected event, central: ");
  receiving = true;
  Serial.println(central.address());
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDR, HIGH);
}

void blePeripheralDisconnectHandler(BLEDevice central)
{
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  receiving = false;
  Serial.println(central.address());
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDR, LOW);
}

void rxCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic)
{
  // central wrote new value to characteristic, update LED
  String value = rxCharacteristic.value();
  if (value == "r")
  {
    receiving = true;
  }
}

void collect_data()
{
  for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 6)
  {
    features[i] = myIMU.readFloatAccelX() * ACCELERATION_DUE_TO_GRAVITY;
    features[i + 1] = myIMU.readFloatAccelY() * ACCELERATION_DUE_TO_GRAVITY;
    features[i + 2] = myIMU.readFloatAccelZ() * ACCELERATION_DUE_TO_GRAVITY;
    features[i + 3] = myIMU.readFloatGyroX() * GYRO_ANGLE_TO_RADIAN;
    features[i + 4] = myIMU.readFloatGyroY() * GYRO_ANGLE_TO_RADIAN;
    features[i + 5] = myIMU.readFloatGyroZ() * GYRO_ANGLE_TO_RADIAN;

    delay(EI_CLASSIFIER_INTERVAL_MS);
  }
}

String run_detection()
{
  // Run the classifier
  ei_impulse_result_t result = {0};

  signal_t features_signal;
  features_signal.total_length = sizeof(features) / sizeof(features[0]);
  features_signal.get_data = &raw_feature_get_data;

  // Invoke the impulse
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
  if (res != EI_IMPULSE_OK)
    return pre_motion;

  float score = 0;
  String label = "";
  // Get result after classifier
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
  {
    if (result.classification[ix].value > score)
    {
      score = result.classification[ix].value;
      label = result.classification[ix].label;
    }
  }

  if (score > 0.7)
  {
    pre_motion = label;
  }

  return pre_motion;
}

void set_up_BLE()
{
  if (!BLE.begin())
  {
    Serial.println("Starting BLE failed!");
    while (1)
      ;
  }
  // Set BLE device name and advertise IMU service
  BLE.setDeviceName(BLENAME);
  BLE.setLocalName(BLENAME);
  BLE.setAdvertisedService(bleService);

  bleService.addCharacteristic(txPredCharacteristic);
  bleService.addCharacteristic(txStepCharacteristic);
  bleService.addCharacteristic(rxCharacteristic);

  // add service
  BLE.addService(bleService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  rxCharacteristic.setEventHandler(BLEWritten, rxCharacteristicWritten);

  // start advertising
  BLE.setAdvertisingInterval(160);    // 0.625mS*160=100mS
  BLE.setConnectionInterval(6, 3200); // 1.25mS*6=7.5mS, 1.25mS*3200=4S
  BLE.advertise();
}

void step_count()
{
  myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  step = (dataByte << 8) & 0xFFFF;

  myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  step |= dataByte;
}

int config_pedometer(bool clearStep)
{
  uint8_t errorAccumulator = 0;
  uint8_t dataToWrite = 0; // Temporary variable

  // Setup the accelerometer******************************
  dataToWrite = 0;

  // dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;

  // Step 1: Configure ODR-26Hz and FS-2g
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Step 2: Set bit Zen_G, Yen_G, Xen_G, FUNC_EN, PEDO_RST_STEP(1 or 0)
  if (clearStep)
  {
    errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
  }
  else
  {
    errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);
  }

  // Step 3: Enable pedometer algorithm
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

  // Step 4: Step Detector interrupt driven to INT1 pin, set bit INT1_FIFO_OVR
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);

  return errorAccumulator;
}

void set_up_IMU()
{
  /* Disable gyro and temp on IMU sensor */
  myIMU.settings.gyroEnabled = 0;
  myIMU.settings.tempEnabled = 0;

  if (myIMU.begin() != 0)
  {
    NRF_GPIOTE->CONFIG[0] = 0x3 | (LED_RED_PIN << 8) | (LED_RED_PORT << 13);
  }
  else
  {
    myIMU.writeRegister(0x12, 0x04); // CTRL3_C: Active Low
    myIMU.writeRegister(0x0D, 0x01); // INT1_CTRL: Enable interrupt on INT1

    // /* Config INPUT falling edge for IMU_INT1 */
    NRF_GPIOTE->CONFIG[1] = 0x1 | (IMU_INT1_PIN << 8) | (IMU_INT1_PORT << 13) | (0x1 << 16);
    // /* Enable INTERRUPT for IMU_INT1 */
    NRF_GPIOTE->INTENSET = 1 << 1;

    NVIC_SetPriority(GPIOTE_IRQn, 15); // Lowes priority
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    config_pedometer(NOT_CLEAR_STEP);
  }
}

static nrfx_err_t QSPI_IsReady()
{
  if (((*QSPI_Status_Ptr & 8) == 8) && (*QSPI_Status_Ptr & 0x01000000) == 0)
  {
    return NRFX_SUCCESS;
  }
  else
  {
    return NRFX_ERROR_BUSY;
  }
}

static nrfx_err_t QSPI_WaitForReady()
{
  while (QSPI_IsReady() == NRFX_ERROR_BUSY)
  {
    // Just wait until ready
  }
  return NRFX_SUCCESS;
}

static void QSIP_Configure_Memory()
{
  uint8_t temporary[] = {0x00, 0x02};

  QSPICinstr_cfg = {
      .opcode = QSPI_STD_CMD_RSTEN,
      .length = NRF_QSPI_CINSTR_LEN_1B,
      .io2_level = true,
      .io3_level = true,
      .wipwait = QSPIWait,
      .wren = true};
  QSPI_WaitForReady();
  nrfx_qspi_cinstr_xfer(&QSPICinstr_cfg, NULL, NULL); // Send reset enable

  QSPICinstr_cfg.opcode = QSPI_STD_CMD_RST;
  QSPI_WaitForReady();
  nrfx_qspi_cinstr_xfer(&QSPICinstr_cfg, NULL, NULL); // Send reset command

  QSPICinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
  QSPICinstr_cfg.length = NRF_QSPI_CINSTR_LEN_3B;
  QSPI_WaitForReady();
  nrfx_qspi_cinstr_xfer(&QSPICinstr_cfg, &temporary, NULL); // Switch to qspi mode
}

static nrfx_err_t QSPI_Initialise()
{ // Initialises the QSPI and NRF LOG
  uint32_t Error_Code;
  // QSPI Config
  QSPIConfig.xip_offset = NRFX_QSPI_CONFIG_XIP_OFFSET;
  QSPIConfig.pins = {
      // Setup for the SEEED XIAO BLE - nRF52840
      .sck_pin = 21,
      .csn_pin = 25,
      .io0_pin = 20,
      .io1_pin = 24,
      .io2_pin = 22,
      .io3_pin = 23,
  };
  QSPIConfig.irq_priority = (uint8_t)NRFX_QSPI_CONFIG_IRQ_PRIORITY;
  QSPIConfig.prot_if = {
      .readoc = (nrf_qspi_readoc_t)NRF_QSPI_READOC_READ4O,
      .writeoc = (nrf_qspi_writeoc_t)NRF_QSPI_WRITEOC_PP4O,
      .addrmode = (nrf_qspi_addrmode_t)NRFX_QSPI_CONFIG_ADDRMODE,
      .dpmconfig = false,
  };
  QSPIConfig.phy_if.sck_freq = (nrf_qspi_frequency_t)NRF_QSPI_FREQ_32MDIV1;
  QSPIConfig.phy_if.spi_mode = (nrf_qspi_spi_mode_t)NRFX_QSPI_CONFIG_MODE;
  QSPIConfig.phy_if.dpmen = false;
  // QSPI Config Complete
  // Setup QSPI to allow for DPM but with it turned off
  QSPIConfig.prot_if.dpmconfig = true;
  NRF_QSPI->DPMDUR = (QSPI_DPM_ENTER << 16) | QSPI_DPM_EXIT; // Sets the Deep power-down mode timer
  Error_Code = 1;
  while (Error_Code != 0)
  {
    Error_Code = nrfx_qspi_init(&QSPIConfig, NULL, NULL);
  }
  QSIP_Configure_Memory();
  NRF_QSPI->TASKS_ACTIVATE = 1;
  QSPI_WaitForReady();
  return QSPI_IsReady();
}

static void QSPI_Erase(uint32_t AStartAddress)
{
  bool QSPIReady = false;

  while (!QSPIReady)
  {
    if (QSPI_IsReady() != NRFX_SUCCESS)
    {
      // Just wait
    }
    else
    {
      QSPIReady = true;
    }
  }
  nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, AStartAddress);
}

void GPIOTE_IRQHandler()
{
  if (NRF_GPIOTE->EVENTS_IN[1])
  {
    NRF_GPIOTE->EVENTS_IN[1] = 0;
    lsm6ds3_interrupt_count++;
  }
}

void setup()
{
  // Serial for debugging
  Serial.begin(9600);

  // set LED pin to output mode
  pinMode(P0_14, OUTPUT);
  digitalWrite(P0_14, LOW);
  pinMode(LEDB, OUTPUT);
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, HIGH);

  // Initialize motion data storage
  initMotionData();

  // Initialize BLE
  set_up_BLE();
  // Initialize the IMU sensor
  set_up_IMU();

  QSPI_Initialise();
}

void loop()
{
  BLEDevice central = BLE.central();
  collect_data();

  // Run detection and get current motion
  String currentMotion = run_detection();

  step_count();
  unsigned long currenttime = millis();

  // Create motion string with timestamp and current motion
  char motionBuffer[MAX_STRING_LENGTH];
  snprintf(motionBuffer, MAX_STRING_LENGTH, "%lu %s", currenttime, currentMotion.c_str());

  // Add to our motion data array
  addMotionEntry(motionBuffer);

  // Print current motion
  Serial.println(motionBuffer);
  Serial.println(step);

  if (central)
  {
    // Send the latest motion data over BLE
    if (motionCount > 0)
    {
      for (int i = 0; i < motionCount; i++)
      {
        txPredCharacteristic.writeValue(motionData[i]);
      }
    }

    if (receiving == true)
    {
      clearMotionData();
      QSPI_Erase(0);
    }
  }
  else
  {
    // Write data to QSPI Flash memory if there's data to write
    if (motionCount > 0)
    {
      QSPI_WaitForReady();
      // Write the latest motion data entry
      nrfx_qspi_write(motionData, MemToUse, 0x0);
    }
  }
}

// void freeMotionData()
// {
//   if (motionData != NULL)
//   {
//     // Free all allocated strings
//     for (int i = 0; i < motionCount; i++)
//     {
//       if (motionData[i] != NULL)
//       {
//         free(motionData[i]);
//       }
//     }
//     // Free the array itself
//     free(motionData);
//     motionData = NULL;
//   }
// }