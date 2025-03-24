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
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "sdk_config.h"
#include "nrf_delay.h"
#include "avr/interrupt.h"

// BLE Setting
#define BLENAME "XIAO_SENSE_ACTIVITY_TRACKER"
#define SERVICE_UUID "4D7D1101-EE27-40B2-836C-17505C1044D7"
#define TX_PRED_CHAR_UUID "4D7D1108-EE27-40B2-836C-17505C1044D7"
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

// Flash storage settings
#define MAX_STRING_LENGTH   32
#define MAX_ENTRIES         2000
#define FLASH_HEADER_SIZE   8  // 4 bytes for entry count + 4 bytes for write position

typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  char data[MAX_STRING_LENGTH];
} LogEntry;

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C device address 0x6A

// Initialize BLE
BLEService bleService(SERVICE_UUID); // Bluetooth Low Energy LED Service
BLEStringCharacteristic txPredCharacteristic(TX_PRED_CHAR_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic rxCharacteristic(RX_CHAR_UUID, BLEWrite, 1024);

// Variable for QSPI
static uint32_t *QSPI_Status_Ptr = (uint32_t *)0x40029604; // Setup for the SEEED XIAO BLE - nRF52840
static nrfx_qspi_config_t QSPIConfig;
static nrf_qspi_cinstr_conf_t QSPICinstr_cfg;
static const uint32_t MemToUse = 64 * 1024; // Alter this to create larger read writes, 64Kb is the size of the Erase
static bool Debug_On = true;
static bool QSPIWait = false;

// Variable for normal mode
static bool receiving = false;
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

// Flash storage variables
static uint32_t               flash_header[2] = {0};  // [0] = entry count, [1] = write position
static LogEntry               current_entry;
static LogEntry               read_buffer[MAX_ENTRIES];

String pre_motion = "idle";
uint8_t dataByte = 0;
uint16_t step = 0;

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr)
{
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
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
  bleService.addCharacteristic(rxCharacteristic);

  // add service
  BLE.addService(bleService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  rxCharacteristic.setEventHandler(BLEWritten, rxCharacteristicWritten);

  // start advertising
  BLE.advertise();
  BLE.setAdvertisingInterval(20);
  BLE.setConnectionInterval(7.5, 7.5);
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
  if (myIMU.begin() != 0)
  {
    Serial.println("Device error");
    digitalWrite(LEDR, LOW);
  }
  else
  {
    Serial.println("Device OK!");
    if (0 != config_pedometer(NOT_CLEAR_STEP))
    {
      Serial.println("Configure pedometer fail!");
    }
    Serial.println("Success to Configure pedometer!");
  }
}
 
static void QSPI_Status(char ASender[]) { // Prints the QSPI Status
  Serial.print("(");
  Serial.print(ASender);
  Serial.print(") QSPI is busy/idle ... Result = ");
  Serial.println(nrfx_qspi_mem_busy_check() & 8);
  Serial.print("(");
  Serial.print(ASender);
  Serial.print(") QSPI Status flag = 0x");
  Serial.print(NRF_QSPI->STATUS, HEX);
  Serial.print(" (from NRF_QSPI) or 0x");
  Serial.print(*QSPI_Status_Ptr, HEX);
  Serial.println(" (from *QSPI_Status_Ptr)");
}

static nrfx_err_t QSPI_IsReady() {
  if (((*QSPI_Status_Ptr & 8) == 8) && (*QSPI_Status_Ptr & 0x01000000) == 0) {
    return NRFX_SUCCESS;  
  } else {
   return NRFX_ERROR_BUSY;
  }
}
 
static nrfx_err_t QSPI_WaitForReady() {
  while (QSPI_IsReady() == NRFX_ERROR_BUSY) {
    if (Debug_On) {
      Serial.print("*QSPI_Status_Ptr & 8 = ");
      Serial.print(*QSPI_Status_Ptr & 8);
      Serial.print(", *QSPI_Status_Ptr & 0x01000000 = 0x");
      Serial.println(*QSPI_Status_Ptr & 0x01000000, HEX);
      QSPI_Status("QSPI_WaitForReady");
    }  
  }
  return NRFX_SUCCESS;
}

static void QSIP_Configure_Memory() {
  uint8_t  temporary[] = {0x00, 0x02};
  uint32_t Error_Code;
 
  QSPICinstr_cfg = {
    .opcode    = QSPI_STD_CMD_RSTEN,
    .length    = NRF_QSPI_CINSTR_LEN_1B,
    .io2_level = true,
    .io3_level = true,
    .wipwait   = QSPIWait,
    .wren      = true
  };
  QSPI_WaitForReady();
  if (nrfx_qspi_cinstr_xfer(&QSPICinstr_cfg, NULL, NULL) != NRFX_SUCCESS) { // Send reset enable
    if (Debug_On) {
      Serial.println("(QSIP_Configure_Memory) QSPI 'Send reset enable' failed!");
    }
  } else {
    QSPICinstr_cfg.opcode = QSPI_STD_CMD_RST;
    QSPI_WaitForReady();
    if (nrfx_qspi_cinstr_xfer(&QSPICinstr_cfg, NULL, NULL) != NRFX_SUCCESS) { // Send reset command
      if (Debug_On) {
        Serial.println("(QSIP_Configure_Memory) QSPI Reset failed!");
      }
    } else {
      QSPICinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
      QSPICinstr_cfg.length = NRF_QSPI_CINSTR_LEN_3B;
      QSPI_WaitForReady();
      if (nrfx_qspi_cinstr_xfer(&QSPICinstr_cfg, &temporary, NULL) != NRFX_SUCCESS) { // Switch to qspi mode
        if (Debug_On) {
          Serial.println("(QSIP_Configure_Memory) QSPI failed to switch to QSPI mode!");
        }
      } else {
          QSPI_Status("QSIP_Configure_Memory");
      }
    }
  }
}
 
static nrfx_err_t QSPI_Initialise() { // Initialises the QSPI and NRF LOG
  uint32_t Error_Code;
 
  NRF_LOG_INIT(NULL); // Initialise the NRF Log
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  // QSPI Config
  QSPIConfig.xip_offset = NRFX_QSPI_CONFIG_XIP_OFFSET;                      
  QSPIConfig.pins = { // Setup for the SEEED XIAO BLE - nRF52840                                                    
   .sck_pin     = 21,                                
   .csn_pin     = 25,                                
   .io0_pin     = 20,                                
   .io1_pin     = 24,                                
   .io2_pin     = 22,                                
   .io3_pin     = 23,                                
  };                                                                  
  QSPIConfig.irq_priority = (uint8_t)NRFX_QSPI_CONFIG_IRQ_PRIORITY;          
  QSPIConfig.prot_if = {                                                        
    .readoc     = (nrf_qspi_readoc_t)NRF_QSPI_READOC_READ4O,      
    .writeoc    = (nrf_qspi_writeoc_t)NRF_QSPI_WRITEOC_PP4O,
    .addrmode   = (nrf_qspi_addrmode_t)NRFX_QSPI_CONFIG_ADDRMODE,  
    .dpmconfig  = false,                                            
  };                  
  QSPIConfig.phy_if.sck_freq   = (nrf_qspi_frequency_t)NRF_QSPI_FREQ_32MDIV1;                                        
  QSPIConfig.phy_if.spi_mode   = (nrf_qspi_spi_mode_t)NRFX_QSPI_CONFIG_MODE;
  QSPIConfig.phy_if.dpmen      = false;
  // QSPI Config Complete
  // Setup QSPI to allow for DPM but with it turned off
  QSPIConfig.prot_if.dpmconfig = true;
  NRF_QSPI->DPMDUR = (QSPI_DPM_ENTER << 16) | QSPI_DPM_EXIT; // Found this on the Nordic Q&A pages, Sets the Deep power-down mode timer
  Error_Code = 1;
  while (Error_Code != 0) {
    Error_Code = nrfx_qspi_init(&QSPIConfig, NULL, NULL);
    if (Error_Code != NRFX_SUCCESS) {
      if (Debug_On) {
        Serial.print("(QSPI_Initialise) nrfx_qspi_init returned : ");
        Serial.println(Error_Code);
      }
    } else {
      if (Debug_On) {
        Serial.println("(QSPI_Initialise) nrfx_qspi_init successful");
      }
    }
  }
  QSPI_Status("QSPI_Initialise (Before QSIP_Configure_Memory)");
  QSIP_Configure_Memory();
  if (Debug_On) {
    Serial.println("(QSPI_Initialise) Wait for QSPI to be ready ...");
  }
  NRF_QSPI->TASKS_ACTIVATE = 1;
  QSPI_WaitForReady();
  if (Debug_On) {
    Serial.println("(QSPI_Initialise) QSPI is ready");
  }
  return QSPI_IsReady();
}
 
static void QSPI_Erase(uint32_t AStartAddress) {
  uint32_t   TimeTaken;
  bool       QSPIReady = false;
  bool       AlreadyPrinted = false;
 
  if (Debug_On) {
    Serial.println("(QSPI_Erase) Erasing memory");
  }
  while (!QSPIReady) {
    if (QSPI_IsReady() != NRFX_SUCCESS) {
      if (!AlreadyPrinted) {
        QSPI_Status("QSPI_Erase (Waiting)");
        AlreadyPrinted = true;
      }
    } else {
      QSPIReady = true;
      QSPI_Status("QSPI_Erase (Waiting Loop Breakout)");
    }
  }
  if (Debug_On) {
    QSPI_Status("QSPI_Erase (Finished Waiting)");
    TimeTaken = millis();
  }
  if (nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, AStartAddress) != NRFX_SUCCESS) {
    if (Debug_On) {
      Serial.print("(QSPI_Initialise_Page) QSPI Address 0x");
      Serial.print(AStartAddress, HEX);
      Serial.println(" failed to erase!");
    }
  } else {    
    if (Debug_On) {
      TimeTaken = millis() - TimeTaken;
      Serial.print("(QSPI_Initialise_Page) QSPI took ");
      Serial.print(TimeTaken);
      Serial.println("ms to erase a 64Kb page");
    }
  }
  flash_header[0] = 0;
  flash_header[1] = 0;
}
 
// Initialize log storage by reading header from flash
static void initLogStorage() {
  QSPI_WaitForReady();
 
  // Read header
  if (nrfx_qspi_read(flash_header, FLASH_HEADER_SIZE, 0) != NRFX_SUCCESS) {
    Serial.println("Failed to read flash header");
    flash_header[0] = 0;  // entry count
    flash_header[1] = 0;  // write position
   
    // Write initialized header back to flash
    QSPI_WaitForReady();
    nrfx_qspi_write(flash_header, FLASH_HEADER_SIZE, 0);
  }
 
  Serial.print("Log storage initialized: entries=");
  Serial.print(flash_header[0]);
  Serial.print(", position=");
  Serial.println(flash_header[1]);
  
  // Print struct size for debugging
  Serial.print("Size of LogEntry: ");
  Serial.println(sizeof(LogEntry));
  Serial.print("Expected size: ");
  Serial.println(4 + MAX_STRING_LENGTH); // timestamp + data
}
 
// Write a string entry to flash
static void writeStringToFlash(const char* str) {
  memset(&current_entry, 0, sizeof(LogEntry));
  current_entry.timestamp = millis();
  strncpy(current_entry.data, str, MAX_STRING_LENGTH - 1);
  current_entry.data[MAX_STRING_LENGTH - 1] = '\0'; // Ensure null termination
 
  // Calculate position to write
  uint32_t position = FLASH_HEADER_SIZE + (flash_header[1] * sizeof(LogEntry));
 
  // Write the entry
  QSPI_WaitForReady();
  if (nrfx_qspi_write(&current_entry, sizeof(LogEntry), position) != NRFX_SUCCESS) {
    Serial.println("Failed to write log entry");
    return;
  }
 
  // Update header
  flash_header[0] = min(flash_header[0] + 1, MAX_ENTRIES);
  flash_header[1] = (flash_header[1] + 1) % MAX_ENTRIES;
 
  QSPI_WaitForReady();
  if (nrfx_qspi_write(flash_header, FLASH_HEADER_SIZE, 0) != NRFX_SUCCESS) {
    Serial.println("Failed to update header");
  }

  Serial.print("Wrote string to flash: ");
  Serial.println(str);
}
 
// Read all entries from flash and output to Serial
static void readAllEntries() {
  uint32_t count = flash_header[0];
  uint32_t start_pos;
 
  if (count == 0) {
    Serial.println("No entries found");
    return;
  }
 
  if (count < MAX_ENTRIES) {
    start_pos = 0;
  } else {
    // Circular buffer is full, start from oldest entry
    start_pos = flash_header[1];
  }
 
  Serial.print("Reading ");
  Serial.print(count);
  Serial.println(" entries:");
 
  // Clear buffer first
  memset(read_buffer, 0, sizeof(read_buffer));
 
  // Read all entries
  QSPI_WaitForReady();
  for (uint32_t i = 0; i < count; i++) {
    uint32_t idx = (start_pos + i) % MAX_ENTRIES;
    uint32_t position = FLASH_HEADER_SIZE + (idx * sizeof(LogEntry));
   
    if (nrfx_qspi_read(&read_buffer[i], sizeof(LogEntry), position) != NRFX_SUCCESS) {
      Serial.print("Failed to read entry at position ");
      Serial.println(position);
      continue;
    }
    
    // Print entry more carefully
    Serial.print(i);
    
    // Explicitly print each character
    for (int j = 0; j < MAX_STRING_LENGTH && read_buffer[i].data[j] != '\0'; j++) {
      Serial.write(read_buffer[i].data[j]);
    }
    Serial.println();
    txPredCharacteristic.writeValue(read_buffer[i].data);
  }
  Serial.println("End of entries");
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

  // Initialize BLE
  set_up_BLE();
  // Initialize the IMU sensor
  set_up_IMU();

  QSPI_Initialise();

  initLogStorage();
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
  snprintf(motionBuffer, MAX_STRING_LENGTH - 1, "%lu %s", currenttime, currentMotion.c_str());

  if (central){
    txPredCharacteristic.writeValue(motionBuffer);
    if (flash_header[0] != 0){
      readAllEntries();
      QSPI_Erase(0);
    }
  }
  else {
    writeStringToFlash(motionBuffer);
  }
}