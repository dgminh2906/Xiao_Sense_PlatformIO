#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include "Motion-detection-with-XIAO-Sense_inferencing.h"

#define BLENAME "XIAO_SENSE_ACTIVITY_TRACKER"
#define SERVICE_UUID "4D7D1101-EE27-40B2-836C-17505C1044D7"

#define TX_ACC_X_UUID "4D7D1102-EE27-40B2-836C-17505C1044D7"
#define TX_ACC_Y_UUID "4D7D1103-EE27-40B2-836C-17505C1044D7"
#define TX_ACC_Z_UUID "4D7D1104-EE27-40B2-836C-17505C1044D7"
#define TX_GYRO_X_UUID "4D7D1105-EE27-40B2-836C-17505C1044D7"
#define TX_GYRO_Y_UUID "4D7D1106-EE27-40B2-836C-17505C1044D7"
#define TX_GYRO_Z_UUID "4D7D1107-EE27-40B2-836C-17505C1044D7"

#define TX_PRED_CHAR_UUID "4D7D1108-EE27-40B2-836C-17505C1044D7"
// #define TX_BAT_CHAR_UUID "4D7D1109-EE27-40B2-836C-17505C1044D7"
#define RX_CHAR_UUID "4D7D1110-EE27-40B2-836C-17505C1044D7"

#define ACCELERATION_DUE_TO_GRAVITY 9.81f
#define GYRO_ANGLE_TO_RADIAN 3.141f / 180.0f

#define CLEAR_STEP true
#define NOT_CLEAR_STEP false

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C device address 0x6A

BLEService bleService(SERVICE_UUID); // Bluetooth Low Energy LED Service

BLEStringCharacteristic txAccXCharacteristic(TX_ACC_X_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txAccYCharacteristic(TX_ACC_Y_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txAccZCharacteristic(TX_ACC_Z_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txGyroXCharacteristic(TX_GYRO_X_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txGyroYCharacteristic(TX_GYRO_Y_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txGyroZCharacteristic(TX_GYRO_Z_UUID, BLERead | BLENotify, 1024);

BLEStringCharacteristic txPredCharacteristic(TX_PRED_CHAR_UUID, BLERead | BLENotify, 1024);
// BLEStringCharacteristic txBatCharacteristic(TX_BAT_CHAR_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic rxCharacteristic(RX_CHAR_UUID, BLEWrite, 1024);

static bool receiving = false;
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
String motion = "rest";
String pre_motion = "rest";
uint8_t dataByte = 0;
uint16_t stepCount = 0;

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
    motion = "";
  }
}

void CollectData()
{
  for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 6)
  {
    features[i] = myIMU.readFloatAccelX() * ACCELERATION_DUE_TO_GRAVITY;
    features[i + 1] = myIMU.readFloatAccelY() * ACCELERATION_DUE_TO_GRAVITY;
    features[i + 2] = myIMU.readFloatAccelZ() * ACCELERATION_DUE_TO_GRAVITY;
    features[i + 3] = myIMU.readFloatGyroX() * GYRO_ANGLE_TO_RADIAN;
    features[i + 4] = myIMU.readFloatGyroY() * GYRO_ANGLE_TO_RADIAN;
    features[i + 5] = myIMU.readFloatGyroZ() * GYRO_ANGLE_TO_RADIAN;

    // // Send IMU data
    // if (central)
    // {
    //   // ei_printf("######  Writing IMU to BLE \n");
    //   txAccXCharacteristic.writeValue(String(features[i]));
    //   txAccYCharacteristic.writeValue(String(features[i + 1]));
    //   txAccZCharacteristic.writeValue(String(features[i + 2]));
    //   txGyroXCharacteristic.writeValue(String(features[i + 3]));
    //   txGyroYCharacteristic.writeValue(String(features[i + 4]));
    //   txGyroZCharacteristic.writeValue(String(features[i + 5]));
    // }

    delay(EI_CLASSIFIER_INTERVAL_MS);
  }
}

void RunDetection()
{
  // Run the classifier
  ei_impulse_result_t result = {0};

  signal_t features_signal;
  features_signal.total_length = sizeof(features) / sizeof(features[0]);
  features_signal.get_data = &raw_feature_get_data;

  // Invoke the impulse
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
  if (res != EI_IMPULSE_OK)
    return;

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
  // motion = motion + " " + label;
  if (score > 0.7)
  {
    motion = label;
    pre_motion = motion;
  }
  else
  {
    motion = pre_motion;
  }
}

void setupBLE()
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

  // add the characteristic to the service
  bleService.addCharacteristic(txAccXCharacteristic);
  bleService.addCharacteristic(txAccYCharacteristic);
  bleService.addCharacteristic(txAccZCharacteristic);
  bleService.addCharacteristic(txGyroXCharacteristic);
  bleService.addCharacteristic(txGyroYCharacteristic);
  bleService.addCharacteristic(txGyroZCharacteristic);

  bleService.addCharacteristic(txPredCharacteristic);
  // bleService.addCharacteristic(txBatCharacteristic);
  // bleService.addCharacteristic(rxCharacteristic);

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

void StepCount()
{
  myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  stepCount = (dataByte << 8) & 0xFFFF;

  myIMU.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  stepCount |= dataByte;
}

int config_pedometer(bool clearStep)
{
  uint8_t errorAccumulator = 0;
  uint8_t dataToWrite = 0; // Temporary variable

  // Setup the accelerometer******************************
  dataToWrite = 0;

  //  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
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

  // Step 3:  Enable pedometer algorithm
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

  // Step 4: Step Detector interrupt driven to INT1 pin, set bit INT1_FIFO_OVR
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);

  return errorAccumulator;
}

void setupIMU()
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
  setupBLE();
  // Initialize the IMU sensor
  setupIMU();
}

void loop()
{
  BLEDevice central = BLE.central();
  CollectData();
  RunDetection();
  StepCount();
  Serial.println(motion);
  Serial.println(stepCount);
  // Send data
  if (central)
  {
    txPredCharacteristic.writeValue(motion.c_str());
    if (receiving == true)
    {
      motion = "";
    }
  }
}