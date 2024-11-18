#include <Arduino.h>
#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include "Motion-detection-with-XIAO-Sense_inferencing.h"

#define BLENAME "XIAO_SENSE"
#define SERVICE_UUID "4D7D1101-EE27-40B2-836C-17505C1044D7"

#define ACC_X_UUID "4D7D1102-EE27-40B2-836C-17505C1044D7"
#define ACC_Y_UUID "4D7D1103-EE27-40B2-836C-17505C1044D7"
#define ACC_Z_UUID "4D7D1104-EE27-40B2-836C-17505C1044D7"
#define GYRO_X_UUID "4D7D1105-EE27-40B2-836C-17505C1044D7"
#define GYRO_Y_UUID "4D7D1106-EE27-40B2-836C-17505C1044D7"
#define GYRO_Z_UUID "4D7D1107-EE27-40B2-836C-17505C1044D7"

#define TX_PRED_CHAR_UUID "4D7D1108-EE27-40B2-836C-17505C1044D7"
#define TX_BAT_CHAR_UUID "4D7D1109-EE27-40B2-836C-17505C1044D7"
// #define RX_CHAR_UUID "4D7D1110-EE27-40B2-836C-17505C1044D7"

#define ACCELERATION_DUE_TO_GRAVITY 9.81f
#define GYRO_ANGLE_TO_RADIAN 3.141f / 180.0f

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C device address 0x6A

BLEService bleService(SERVICE_UUID); // Bluetooth Low Energy LED Service

BLEStringCharacteristic txAccXCharacteristic(ACC_X_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txAccYCharacteristic(ACC_Y_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txAccZCharacteristic(ACC_Z_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txGyroXCharacteristic(GYRO_X_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txGyroYCharacteristic(GYRO_Y_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txGyroZCharacteristic(GYRO_Z_UUID, BLERead | BLENotify, 1024);

// BLEStringCharacteristic rxCharacteristic(RX_CHAR_UUID, BLEWrite, 1024);
BLEStringCharacteristic txPredCharacteristic(TX_PRED_CHAR_UUID, BLERead | BLENotify, 1024);
BLEStringCharacteristic txBatCharacteristic(TX_BAT_CHAR_UUID, BLERead | BLENotify, 1024);

static bool receiving = false;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
String global_data;

#define PSIZE 5
String predictions[PSIZE];
int pCounter = 0;

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr)
{
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

String getFinalPrediction()
{
  int rest = 0;
  int walk = 0;
  int run = 0;
  int goingstair = 0;
  int unknown = 0;

  String finalLabel = "1";

  for (int i = 0; i < PSIZE; i++)
  {

    String label = predictions[i];

    if (label == "1")
    {
      rest++;
    }
    else if (label == "2")
    {
      walk++;
    }
    else if (label == "3")
    {
      run++;
    }
    else if (label == "4")
    {
      goingstair++;
    }
    else
    {
      unknown++;
    }
  }

  if (goingstair > PSIZE / 3)
  {
    finalLabel = "goingstair";
  }
  else if (run > PSIZE / 3)
  {
    finalLabel = "running";
  }
  else if (walk > PSIZE / 3)
  {
    finalLabel = "walking";
  }
  else
  {
    finalLabel = "rest";
  }

  ei_printf("REST = %d, WALK = %d, RUN = %d, GOINGSTAIR = %d \n", rest, walk, run, goingstair);

  return finalLabel;
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

// void rxCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic)
// {
//   // central wrote new value to characteristic, update LED
//   String value = rxCharacteristic.value();
//   if (value == "r")
//   {
//     receiving = true;
//     global_data = "";
//   }
// }

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
  bleService.addCharacteristic(txBatCharacteristic);
  bleService.addCharacteristic(txPredCharacteristic);
  // bleService.addCharacteristic(rxCharacteristic);
  // add service
  BLE.addService(bleService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  // rxCharacteristic.setEventHandler(BLEWritten, rxCharacteristicWritten);

  // start advertising
  BLE.advertise();
  // Initialize the IMU sensor
  if (myIMU.begin() != 0)
  {
    Serial.println("Device error");
    digitalWrite(LEDR, LOW);
  }
  else
  {
    Serial.println("Device OK!");
  }
}

void loop()
{
  BLEDevice central = BLE.central();

  // Collect data
  for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 6)
  {
    features[ix] = myIMU.readFloatAccelX() * ACCELERATION_DUE_TO_GRAVITY;
    features[ix + 1] = myIMU.readFloatAccelY() * ACCELERATION_DUE_TO_GRAVITY;
    features[ix + 2] = myIMU.readFloatAccelZ() * ACCELERATION_DUE_TO_GRAVITY;
    features[ix + 3] = myIMU.readFloatGyroX() * GYRO_ANGLE_TO_RADIAN;
    features[ix + 4] = myIMU.readFloatGyroY() * GYRO_ANGLE_TO_RADIAN;
    features[ix + 5] = myIMU.readFloatGyroZ() * GYRO_ANGLE_TO_RADIAN;

    txAccXCharacteristic.writeValue(String(features[ix], 4));
    txAccYCharacteristic.writeValue(String(features[ix + 1], 4));
    txAccZCharacteristic.writeValue(String(features[ix + 2], 4));
    txGyroXCharacteristic.writeValue(String(features[ix + 3], 4));
    txGyroYCharacteristic.writeValue(String(features[ix + 4], 4));
    txGyroZCharacteristic.writeValue(String(features[ix + 5], 4));

    delay(EI_CLASSIFIER_INTERVAL_MS);
  }

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
  
  if (label == "rest")
  {
    label = "1";
  }
  else if (label == "walking")
  {
    label = "2";
  }
  else if (label == "running")
  {
    label = "3";
  }
  else if (label == "goingstair")
  {
    label = "4";
  }

  ei_printf("$$$$ Detected %s with score %f \n", label.c_str(), score);

  predictions[pCounter++] = label;

  if (pCounter == PSIZE)
  {
    pCounter = 0;

    String fLabel = getFinalPrediction();
    // global_data = global_data + "," + fLabel;
    global_data = fLabel;

    ei_printf("######  Final prediction %s \n", fLabel.c_str());

    ei_printf("Global data written %s\n", global_data.c_str());

    if (central)
    {

      ei_printf("######  Writing to BLE \n");
      // txBatCharacteristic.writeValue(String(vBat));
      // txPredCharacteristic.writeValue(label.c_str());
      txPredCharacteristic.writeValue(global_data.c_str());

      if (receiving == true)
      {
        global_data = "";
      }
    }
  }
}