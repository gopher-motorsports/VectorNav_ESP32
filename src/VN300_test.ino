#include <ESP32-TWAI-CAN.hpp>
#include "CAN_OFFSET_SCALE.h"
#include <TelnetStream.h>
#include <TelnetPrint.h>
#include "OTA.h"
#include "LUT.h"

#define BINARY_OUTPUT_RATE_DIVIDER 4
#define FILTER_CUTOFF_FREQ 5.0
#define CAN_TX 25
#define CAN_RX 26
#define MAX_TX 17
#define MAX_RX 5
#define HBT_LED 22

#define FRONT_TRACKWIDTH_M 1.2f
#define REAR_TRACKWIDTH_M 1.18f
#define WHEELBASE_M 1.6f

volatile bool crcFail000B = false;
volatile bool crcFail0046 = false;
volatile bool crcFail0040 = false;

volatile uint32_t crcFail000B_count = 0;
volatile uint32_t crcFail0046_count = 0;
volatile uint32_t crcFail0040_count = 0;

volatile uint32_t canRxFrameCount = 0;
volatile uint32_t canRx105Count = 0;
volatile uint32_t canRx30ACount = 0;
volatile uint32_t canRx30BCount = 0;

volatile uint32_t canRxLastId = 0;
volatile uint8_t canRxLastDlc = 0;
volatile uint32_t canRxLastTime_ms = 0;
/*
Convention:
X forward
Y right
positive steering = left turn
left turn:
  steer > 0
  yawRate < 0
  Vy > 0

Slip convention:
left turn  -> positive slip angles
right turn -> negative slip angles
*/

// VN300 position with FL as origin.
// FL = (0,0), FR = (0, +front track), rear axle at x = -wheelbase.
// VN300 = (-a, b)
#define VN300_A_FROM_FRONT_M 0.66f  // replace with measured front axle distance
#define VN300_B_FROM_FL_M 0.59f     // replace with measured lateral distance from FL

#define MIN_SLIP_VX_MPS 2.0f
#define MAX_ABS_SLIP_DEG 45.0f

#define STEERING_FILTER_CUTOFF_HZ 5.0f
#define STEERING_RX_DT_S 0.01f  // set to actual CAN frame period

#define MTF01_LEFT_CAN_ID 0x30A
#define MTF01_RIGHT_CAN_ID 0x30B

// MTF positions relative to VN300:
// Left  = (+x1, -y)
// Right = (+x1, +y)
#define MTF01_X1_M -1.3f  // replace with measured value
#define MTF01_Y_M 0.55f    // replace with measured value

// Angle from MTF01 +X axis to VN300/body +X/+Y axes.
// Change to 135, -135, or -45 after straight-line testing if needed.
#define MTF01_ROTATION_DEG -45.0f

// MTF01 reports cm/s at a reference height of 1 m.
// This is therefore the maximum accepted angular-flow-equivalent speed.
#define MTF01_MAX_FLOW_RADPS 7.0f

#define MTF01_MIN_HEIGHT_M 0.20f
#define MTF01_MAX_HEIGHT_M 2.00f

#define MTF01_MIN_FLOW_QUALITY 0

// Reject stale MTF data.
#define MTF01_TIMEOUT_MS 100

// Weight given to the averaged MTF estimate when valid.
// fused = weight*MTF + (1-weight)*VN300
#define MTF01_FUSION_WEIGHT 0.70f

static float steeringWheelFilt_deg = 0.0f;
static bool steeringFiltInit = false;

float rc2 = 1.0f / (2.0f * PI * STEERING_FILTER_CUTOFF_HZ);
float k = STEERING_RX_DT_S / (STEERING_RX_DT_S + rc2);

float rc = 1 / (6.28318 * FILTER_CUTOFF_FREQ);
float tp = BINARY_OUTPUT_RATE_DIVIDER / 400.0;
float filt_coeff[2] = { tp / (tp + rc), rc / (tp + rc) };

const float RFR[3][3] = { { -1.0, 0, 0 }, { 0, -1.0, 0 }, { 0, 0, 1.0 } };
const float GNSSA_offset[3] = { 1.334, 0.0, -0.464 };
const float GNSSB_baseline[6] = { -0.762, 0, -0.057, 0.01, 0.01, 0.01 };

void TaskVN300Config(void *pvParameters);
void TaskVN300CheckStatus(void *pvParameters);
void TaskVN300Update(void *pvParameters);
void TaskOTAHandleAlive(void *pvParameters);
void TaskTelnetPrint(void *pvParameters);
void TaskSerialPrint(void *pvParameters);
void TaskAttitudeFilt(void *pvParameters);
void TaskIMUFilt(void *pvParameters);
void TaskCANComms(void *pvParameters);
void TaskCANRx(void *pvParameters);
void TaskSlipAngle(void *pvParameters);

TaskHandle_t VN300_Config_Task_Handle;
TaskHandle_t VN300_Check_Status_Task_Handle;
TaskHandle_t VN300_Update_Task_Handle;
TaskHandle_t OTA_Handle_Alive_Task_Handle;
TaskHandle_t Telnet_Print_Task_Handle;
TaskHandle_t Serial_Print_Task_Handle;
TaskHandle_t Attitude_Filt_Task_Handle;
TaskHandle_t IMU_Filt_Task_Handle;
TaskHandle_t CAN_Comms_Task_Handle;
TaskHandle_t CAN_Rx_Task_Handle;
TaskHandle_t Slip_Angle_Task_Handle;

QueueHandle_t INS_status;
QueueHandle_t Latitude;
QueueHandle_t Longitude;
QueueHandle_t Altitude;
QueueHandle_t VelBodyX;
QueueHandle_t VelBodyY;
QueueHandle_t VelBodyZ;

QueueHandle_t Yaw;
QueueHandle_t Pitch;
QueueHandle_t Roll;
QueueHandle_t QuatX;
QueueHandle_t QuatY;
QueueHandle_t QuatZ;
QueueHandle_t QuatS;
QueueHandle_t LinBodyAccX;
QueueHandle_t LinBodyAccY;
QueueHandle_t LinBodyAccZ;

QueueHandle_t QuatX_filt;
QueueHandle_t QuatY_filt;
QueueHandle_t QuatZ_filt;
QueueHandle_t QuatS_filt;
QueueHandle_t LinBodyAccX_filt;
QueueHandle_t LinBodyAccY_filt;
QueueHandle_t LinBodyAccZ_filt;

QueueHandle_t TimeUtcY;
QueueHandle_t TimeUtcMonth;
QueueHandle_t TimeUtcD;
QueueHandle_t TimeUtcH;
QueueHandle_t TimeUtcMin;
QueueHandle_t TimeUtcS;
QueueHandle_t TimeUtcF;

QueueHandle_t GyroBodyX;
QueueHandle_t GyroBodyY;
QueueHandle_t GyroBodyZ;

QueueHandle_t GyroBodyX_filt;
QueueHandle_t GyroBodyY_filt;
QueueHandle_t GyroBodyZ_filt;

QueueHandle_t SteeringAngle_deg;
QueueHandle_t SteerAngleFL_deg;
QueueHandle_t SteerAngleFR_deg;

QueueHandle_t SlipAngleFL_deg;
QueueHandle_t SlipAngleFR_deg;
QueueHandle_t SlipAngleRL_deg;
QueueHandle_t SlipAngleRR_deg;

QueueHandle_t MTF01_Left_Distance_m;
QueueHandle_t MTF01_Left_FlowX_raw;
QueueHandle_t MTF01_Left_FlowY_raw;
QueueHandle_t MTF01_Left_VxBody_mps;
QueueHandle_t MTF01_Left_VyBody_mps;
QueueHandle_t MTF01_Left_VxVN_mps;
QueueHandle_t MTF01_Left_VyVN_mps;
QueueHandle_t MTF01_Left_Valid;

QueueHandle_t MTF01_Right_Distance_m;
QueueHandle_t MTF01_Right_FlowX_raw;
QueueHandle_t MTF01_Right_FlowY_raw;
QueueHandle_t MTF01_Right_VxBody_mps;
QueueHandle_t MTF01_Right_VyBody_mps;
QueueHandle_t MTF01_Right_VxVN_mps;
QueueHandle_t MTF01_Right_VyVN_mps;
QueueHandle_t MTF01_Right_Valid;

QueueHandle_t VelBodyX_fused;
QueueHandle_t VelBodyY_fused;

QueueHandle_t MTF01_Left_SQUAL;
QueueHandle_t MTF01_Right_SQUAL;

volatile uint32_t mtf01LeftLastRx_ms = 0;
volatile uint32_t mtf01RightLastRx_ms = 0;

void setup() {
  pinMode(22, OUTPUT);
  INS_status = xQueueCreate(1, sizeof(uint16_t));
  Latitude = xQueueCreate(1, sizeof(double));
  Longitude = xQueueCreate(1, sizeof(double));
  Altitude = xQueueCreate(1, sizeof(double));
  VelBodyX = xQueueCreate(1, sizeof(float));
  VelBodyY = xQueueCreate(1, sizeof(float));
  VelBodyZ = xQueueCreate(1, sizeof(float));
  Yaw = xQueueCreate(1, sizeof(float));
  Pitch = xQueueCreate(1, sizeof(float));
  Roll = xQueueCreate(1, sizeof(float));
  QuatX = xQueueCreate(1, sizeof(float));
  QuatY = xQueueCreate(1, sizeof(float));
  QuatZ = xQueueCreate(1, sizeof(float));
  QuatS = xQueueCreate(1, sizeof(float));
  LinBodyAccX = xQueueCreate(1, sizeof(float));
  LinBodyAccY = xQueueCreate(1, sizeof(float));
  LinBodyAccZ = xQueueCreate(1, sizeof(float));
  TimeUtcY = xQueueCreate(1, sizeof(int8_t));
  TimeUtcMonth = xQueueCreate(1, sizeof(uint8_t));
  TimeUtcD = xQueueCreate(1, sizeof(uint8_t));
  TimeUtcH = xQueueCreate(1, sizeof(uint8_t));
  TimeUtcMin = xQueueCreate(1, sizeof(uint8_t));
  TimeUtcS = xQueueCreate(1, sizeof(uint8_t));
  TimeUtcF = xQueueCreate(1, sizeof(uint16_t));
  GyroBodyX = xQueueCreate(1, sizeof(float));
  GyroBodyY = xQueueCreate(1, sizeof(float));
  GyroBodyZ = xQueueCreate(1, sizeof(float));
  QuatX_filt = xQueueCreate(1, sizeof(float));
  QuatY_filt = xQueueCreate(1, sizeof(float));
  QuatZ_filt = xQueueCreate(1, sizeof(float));
  QuatS_filt = xQueueCreate(1, sizeof(float));
  LinBodyAccX_filt = xQueueCreate(1, sizeof(float));
  LinBodyAccY_filt = xQueueCreate(1, sizeof(float));
  LinBodyAccZ_filt = xQueueCreate(1, sizeof(float));
  GyroBodyX_filt = xQueueCreate(1, sizeof(float));
  GyroBodyY_filt = xQueueCreate(1, sizeof(float));
  GyroBodyZ_filt = xQueueCreate(1, sizeof(float));
  SteeringAngle_deg = xQueueCreate(1, sizeof(float));
  SteerAngleFL_deg = xQueueCreate(1, sizeof(float));
  SteerAngleFR_deg = xQueueCreate(1, sizeof(float));
  SlipAngleFL_deg = xQueueCreate(1, sizeof(float));
  SlipAngleFR_deg = xQueueCreate(1, sizeof(float));
  SlipAngleRL_deg = xQueueCreate(1, sizeof(float));
  SlipAngleRR_deg = xQueueCreate(1, sizeof(float));
  MTF01_Left_Distance_m = xQueueCreate(1, sizeof(float));
  MTF01_Left_FlowX_raw = xQueueCreate(1, sizeof(int16_t));
  MTF01_Left_FlowY_raw = xQueueCreate(1, sizeof(int16_t));
  MTF01_Left_VxBody_mps = xQueueCreate(1, sizeof(float));
  MTF01_Left_VyBody_mps = xQueueCreate(1, sizeof(float));
  MTF01_Left_VxVN_mps = xQueueCreate(1, sizeof(float));
  MTF01_Left_VyVN_mps = xQueueCreate(1, sizeof(float));
  MTF01_Left_Valid = xQueueCreate(1, sizeof(bool));
  MTF01_Right_Distance_m = xQueueCreate(1, sizeof(float));
  MTF01_Right_FlowX_raw = xQueueCreate(1, sizeof(int16_t));
  MTF01_Right_FlowY_raw = xQueueCreate(1, sizeof(int16_t));
  MTF01_Right_VxBody_mps = xQueueCreate(1, sizeof(float));
  MTF01_Right_VyBody_mps = xQueueCreate(1, sizeof(float));
  MTF01_Right_VxVN_mps = xQueueCreate(1, sizeof(float));
  MTF01_Right_VyVN_mps = xQueueCreate(1, sizeof(float));
  MTF01_Right_Valid = xQueueCreate(1, sizeof(bool));
  VelBodyX_fused = xQueueCreate(1, sizeof(float));
  VelBodyY_fused = xQueueCreate(1, sizeof(float));
  MTF01_Left_SQUAL = xQueueCreate(1, sizeof(uint8_t));
  MTF01_Right_SQUAL = xQueueCreate(1, sizeof(uint8_t));

  uint8_t t1 = 0;
  float t2 = 0.0;
  double t3 = 0.0;
  uint16_t t4 = 0;
  int16_t zeroS16 = 0;
  bool falseValue = false;

  xQueueOverwrite(INS_status, &t4);
  xQueueOverwrite(TimeUtcF, &t4);

  xQueueOverwrite(Latitude, &t3);
  xQueueOverwrite(Longitude, &t3);
  xQueueOverwrite(Altitude, &t3);

  xQueueOverwrite(VelBodyX, &t2);
  xQueueOverwrite(VelBodyY, &t2);
  xQueueOverwrite(VelBodyZ, &t2);
  xQueueOverwrite(Yaw, &t2);
  xQueueOverwrite(Pitch, &t2);
  xQueueOverwrite(Roll, &t2);
  xQueueOverwrite(QuatX, &t2);
  xQueueOverwrite(QuatY, &t2);
  xQueueOverwrite(QuatZ, &t2);
  xQueueOverwrite(QuatS, &t2);
  xQueueOverwrite(LinBodyAccX, &t2);
  xQueueOverwrite(LinBodyAccY, &t2);
  xQueueOverwrite(LinBodyAccZ, &t2);
  xQueueOverwrite(GyroBodyX, &t2);
  xQueueOverwrite(GyroBodyY, &t2);
  xQueueOverwrite(GyroBodyZ, &t2);

  xQueueOverwrite(TimeUtcY, &t1);
  xQueueOverwrite(TimeUtcMonth, &t1);
  xQueueOverwrite(TimeUtcD, &t1);
  xQueueOverwrite(TimeUtcH, &t1);
  xQueueOverwrite(TimeUtcMin, &t1);
  xQueueOverwrite(TimeUtcS, &t1);

  xQueueOverwrite(QuatX_filt, &t2);
  xQueueOverwrite(QuatY_filt, &t2);
  xQueueOverwrite(QuatZ_filt, &t2);
  xQueueOverwrite(QuatS_filt, &t2);
  xQueueOverwrite(LinBodyAccX_filt, &t2);
  xQueueOverwrite(LinBodyAccY_filt, &t2);
  xQueueOverwrite(LinBodyAccZ_filt, &t2);

  xQueueOverwrite(GyroBodyX_filt, &t2);
  xQueueOverwrite(GyroBodyY_filt, &t2);
  xQueueOverwrite(GyroBodyZ_filt, &t2);

  xQueueOverwrite(SteeringAngle_deg, &t2);
  xQueueOverwrite(SteerAngleFL_deg, &t2);
  xQueueOverwrite(SteerAngleFR_deg, &t2);

  xQueueOverwrite(SlipAngleFL_deg, &t2);
  xQueueOverwrite(SlipAngleFR_deg, &t2);
  xQueueOverwrite(SlipAngleRL_deg, &t2);
  xQueueOverwrite(SlipAngleRR_deg, &t2);

  xQueueOverwrite(MTF01_Left_Distance_m, &t2);
  xQueueOverwrite(MTF01_Left_FlowX_raw, &zeroS16);
  xQueueOverwrite(MTF01_Left_FlowY_raw, &zeroS16);
  xQueueOverwrite(MTF01_Left_VxBody_mps, &t2);
  xQueueOverwrite(MTF01_Left_VyBody_mps, &t2);
  xQueueOverwrite(MTF01_Left_VxVN_mps, &t2);
  xQueueOverwrite(MTF01_Left_VyVN_mps, &t2);
  xQueueOverwrite(MTF01_Left_Valid, &falseValue);

  xQueueOverwrite(MTF01_Right_Distance_m, &t2);
  xQueueOverwrite(MTF01_Right_FlowX_raw, &zeroS16);
  xQueueOverwrite(MTF01_Right_FlowY_raw, &zeroS16);
  xQueueOverwrite(MTF01_Right_VxBody_mps, &t2);
  xQueueOverwrite(MTF01_Right_VyBody_mps, &t2);
  xQueueOverwrite(MTF01_Right_VxVN_mps, &t2);
  xQueueOverwrite(MTF01_Right_VyVN_mps, &t2);
  xQueueOverwrite(MTF01_Right_Valid, &falseValue);

  xQueueOverwrite(VelBodyX_fused, &t2);
  xQueueOverwrite(VelBodyY_fused, &t2);

  xQueueOverwrite(MTF01_Left_SQUAL, &t1);
  xQueueOverwrite(MTF01_Right_SQUAL, &t1);

  Serial2.begin(115200, SERIAL_8N1, MAX_RX, MAX_TX);
  Serial.begin(115200);
  //twai_filter_config_t fvc_rx_filter = { .acceptance_code = (0x109 << 21), .acceptance_mask = ~(0x7FF << 21), .single_filter = true };
  //twai_general_config_t noack_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)(CAN_TX), (gpio_num_t)(CAN_RX), TWAI_MODE_NORMAL);
  //ESP32Can.begin(ESP32Can.convertSpeed(1000), CAN_TX, CAN_RX, 10, 10, &fvc_rx_filter, &noack_config);


  twai_general_config_t noack_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)(CAN_TX), (gpio_num_t)(CAN_RX), TWAI_MODE_NO_ACK);
  ESP32Can.begin(ESP32Can.convertSpeed(1000), CAN_TX, CAN_RX, 10, 10, nullptr, &noack_config);

  xTaskCreatePinnedToCore(TaskVN300Config, "Initial config task", 2048, NULL, 1, &VN300_Config_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskVN300CheckStatus, "Check INS status", 2048, NULL, 1, &VN300_Check_Status_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskVN300Update, "VN300 Update values", 4096, NULL, 1, &VN300_Update_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskOTAHandleAlive, "OTA handle", 16384, NULL, 1, &OTA_Handle_Alive_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskTelnetPrint, "Debug print", 8192, NULL, 1, &Telnet_Print_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskSerialPrint, "Debug print serial", 4096, NULL, 1, &Serial_Print_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskAttitudeFilt, "Attitude filter", 2048, NULL, 1, &Attitude_Filt_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskIMUFilt, "IMU Gyro filter", 2048, NULL, 1, &IMU_Filt_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskCANComms, "CAN Comms", 4096, NULL, 1, &CAN_Comms_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskCANRx, "CAN RX", 4096, NULL, 1, &CAN_Rx_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskSlipAngle, "Slip angle task", 4096, NULL, 1, &Slip_Angle_Task_Handle, 1);

  delay(1000);
}

static inline void rotateMTFToBody(float vxMtf,
                                   float vyMtf,
                                   float *vxBody,
                                   float *vyBody) {
  const float theta = MTF01_ROTATION_DEG * DEG_TO_RAD;
  const float cosTheta = cosf(theta);
  const float sinTheta = sinf(theta);

  *vxBody = cosTheta * vxMtf - sinTheta * vyMtf;
  *vyBody = sinTheta * vxMtf + cosTheta * vyMtf;
}

void filterOne(QueueHandle_t rawQ, QueueHandle_t filtQ) {
  float raw = 0.0f;
  float prev = 0.0f;
  float out = 0.0f;

  xQueuePeek(rawQ, &raw, portMAX_DELAY);
  xQueuePeek(filtQ, &prev, portMAX_DELAY);
  if (!isfinite(raw) || fabs(raw) > 50.0f) {
    return;  // reject bad gyro sample
  }
  if (!isfinite(prev) || fabs(prev) > 50.0f) {
    prev = raw;  // recover from poisoned previous value
  }
  out = raw * filt_coeff[0] + prev * filt_coeff[1];
  if (isfinite(out)) {
    xQueueOverwrite(filtQ, &out);
  }
}

void append_ascii_checksum(String &cmd) {
  uint8_t checksum = 0;
  for (size_t i = 1; i < cmd.length(); i++) {
    checksum ^= cmd[i];
  }
  char buf[5];
  sprintf(buf, "*%02X\r\n", checksum);
  cmd += buf;
}

bool send_message(String &cmd) {
  char ctemp = 0x00;
  String rx = "";
  //Serial.println(cmd);
  for (size_t i = 0; i < cmd.length(); i++) {
    Serial2.write(cmd[i]);
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
  while (Serial2.available()) {
    ctemp = Serial2.read();
    if (ctemp == '$') {
      ctemp = Serial2.read();
      if (ctemp == 'V') {
        rx += "$" + String(ctemp);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        for (size_t i = 2; i < cmd.length(); i++) {
          ctemp = Serial2.read();
          if (ctemp == '\r') {
            break;
          }
          rx += String(ctemp);
        }
      }
      break;
    }
  }

  //Serial.println(rx);
  return true;
}

uint16_t vn_crc_update(uint16_t crc, uint8_t data) {
  crc = (uint8_t)(crc >> 8) | (crc << 8);
  crc ^= data;
  crc ^= (uint8_t)(crc & 0xFF) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0x00FF) << 5;
  return crc;
}

void TaskVN300Config(void *pvParameters) {
  //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  String c[9];
  c[0] = "$VNWRG,06,0";
  c[1] = "$VNWRG,05,115200";
  c[2] = "$VNWRG,26";
  c[3] = "$VNWRG,57";
  c[4] = "$VNWRG,93";
  c[5] = "$VNRRG,04";
  c[6] = "$VNWRG,75,1,";
  c[7] = "$VNWNV";
  c[8] = "$VNRST";

  c[6] += String(BINARY_OUTPUT_RATE_DIVIDER) + ",20,0001";

  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      c[2] += "," + String(RFR[i][j], 5);
    }
  }
  for (uint8_t i = 0; i < 3; i++) {
    c[3] += "," + String(GNSSA_offset[i], 4);
  }
  for (uint8_t i = 0; i < 6; i++) {
    c[4] += "," + String(GNSSB_baseline[i], 4);
  }

  for (uint8_t i = 0; i < 9; i++) {
    append_ascii_checksum(c[i]);
  }

  TelnetStream.print("Writing configuration...\r\n");
  for (uint8_t i = 0; i < 9; i++) {
    bool txs = send_message(c[i]);
    if (txs == false) {
      //i -= 1;
    }
  }
  TelnetStream.print("\r\nConfiguration complete\r\n");
  xTaskNotifyGive(Telnet_Print_Task_Handle);
  xTaskNotifyGive(VN300_Check_Status_Task_Handle);
  xTaskNotifyGive(CAN_Comms_Task_Handle);
  vTaskDelete(NULL);
}

void TaskVN300CheckStatus(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  char c = 0x00;
  char temp = 0x00;
  uint8_t i = 0;
  uint32_t ready_count = 0;
  uint16_t type_byte = 0x0000;
  uint16_t ins_status = 0x0000;
  uint16_t crc = 0x0000;
  while (1) {
    if (ready_count > 10) {
      TelnetStream.print("Ready!\r\n");
      xTaskNotifyGive(VN300_Update_Task_Handle);
      xTaskNotifyGive(Serial_Print_Task_Handle);
      vTaskDelete(NULL);
    }
    switch (i) {
      case 0:
        c = Serial2.read();
        if (c == 0xFA) {
          i = 1;
          vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        break;
      case 1:
        c = Serial2.read();
        if (c == 0x20) {
          i = 2;
        } else {
          i = 0;
        }
        break;
      case 2:
        c = Serial2.read();
        temp = (uint16_t)(c & 0x00FF);
        c = Serial2.read();
        temp |= (uint16_t)(c << 8);
        if (temp == 0x00001) {
          i = 3;
        } else {
          i = 0;
        }
        break;
      case 3:
        c = Serial2.read();
        ins_status = (uint16_t)(c & 0x00FF);
        c = Serial2.read();
        ins_status |= (uint16_t)(c << 8);
        ins_status &= 0x0377;
        //Serial.println(ins_status,BIN);
        c = Serial2.read();
        crc = (uint16_t)(c & 0x00FF);
        c = Serial2.read();
        crc |= (uint16_t)(c << 8);
        xQueueOverwrite(INS_status, &ins_status);
        if (((ins_status & 0x0003) > 0) && ((ins_status | 0x018B) == 0x038F)) {
          ready_count++;
        } else {
          ready_count = 0;
        }
        i = 0;
        break;
      default:
        i = 0;
    }
  }
}

void TaskVN300Update(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  String cm[4];

  cm[0] = "$VNWRG,06,0";

  cm[1] = "$VNWRG,75,1,";
  cm[1] += String(BINARY_OUTPUT_RATE_DIVIDER) + ",20,000B";

  cm[2] = "$VNWRG,76,1,";
  cm[2] += String(BINARY_OUTPUT_RATE_DIVIDER) + ",10,0046";

  cm[3] = "$VNWRG,77,1,";
  cm[3] += String(BINARY_OUTPUT_RATE_DIVIDER) + ",06,0040,0400";

  for (uint8_t j = 0; j < 4; j++) {
    append_ascii_checksum(cm[j]);
  }

  for (uint8_t j = 0; j < 4; j++) {
    send_message(cm[j]);
  }

  uint8_t c = 0x00;
  uint8_t state = 0;

  uint8_t buf[8] = { 0 };
  uint16_t type = 0;
  uint16_t crcRun = 0;

  while (1) {
    switch (state) {

      case 0:
        if (!Serial2.available()) {
          vTaskDelay(1 / portTICK_PERIOD_MS);
          break;
        }

        c = (uint8_t)Serial2.read();

        if (c == 0xFA) {
          crcRun = 0;
          state = 1;
        }
        break;

      case 1:
        if (!Serial2.available()) break;

        c = (uint8_t)Serial2.read();
        crcRun = vn_crc_update(crcRun, c);
        vTaskDelay(6 / portTICK_PERIOD_MS);

        if (c == 0x20) {
          state = 2;
        } else if (c == 0x10) {
          state = 3;
        } else if (c == 0x06) {
          state = 4;
        } else {
          state = 0;
        }
        break;

      case 2:
        {
          // Group 0x20, type 0x000B:
          // INS Status, Lat, Lon, Alt, VelBodyX, VelBodyY, VelBodyZ

          uint16_t insStatusLocal = 0;
          double latLocal = 0.0;
          double lonLocal = 0.0;
          double altLocal = 0.0;
          float vxLocal = 0.0f;
          float vyLocal = 0.0f;
          float vzLocal = 0.0f;

          c = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, c);
          type = c;

          c = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, c);
          type |= ((uint16_t)c << 8);

          if (type != 0x000B) {
            state = 0;
            break;
          }

          for (uint8_t j = 0; j < 2; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&insStatusLocal, buf, sizeof(uint16_t));
          insStatusLocal &= 0x0177;

          for (uint8_t j = 0; j < 8; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&latLocal, buf, sizeof(double));

          for (uint8_t j = 0; j < 8; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&lonLocal, buf, sizeof(double));

          for (uint8_t j = 0; j < 8; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&altLocal, buf, sizeof(double));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&vxLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&vyLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&vzLocal, buf, sizeof(float));

          uint8_t crc0 = (uint8_t)Serial2.read();
          uint8_t crc1 = (uint8_t)Serial2.read();

          uint16_t crcReceived = ((uint16_t)crc0 << 8) | crc1;

          bool check = (crcRun == crcReceived);

          if (check) {
            xQueueOverwrite(INS_status, &insStatusLocal);

            if (isfinite(latLocal)) xQueueOverwrite(Latitude, &latLocal);
            if (isfinite(lonLocal)) xQueueOverwrite(Longitude, &lonLocal);
            if (isfinite(altLocal)) xQueueOverwrite(Altitude, &altLocal);

            if (isfinite(vxLocal)) xQueueOverwrite(VelBodyX, &vxLocal);
            if (isfinite(vyLocal)) xQueueOverwrite(VelBodyY, &vyLocal);
            if (isfinite(vzLocal)) xQueueOverwrite(VelBodyZ, &vzLocal);
          } else {
            crcFail000B = true;
            crcFail000B_count++;
          }

          state = 0;
          break;
        }

      case 3:
        {
          // Group 0x10, type 0x0046:
          // YPR, Quaternion, Linear Body Accel

          float yawLocal = 0.0f;
          float pitchLocal = 0.0f;
          float rollLocal = 0.0f;
          float qxLocal = 0.0f;
          float qyLocal = 0.0f;
          float qzLocal = 0.0f;
          float qsLocal = 0.0f;
          float axLocal = 0.0f;
          float ayLocal = 0.0f;
          float azLocal = 0.0f;

          c = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, c);
          type = c;

          c = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, c);
          type |= ((uint16_t)c << 8);

          if (type != 0x0046) {
            state = 0;
            break;
          }

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&yawLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&pitchLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&rollLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&qxLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&qyLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&qzLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&qsLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&axLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&ayLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&azLocal, buf, sizeof(float));

          uint8_t crc0 = (uint8_t)Serial2.read();
          uint8_t crc1 = (uint8_t)Serial2.read();

          uint16_t crcReceived = ((uint16_t)crc0 << 8) | crc1;

          bool check = (crcRun == crcReceived);

          if (check) {
            if (isfinite(yawLocal)) xQueueOverwrite(Yaw, &yawLocal);
            if (isfinite(pitchLocal)) xQueueOverwrite(Pitch, &pitchLocal);
            if (isfinite(rollLocal)) xQueueOverwrite(Roll, &rollLocal);

            if (isfinite(qxLocal)) xQueueOverwrite(QuatX, &qxLocal);
            if (isfinite(qyLocal)) xQueueOverwrite(QuatY, &qyLocal);
            if (isfinite(qzLocal)) xQueueOverwrite(QuatZ, &qzLocal);
            if (isfinite(qsLocal)) xQueueOverwrite(QuatS, &qsLocal);

            if (isfinite(axLocal)) xQueueOverwrite(LinBodyAccX, &axLocal);
            if (isfinite(ayLocal)) xQueueOverwrite(LinBodyAccY, &ayLocal);
            if (isfinite(azLocal)) xQueueOverwrite(LinBodyAccZ, &azLocal);

            xTaskNotifyGive(Attitude_Filt_Task_Handle);
          } else {
            crcFail0046 = true;
            crcFail0046_count++;
          }

          state = 0;
          break;
        }

      case 4:
        {
          // Group 0x06, types 0x0040 and 0x0400:
          // Time UTC + Gyro

          uint8_t yearLocal = 0;
          uint8_t monthLocal = 0;
          uint8_t dayLocal = 0;
          uint8_t hourLocal = 0;
          uint8_t minLocal = 0;
          uint8_t secLocal = 0;
          uint16_t fracLocal = 0;
          float gxLocal = 0.0f;
          float gyLocal = 0.0f;
          float gzLocal = 0.0f;

          c = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, c);
          type = c;

          c = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, c);
          type |= ((uint16_t)c << 8);

          if (type != 0x0040) {
            state = 0;
            break;
          }

          c = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, c);
          type = c;

          c = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, c);
          type |= ((uint16_t)c << 8);

          if (type != 0x0400) {
            state = 0;
            break;
          }

          yearLocal = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, yearLocal);

          monthLocal = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, monthLocal);

          dayLocal = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, dayLocal);

          hourLocal = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, hourLocal);

          minLocal = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, minLocal);

          secLocal = (uint8_t)Serial2.read();
          crcRun = vn_crc_update(crcRun, secLocal);

          for (uint8_t j = 0; j < 2; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&fracLocal, buf, sizeof(uint16_t));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&gxLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&gyLocal, buf, sizeof(float));

          for (uint8_t j = 0; j < 4; j++) {
            c = (uint8_t)Serial2.read();
            crcRun = vn_crc_update(crcRun, c);
            buf[j] = c;
          }
          memcpy(&gzLocal, buf, sizeof(float));

          uint8_t crc0 = (uint8_t)Serial2.read();
          uint8_t crc1 = (uint8_t)Serial2.read();

          uint16_t crcReceived = ((uint16_t)crc0 << 8) | crc1;

          bool check = (crcRun == crcReceived);

          if (check) {
            xQueueOverwrite(TimeUtcY, &yearLocal);
            xQueueOverwrite(TimeUtcMonth, &monthLocal);
            xQueueOverwrite(TimeUtcD, &dayLocal);
            xQueueOverwrite(TimeUtcH, &hourLocal);
            xQueueOverwrite(TimeUtcMin, &minLocal);
            xQueueOverwrite(TimeUtcS, &secLocal);
            xQueueOverwrite(TimeUtcF, &fracLocal);

            if (isfinite(gxLocal)) xQueueOverwrite(GyroBodyX, &gxLocal);
            if (isfinite(gyLocal)) xQueueOverwrite(GyroBodyY, &gyLocal);
            if (isfinite(gzLocal)) xQueueOverwrite(GyroBodyZ, &gzLocal);

            xTaskNotifyGive(IMU_Filt_Task_Handle);
          } else {
            crcFail0040 = true;
            crcFail0040_count++;
          }

          state = 0;
          break;
        }

      default:
        state = 0;
        break;
    }
  }
}

void TaskOTAHandleAlive(void *pvParameters) {
  bool setup_ret = false;
  ArduinoOTA.setHostname("ESP32_VN300");
  while (setup_ret != true) {
    setup_ret = setupOTA("", my_ssid, my_password);
    if (setup_ret != true) {
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
  }
  //xTaskNotifyGive(VN300_Config_Task_Handle);
  while (1) {
    ArduinoOTA.handle();
    vTaskDelay(93 / portTICK_PERIOD_MS);
    digitalWrite(HBT_LED, !digitalRead(HBT_LED));
  }
}

void TaskTelnetPrint(void *pvParemeters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  uint8_t i = 0;
  char c = 0x00;
  char temp = 0x00;
  uint16_t t1 = 0;
  uint8_t t2 = 0;
  float t3 = 0.0;
  double t4 = 0.0;

  while (1) {
    while (TelnetStream.available()) {
      temp = TelnetStream.read();
      if ((uint8_t)(temp) >= 48 && (uint8_t)(temp) <= 54) {
        c = temp;
        i = ((uint8_t)(c)-48);
      }
    }
    switch (i) {
      case 0:
        TelnetStream.print("Enter 1 for INS info\r\n");
        TelnetStream.print("Enter 2 for Attitude\r\n");
        TelnetStream.print("Enter 3 for Time and IMU and CRC\r\n");
        TelnetStream.print("Enter 4 for Steering and Slip Angles\r\n");
        TelnetStream.print("Enter 5 for MTF01 Fusion Data\r\n");
        TelnetStream.print("Enter 6 for CAN RX Statistics\r\n");
        TelnetStream.print("Enter 0 to restart the menu\r\n");
        while (!TelnetStream.available()) {
        }
        break;
      case 1:
        xQueuePeek(INS_status, &t1, portMAX_DELAY);
        TelnetStream.print("INS Status bits: ");
        TelnetStream.print(t1, BIN);
        TelnetStream.print("\r\n");
        xQueuePeek(Latitude, &t4, portMAX_DELAY);
        TelnetStream.print("Latitude: ");
        TelnetStream.print(t4, 7);
        TelnetStream.print(" deg\r\n");
        xQueuePeek(Longitude, &t4, portMAX_DELAY);
        TelnetStream.print("Longitude: ");
        TelnetStream.print(t4, 7);
        TelnetStream.print(" deg\r\n");
        xQueuePeek(Altitude, &t4, portMAX_DELAY);
        TelnetStream.print("Altitude: ");
        TelnetStream.print(t4, 7);
        TelnetStream.print(" m\r\n");
        xQueuePeek(VelBodyX, &t3, portMAX_DELAY);
        TelnetStream.print("Vx: ");
        TelnetStream.print(t3, 4);
        TelnetStream.print(" m/s\r\n");
        xQueuePeek(VelBodyY, &t3, portMAX_DELAY);
        TelnetStream.print("Vy: ");
        TelnetStream.print(t3, 4);
        TelnetStream.print(" m/s\r\n");
        xQueuePeek(VelBodyZ, &t3, portMAX_DELAY);
        TelnetStream.print("Vz: ");
        TelnetStream.print(t3, 4);
        TelnetStream.print(" m/s\r\n");
        break;
      case 2:
        {
          xQueuePeek(Yaw, &t3, portMAX_DELAY);
          TelnetStream.print("Yaw: ");
          TelnetStream.print(t3, 4);
          TelnetStream.print(" deg\r\n");
          xQueuePeek(Pitch, &t3, portMAX_DELAY);
          TelnetStream.print("Pitch: ");
          TelnetStream.print(t3, 4);
          TelnetStream.print(" deg\r\n");
          xQueuePeek(Roll, &t3, portMAX_DELAY);
          TelnetStream.print("Roll: ");
          TelnetStream.print(t3, 4);
          TelnetStream.print(" deg\r\n");
          TelnetStream.print("Quat: ");
          xQueuePeek(QuatX_filt, &t3, portMAX_DELAY);
          TelnetStream.print(t3, 4);
          TelnetStream.print(" ");
          xQueuePeek(QuatY_filt, &t3, portMAX_DELAY);
          TelnetStream.print(t3, 4);
          TelnetStream.print(" ");
          xQueuePeek(QuatZ_filt, &t3, portMAX_DELAY);
          TelnetStream.print(t3, 4);
          TelnetStream.print(" ");
          xQueuePeek(QuatS_filt, &t3, portMAX_DELAY);
          TelnetStream.print(t3, 4);
          TelnetStream.print("\r\n");
          xQueuePeek(LinBodyAccX_filt, &t3, portMAX_DELAY);
          TelnetStream.print("Ax: ");
          TelnetStream.print(t3, 4);
          TelnetStream.print(" m/s^2\r\n");
          xQueuePeek(LinBodyAccY_filt, &t3, portMAX_DELAY);
          TelnetStream.print("Ay: ");
          TelnetStream.print(t3, 4);
          TelnetStream.print(" m/s^2\r\n");
          xQueuePeek(LinBodyAccZ_filt, &t3, portMAX_DELAY);
          TelnetStream.print("Az: ");
          TelnetStream.print(t3, 4);
          TelnetStream.print(" m/s^2\r\n");

          float qx_raw = 0.0f;
          float qy_raw = 0.0f;
          float qz_raw = 0.0f;
          float qs_raw = 0.0f;
          float ax_raw = 0.0f;
          float ay_raw = 0.0f;
          float az_raw = 0.0f;

          xQueuePeek(QuatX, &qx_raw, portMAX_DELAY);
          xQueuePeek(QuatY, &qy_raw, portMAX_DELAY);
          xQueuePeek(QuatZ, &qz_raw, portMAX_DELAY);
          xQueuePeek(QuatS, &qs_raw, portMAX_DELAY);

          xQueuePeek(LinBodyAccX, &ax_raw, portMAX_DELAY);
          xQueuePeek(LinBodyAccY, &ay_raw, portMAX_DELAY);
          xQueuePeek(LinBodyAccZ, &az_raw, portMAX_DELAY);

          TelnetStream.print("Raw finite Qx Qy Qz Qs: ");
          TelnetStream.print(isfinite(qx_raw));
          TelnetStream.print(" ");
          TelnetStream.print(isfinite(qy_raw));
          TelnetStream.print(" ");
          TelnetStream.print(isfinite(qz_raw));
          TelnetStream.print(" ");
          TelnetStream.print(isfinite(qs_raw));
          TelnetStream.print("\r\n");

          TelnetStream.print("Raw finite Ax Ay Az: ");
          TelnetStream.print(isfinite(ax_raw));
          TelnetStream.print(" ");
          TelnetStream.print(isfinite(ay_raw));
          TelnetStream.print(" ");
          TelnetStream.print(isfinite(az_raw));
          TelnetStream.print("\r\n");

          TelnetStream.print("CRC fail counts 000B/0046/0040: ");
          TelnetStream.print(crcFail000B_count);
          TelnetStream.print(" / ");
          TelnetStream.print(crcFail0046_count);
          TelnetStream.print(" / ");
          TelnetStream.print(crcFail0040_count);
          TelnetStream.print("\r\n");
          break;
        }
      case 3:
        TelnetStream.print("Time UTC: ");
        xQueuePeek(TimeUtcMonth, &t2, portMAX_DELAY);
        TelnetStream.print(t2);
        TelnetStream.print("/");
        xQueuePeek(TimeUtcD, &t2, portMAX_DELAY);
        TelnetStream.print(t2);
        TelnetStream.print("/");
        xQueuePeek(TimeUtcY, &t2, portMAX_DELAY);
        t1 = t2 + 2000;
        TelnetStream.print(t1);
        TelnetStream.print("     ");
        xQueuePeek(TimeUtcH, &t2, portMAX_DELAY);
        TelnetStream.print(t2);
        TelnetStream.print(":");
        xQueuePeek(TimeUtcMin, &t2, portMAX_DELAY);
        TelnetStream.print(t2);
        TelnetStream.print(":");
        xQueuePeek(TimeUtcS, &t2, portMAX_DELAY);
        TelnetStream.print(t2);
        TelnetStream.print(":");
        xQueuePeek(TimeUtcF, &t1, portMAX_DELAY);
        TelnetStream.print(t1);
        TelnetStream.print("\r\n");
        xQueuePeek(GyroBodyX_filt, &t3, portMAX_DELAY);
        TelnetStream.print("Gx: ");
        TelnetStream.print(t3, 4);
        TelnetStream.print(" rad/s\r\n");
        xQueuePeek(GyroBodyY_filt, &t3, portMAX_DELAY);
        TelnetStream.print("Gy: ");
        TelnetStream.print(t3, 4);
        TelnetStream.print(" rad/s\r\n");
        xQueuePeek(GyroBodyZ_filt, &t3, portMAX_DELAY);
        TelnetStream.print("Gz: ");
        TelnetStream.print(t3, 4);
        TelnetStream.print(" rad/s\r\n");
        break;
      case 4:
        {
          float steerWheel = 0.0f;
          float steerFL = 0.0f;
          float steerFR = 0.0f;

          float slipFL = 0.0f;
          float slipFR = 0.0f;
          float slipRL = 0.0f;
          float slipRR = 0.0f;

          xQueuePeek(SteeringAngle_deg, &steerWheel, portMAX_DELAY);
          xQueuePeek(SteerAngleFL_deg, &steerFL, portMAX_DELAY);
          xQueuePeek(SteerAngleFR_deg, &steerFR, portMAX_DELAY);

          xQueuePeek(SlipAngleFL_deg, &slipFL, portMAX_DELAY);
          xQueuePeek(SlipAngleFR_deg, &slipFR, portMAX_DELAY);
          xQueuePeek(SlipAngleRL_deg, &slipRL, portMAX_DELAY);
          xQueuePeek(SlipAngleRR_deg, &slipRR, portMAX_DELAY);

          TelnetStream.print("Steering Wheel: ");
          TelnetStream.print(steerWheel, 2);
          TelnetStream.print(" deg\r\n");

          TelnetStream.print("Steer FL: ");
          TelnetStream.print(steerFL, 2);
          TelnetStream.print(" deg\r\n");

          TelnetStream.print("Steer FR: ");
          TelnetStream.print(steerFR, 2);
          TelnetStream.print(" deg\r\n");

          TelnetStream.print("Slip FL: ");
          TelnetStream.print(slipFL, 2);
          TelnetStream.print(" deg\r\n");

          TelnetStream.print("Slip FR: ");
          TelnetStream.print(slipFR, 2);
          TelnetStream.print(" deg\r\n");

          TelnetStream.print("Slip RL: ");
          TelnetStream.print(slipRL, 2);
          TelnetStream.print(" deg\r\n");

          TelnetStream.print("Slip RR: ");
          TelnetStream.print(slipRR, 2);
          TelnetStream.print(" deg\r\n");

          break;
        }
      case 5:
        {
          float leftDistance_m = 0.0f;
          int16_t leftFlowX_raw = 0;
          int16_t leftFlowY_raw = 0;
          float leftVxBody_mps = 0.0f;
          float leftVyBody_mps = 0.0f;
          float leftVxVN_mps = 0.0f;
          float leftVyVN_mps = 0.0f;
          bool leftValid = false;

          float rightDistance_m = 0.0f;
          int16_t rightFlowX_raw = 0;
          int16_t rightFlowY_raw = 0;
          float rightVxBody_mps = 0.0f;
          float rightVyBody_mps = 0.0f;
          float rightVxVN_mps = 0.0f;
          float rightVyVN_mps = 0.0f;
          bool rightValid = false;

          float vxVN_mps = 0.0f;
          float vyVN_mps = 0.0f;
          float vxFused_mps = 0.0f;
          float vyFused_mps = 0.0f;
          float yawRate_radps = 0.0f;

          uint8_t leftSqual = 0;
          uint8_t rightSqual = 0;

          xQueuePeek(MTF01_Left_SQUAL, &leftSqual, portMAX_DELAY);
          xQueuePeek(MTF01_Right_SQUAL, &rightSqual, portMAX_DELAY);

          xQueuePeek(MTF01_Left_Distance_m,
                     &leftDistance_m,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Left_FlowX_raw,
                     &leftFlowX_raw,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Left_FlowY_raw,
                     &leftFlowY_raw,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Left_VxBody_mps,
                     &leftVxBody_mps,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Left_VyBody_mps,
                     &leftVyBody_mps,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Left_VxVN_mps,
                     &leftVxVN_mps,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Left_VyVN_mps,
                     &leftVyVN_mps,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Left_Valid,
                     &leftValid,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Right_Distance_m,
                     &rightDistance_m,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Right_FlowX_raw,
                     &rightFlowX_raw,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Right_FlowY_raw,
                     &rightFlowY_raw,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Right_VxBody_mps,
                     &rightVxBody_mps,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Right_VyBody_mps,
                     &rightVyBody_mps,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Right_VxVN_mps,
                     &rightVxVN_mps,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Right_VyVN_mps,
                     &rightVyVN_mps,
                     portMAX_DELAY);

          xQueuePeek(MTF01_Right_Valid,
                     &rightValid,
                     portMAX_DELAY);

          xQueuePeek(VelBodyX, &vxVN_mps, portMAX_DELAY);
          xQueuePeek(VelBodyY, &vyVN_mps, portMAX_DELAY);

          xQueuePeek(VelBodyX_fused,
                     &vxFused_mps,
                     portMAX_DELAY);

          xQueuePeek(VelBodyY_fused,
                     &vyFused_mps,
                     portMAX_DELAY);

          xQueuePeek(GyroBodyZ_filt,
                     &yawRate_radps,
                     portMAX_DELAY);

          TelnetStream.print("Yaw rate: ");
          TelnetStream.print(yawRate_radps, 4);
          TelnetStream.print(" rad/s\r\n");

          TelnetStream.print("\r\nLeft MTF01 - ID 0x30A\r\n");

          TelnetStream.print("Distance: ");
          TelnetStream.print(leftDistance_m, 3);
          TelnetStream.print(" m\r\n");

          TelnetStream.print("SQUAL: ");
          TelnetStream.print(leftSqual);
          TelnetStream.print("\r\n");

          TelnetStream.print("Raw flow X/Y: ");
          TelnetStream.print(leftFlowX_raw);
          TelnetStream.print(" / ");
          TelnetStream.print(leftFlowY_raw);
          TelnetStream.print("\r\n");

          TelnetStream.print("Body Vx/Vy: ");
          TelnetStream.print(leftVxBody_mps, 4);
          TelnetStream.print(" / ");
          TelnetStream.print(leftVyBody_mps, 4);
          TelnetStream.print(" m/s\r\n");

          TelnetStream.print("At VN300 Vx/Vy: ");
          TelnetStream.print(leftVxVN_mps, 4);
          TelnetStream.print(" / ");
          TelnetStream.print(leftVyVN_mps, 4);
          TelnetStream.print(" m/s\r\n");

          TelnetStream.print("Valid: ");
          TelnetStream.print(leftValid ? "YES" : "NO");
          TelnetStream.print("\r\n");

          TelnetStream.print("\r\nRight MTF01 - ID 0x30B\r\n");

          TelnetStream.print("Distance: ");
          TelnetStream.print(rightDistance_m, 3);
          TelnetStream.print(" m\r\n");

          TelnetStream.print("SQUAL: ");
          TelnetStream.print(rightSqual);
          TelnetStream.print("\r\n");

          TelnetStream.print("Raw flow X/Y: ");
          TelnetStream.print(rightFlowX_raw);
          TelnetStream.print(" / ");
          TelnetStream.print(rightFlowY_raw);
          TelnetStream.print("\r\n");

          TelnetStream.print("Body Vx/Vy: ");
          TelnetStream.print(rightVxBody_mps, 4);
          TelnetStream.print(" / ");
          TelnetStream.print(rightVyBody_mps, 4);
          TelnetStream.print(" m/s\r\n");

          TelnetStream.print("At VN300 Vx/Vy: ");
          TelnetStream.print(rightVxVN_mps, 4);
          TelnetStream.print(" / ");
          TelnetStream.print(rightVyVN_mps, 4);
          TelnetStream.print(" m/s\r\n");

          TelnetStream.print("Valid: ");
          TelnetStream.print(rightValid ? "YES" : "NO");
          TelnetStream.print("\r\n");

          TelnetStream.print("\r\nVN300 Vx/Vy: ");
          TelnetStream.print(vxVN_mps, 4);
          TelnetStream.print(" / ");
          TelnetStream.print(vyVN_mps, 4);
          TelnetStream.print(" m/s\r\n");

          TelnetStream.print("Fused Vx/Vy: ");
          TelnetStream.print(vxFused_mps, 4);
          TelnetStream.print(" / ");
          TelnetStream.print(vyFused_mps, 4);
          TelnetStream.print(" m/s\r\n");
          break;
        }
      case 6:
        {
          uint32_t totalCount = canRxFrameCount;
          uint32_t count105 = canRx105Count;
          uint32_t count30A = canRx30ACount;
          uint32_t count30B = canRx30BCount;

          uint32_t lastId = canRxLastId;
          uint8_t lastDlc = canRxLastDlc;
          uint32_t lastTime = canRxLastTime_ms;

          uint32_t now_ms = millis();

          TelnetStream.print("CAN RX total: ");
          TelnetStream.print(totalCount);
          TelnetStream.print("\r\n");

          TelnetStream.print("CAN 0x109 count: ");
          TelnetStream.print(count105);
          TelnetStream.print("\r\n");

          TelnetStream.print("CAN 0x30A count: ");
          TelnetStream.print(count30A);
          TelnetStream.print("\r\n");

          TelnetStream.print("CAN 0x30B count: ");
          TelnetStream.print(count30B);
          TelnetStream.print("\r\n");

          TelnetStream.print("Other CAN count: ");
          TelnetStream.print(
            totalCount - count105 - count30A - count30B);
          TelnetStream.print("\r\n");

          TelnetStream.print("Last CAN ID: 0x");
          TelnetStream.print(lastId, HEX);
          TelnetStream.print("\r\n");

          TelnetStream.print("Last CAN DLC: ");
          TelnetStream.print(lastDlc);
          TelnetStream.print("\r\n");

          TelnetStream.print("Last CAN frame age: ");

          if (totalCount == 0) {
            TelnetStream.print("N/A");
          } else {
            TelnetStream.print(now_ms - lastTime);
            TelnetStream.print(" ms");
          }

          TelnetStream.print("\r\n");

          break;
        }
      default:
        i = 0;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void TaskSerialPrint(void *pvParemeters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  uint16_t ins_status_temp = 0x0000;
  double lat_temp = 0.0;
  double long_temp = 0.0;
  float vbx_temp = 0.0;
  float vby_temp = 0.0;
  float vbz_temp = 0.0;
  float y_temp = 0.0;
  float p_temp = 0.0;
  float r_temp = 0.0;
  float lbax_temp = 0.0;
  float lbay_temp = 0.0;
  float lbaz_temp = 0.0;
  int8_t ty_temp = 0;
  uint8_t tmon_temp = 0;
  uint8_t td_temp = 0;
  uint8_t th_temp = 0;
  uint8_t tmin_temp = 0;
  uint8_t ts_temp = 0;
  uint16_t tf_temp = 0;
  float gbx_temp = 0.0;
  float gby_temp = 0.0;
  float gbz_temp = 0.0;
  float steerWheel_temp = 0.0;
  float steerFL_temp = 0.0;
  float steerFR_temp = 0.0;
  float slipFL_temp = 0.0;
  float slipFR_temp = 0.0;
  float slipRL_temp = 0.0;
  float slipRR_temp = 0.0;
  float mtfLeftDistance_temp = 0.0f;
  int16_t mtfLeftFlowXRaw_temp = 0;
  int16_t mtfLeftFlowYRaw_temp = 0;
  float mtfLeftVxBody_temp = 0.0f;
  float mtfLeftVyBody_temp = 0.0f;
  float mtfLeftVxVN_temp = 0.0f;
  float mtfLeftVyVN_temp = 0.0f;
  bool mtfLeftValid_temp = false;

  float mtfRightDistance_temp = 0.0f;
  int16_t mtfRightFlowXRaw_temp = 0;
  int16_t mtfRightFlowYRaw_temp = 0;
  float mtfRightVxBody_temp = 0.0f;
  float mtfRightVyBody_temp = 0.0f;
  float mtfRightVxVN_temp = 0.0f;
  float mtfRightVyVN_temp = 0.0f;
  bool mtfRightValid_temp = false;

  float vxFused_temp = 0.0f;
  float vyFused_temp = 0.0f;

  while (1) {
    xQueuePeek(INS_status, &ins_status_temp, portMAX_DELAY);
    xQueuePeek(Latitude, &lat_temp, portMAX_DELAY);
    xQueuePeek(Longitude, &long_temp, portMAX_DELAY);
    xQueuePeek(VelBodyX, &vbx_temp, portMAX_DELAY);
    xQueuePeek(VelBodyY, &vby_temp, portMAX_DELAY);
    xQueuePeek(VelBodyZ, &vbz_temp, portMAX_DELAY);
    xQueuePeek(Yaw, &y_temp, portMAX_DELAY);
    xQueuePeek(Pitch, &p_temp, portMAX_DELAY);
    xQueuePeek(Roll, &r_temp, portMAX_DELAY);
    xQueuePeek(LinBodyAccX_filt, &lbax_temp, portMAX_DELAY);
    xQueuePeek(LinBodyAccY_filt, &lbay_temp, portMAX_DELAY);
    xQueuePeek(LinBodyAccZ_filt, &lbaz_temp, portMAX_DELAY);
    xQueuePeek(TimeUtcY, &ty_temp, portMAX_DELAY);
    xQueuePeek(TimeUtcMonth, &tmon_temp, portMAX_DELAY);
    xQueuePeek(TimeUtcD, &td_temp, portMAX_DELAY);
    xQueuePeek(TimeUtcH, &th_temp, portMAX_DELAY);
    xQueuePeek(TimeUtcMin, &tmin_temp, portMAX_DELAY);
    xQueuePeek(TimeUtcS, &ts_temp, portMAX_DELAY);
    xQueuePeek(TimeUtcF, &tf_temp, portMAX_DELAY);
    xQueuePeek(GyroBodyX_filt, &gbx_temp, portMAX_DELAY);
    xQueuePeek(GyroBodyY_filt, &gby_temp, portMAX_DELAY);
    xQueuePeek(GyroBodyZ_filt, &gbz_temp, portMAX_DELAY);
    xQueuePeek(SteeringAngle_deg, &steerWheel_temp, portMAX_DELAY);
    xQueuePeek(SteerAngleFL_deg, &steerFL_temp, portMAX_DELAY);
    xQueuePeek(SteerAngleFR_deg, &steerFR_temp, portMAX_DELAY);
    xQueuePeek(SlipAngleFL_deg, &slipFL_temp, portMAX_DELAY);
    xQueuePeek(SlipAngleFR_deg, &slipFR_temp, portMAX_DELAY);
    xQueuePeek(SlipAngleRL_deg, &slipRL_temp, portMAX_DELAY);
    xQueuePeek(SlipAngleRR_deg, &slipRR_temp, portMAX_DELAY);
    xQueuePeek(MTF01_Left_Distance_m,
               &mtfLeftDistance_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Left_FlowX_raw,
               &mtfLeftFlowXRaw_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Left_FlowY_raw,
               &mtfLeftFlowYRaw_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Left_VxBody_mps,
               &mtfLeftVxBody_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Left_VyBody_mps,
               &mtfLeftVyBody_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Left_VxVN_mps,
               &mtfLeftVxVN_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Left_VyVN_mps,
               &mtfLeftVyVN_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Left_Valid,
               &mtfLeftValid_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_Distance_m,
               &mtfRightDistance_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_FlowX_raw,
               &mtfRightFlowXRaw_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_FlowY_raw,
               &mtfRightFlowYRaw_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_VxBody_mps,
               &mtfRightVxBody_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_VyBody_mps,
               &mtfRightVyBody_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_VxVN_mps,
               &mtfRightVxVN_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_VyVN_mps,
               &mtfRightVyVN_temp,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_Valid,
               &mtfRightValid_temp,
               portMAX_DELAY);

    xQueuePeek(VelBodyX_fused, &vxFused_temp, portMAX_DELAY);
    xQueuePeek(VelBodyY_fused, &vyFused_temp, portMAX_DELAY);
    Serial.print(ins_status_temp, BIN);
    Serial.print(",");
    Serial.print(lat_temp, 7);
    Serial.print(",");
    Serial.print(long_temp, 7);
    Serial.print(",");
    Serial.print(vbx_temp, 4);
    Serial.print(",");
    Serial.print(vby_temp, 4);
    Serial.print(",");
    Serial.print(vbz_temp, 4);
    Serial.print(",");
    Serial.print(y_temp, 4);
    Serial.print(",");
    Serial.print(p_temp, 4);
    Serial.print(",");
    Serial.print(r_temp, 4);
    Serial.print(",");
    Serial.print(lbax_temp, 4);
    Serial.print(",");
    Serial.print(lbay_temp, 4);
    Serial.print(",");
    Serial.print(lbaz_temp, 4);
    Serial.print(",");
    Serial.print(ty_temp);
    Serial.print(",");
    Serial.print(tmon_temp);
    Serial.print(",");
    Serial.print(td_temp);
    Serial.print(",");
    Serial.print(th_temp);
    Serial.print(",");
    Serial.print(tmin_temp);
    Serial.print(",");
    Serial.print(ts_temp);
    Serial.print(",");
    Serial.print(tf_temp);
    Serial.print(",");
    Serial.print(gbx_temp, 4);
    Serial.print(",");
    Serial.print(gby_temp, 4);
    Serial.print(",");
    Serial.print(gbz_temp, 4);
    Serial.print(",");
    Serial.print(steerWheel_temp, 4);
    Serial.print(",");
    Serial.print(steerFL_temp, 4);
    Serial.print(",");
    Serial.print(steerFR_temp, 4);
    Serial.print(",");
    Serial.print(slipFL_temp, 4);
    Serial.print(",");
    Serial.print(slipFR_temp, 4);
    Serial.print(",");
    Serial.print(slipRL_temp, 4);
    Serial.print(",");
    Serial.print(slipRR_temp, 4);
    // Left MTF01
    Serial.print(",");
    Serial.print(mtfLeftDistance_temp, 4);

    Serial.print(",");
    Serial.print(mtfLeftFlowXRaw_temp);

    Serial.print(",");
    Serial.print(mtfLeftFlowYRaw_temp);

    Serial.print(",");
    Serial.print(mtfLeftVxBody_temp, 4);

    Serial.print(",");
    Serial.print(mtfLeftVyBody_temp, 4);

    Serial.print(",");
    Serial.print(mtfLeftVxVN_temp, 4);

    Serial.print(",");
    Serial.print(mtfLeftVyVN_temp, 4);

    Serial.print(",");
    Serial.print((uint8_t)mtfLeftValid_temp);

    // Right MTF01
    Serial.print(",");
    Serial.print(mtfRightDistance_temp, 4);

    Serial.print(",");
    Serial.print(mtfRightFlowXRaw_temp);

    Serial.print(",");
    Serial.print(mtfRightFlowYRaw_temp);

    Serial.print(",");
    Serial.print(mtfRightVxBody_temp, 4);

    Serial.print(",");
    Serial.print(mtfRightVyBody_temp, 4);

    Serial.print(",");
    Serial.print(mtfRightVxVN_temp, 4);

    Serial.print(",");
    Serial.print(mtfRightVyVN_temp, 4);

    Serial.print(",");
    Serial.print((uint8_t)mtfRightValid_temp);

    // Final fused velocity
    Serial.print(",");
    Serial.print(vxFused_temp, 4);

    Serial.print(",");
    Serial.print(vyFused_temp, 4);

    Serial.print("\r\n");
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void TaskAttitudeFilt(void *pvParemeters) {
  float t1 = 0.0;
  float t2 = 0.0;
  float t3 = 0.0;
  float t4 = 0.0;
  float t5 = 0.0;
  float t6 = 0.0;
  float mag = 0.0;
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    float qx, qy, qz, qs;
    float qxf, qyf, qzf, qsf;

    xQueuePeek(QuatX, &qx, portMAX_DELAY);
    xQueuePeek(QuatY, &qy, portMAX_DELAY);
    xQueuePeek(QuatZ, &qz, portMAX_DELAY);
    xQueuePeek(QuatS, &qs, portMAX_DELAY);

    if (!isfinite(qx) || !isfinite(qy) || !isfinite(qz) || !isfinite(qs)) {
      continue;
    }

    xQueuePeek(QuatX_filt, &qxf, portMAX_DELAY);
    xQueuePeek(QuatY_filt, &qyf, portMAX_DELAY);
    xQueuePeek(QuatZ_filt, &qzf, portMAX_DELAY);
    xQueuePeek(QuatS_filt, &qsf, portMAX_DELAY);

    if (!isfinite(qxf)) qxf = qx;
    if (!isfinite(qyf)) qyf = qy;
    if (!isfinite(qzf)) qzf = qz;
    if (!isfinite(qsf)) qsf = qs;

    t3 = qx * filt_coeff[0] + qxf * filt_coeff[1];
    t4 = qy * filt_coeff[0] + qyf * filt_coeff[1];
    t5 = qz * filt_coeff[0] + qzf * filt_coeff[1];
    t6 = qs * filt_coeff[0] + qsf * filt_coeff[1];

    mag = sqrtf(t3 * t3 + t4 * t4 + t5 * t5 + t6 * t6);

    if (isfinite(mag) && mag > 1e-6f) {
      t3 /= mag;
      t4 /= mag;
      t5 /= mag;
      t6 /= mag;

      xQueueOverwrite(QuatX_filt, &t3);
      xQueueOverwrite(QuatY_filt, &t4);
      xQueueOverwrite(QuatZ_filt, &t5);
      xQueueOverwrite(QuatS_filt, &t6);
    }
    xQueuePeek(LinBodyAccX, &t1, portMAX_DELAY);
    xQueuePeek(LinBodyAccX_filt, &t2, portMAX_DELAY);
    if (!isfinite(t1)) continue;
    if (!isfinite(t2)) t2 = t1;
    t3 = t1 * filt_coeff[0] + t2 * filt_coeff[1];
    xQueueOverwrite(LinBodyAccX_filt, &t3);
    xQueuePeek(LinBodyAccY, &t1, portMAX_DELAY);
    xQueuePeek(LinBodyAccY_filt, &t2, portMAX_DELAY);
    if (!isfinite(t1)) continue;
    if (!isfinite(t2)) t2 = t1;
    t3 = t1 * filt_coeff[0] + t2 * filt_coeff[1];
    xQueueOverwrite(LinBodyAccY_filt, &t3);
    xQueuePeek(LinBodyAccZ, &t1, portMAX_DELAY);
    xQueuePeek(LinBodyAccZ_filt, &t2, portMAX_DELAY);
    if (!isfinite(t1)) continue;
    if (!isfinite(t2)) t2 = t1;
    t3 = t1 * filt_coeff[0] + t2 * filt_coeff[1];
    xQueueOverwrite(LinBodyAccZ_filt, &t3);
  }
}

void TaskIMUFilt(void *pvParemeters) {
  float t1 = 0.0;
  float t2 = 0.0;
  float t3 = 0.0;
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    filterOne(GyroBodyX, GyroBodyX_filt);
    filterOne(GyroBodyY, GyroBodyY_filt);
    filterOne(GyroBodyZ, GyroBodyZ_filt);
  }
}

void TaskCANComms(void *pvParemeters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  uint64_t t1 = 0;
  uint64_t t2 = 0;
  uint8_t t3 = 0;
  uint32_t t4 = 0;
  uint32_t t5 = 0;
  uint16_t t6 = 0;
  uint16_t t7 = 0;
  uint8_t i = 0;
  uint8_t j = 0;
  double d1 = 0.0;
  float f1 = 0.0;
  float f2 = 0.0;
  CanFrame TxFrame = { 0 };

  while (1) {
    switch (i) {
      case 0:
        xQueuePeek(Latitude, &d1, portMAX_DELAY);
        d1 = (d1 - LATITUDE_OFFSET) / LATITUDE_SCALE;
        TxFrame.identifier = 0x250;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t1, &d1, sizeof(double));
        t1 = (uint64_t)(int64_t)llround(d1);
        for (j = 0; j < 8; j++) {
          t2 = (t1 >> ((7 - j) * 8)) & 0x00000000000000FF;
          t3 = (uint8_t)(t2);
          TxFrame.data[j] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 1;
        break;
      case 1:
        xQueuePeek(Longitude, &d1, portMAX_DELAY);
        d1 = (d1 - LONGITUDE_OFFSET) / LONGITUDE_SCALE;
        TxFrame.identifier = 0x251;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t1, &d1, sizeof(double));
        t1 = (uint64_t)(int64_t)llround(d1);
        for (j = 0; j < 8; j++) {
          t2 = (t1 >> ((7 - j) * 8)) & 0x00000000000000FF;
          t3 = (uint8_t)(t2);
          TxFrame.data[j] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 2;
        break;
      case 2:
        xQueuePeek(Altitude, &d1, portMAX_DELAY);
        d1 = (d1 - ALTITUDE_OFFSET) / ALTITUDE_SCALE;
        TxFrame.identifier = 0x252;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t1, &d1, sizeof(double));
        t1 = (uint64_t)(int64_t)llround(d1);
        for (j = 0; j < 8; j++) {
          t2 = (t1 >> ((7 - j) * 8)) & 0x00000000000000FF;
          t3 = (uint8_t)(t2);
          TxFrame.data[j] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 3;
        break;
      case 3:
        xQueuePeek(VelBodyX, &f1, portMAX_DELAY);
        f1 = (f1 - VELBODYX_OFFSET) / VELBODYX_SCALE;
        xQueuePeek(VelBodyY, &f2, portMAX_DELAY);
        f2 = (f2 - VELBODYY_OFFSET) / VELBODYY_SCALE;
        TxFrame.identifier = 0x253;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t4, &f1, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f1);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j] = t3;
        }
        //memcpy(&t4, &f2, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f2);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j + 4] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 4;
        break;
      case 4:
        xQueuePeek(Yaw, &f1, portMAX_DELAY);
        f1 = (f1 - YAW_OFFSET) / YAW_SCALE;
        xQueuePeek(Pitch, &f2, portMAX_DELAY);
        f2 = (f2 - PITCH_OFFSET) / PITCH_SCALE;
        TxFrame.identifier = 0x254;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t4, &f1, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f1);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j] = t3;
        }
        //memcpy(&t4, &f2, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f2);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j + 4] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 5;
        break;
      case 5:
        xQueuePeek(Roll, &f1, portMAX_DELAY);
        f1 = (f1 - ROLL_OFFSET) / ROLL_SCALE;
        xQueuePeek(LinBodyAccZ_filt, &f2, portMAX_DELAY);
        f2 = (f2 - LINBODYACCZ_OFFSET) / LINBODYACCZ_SCALE;
        TxFrame.identifier = 0x255;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t4, &f1, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f1);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j] = t3;
        }
        //memcpy(&t4, &f2, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f2);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j + 4] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 6;
        break;
      case 6:
        xQueuePeek(LinBodyAccX_filt, &f1, portMAX_DELAY);
        f1 = (f1 - LINBODYACCX_OFFSET) / LINBODYACCX_SCALE;
        xQueuePeek(LinBodyAccY_filt, &f2, portMAX_DELAY);
        f2 = (f2 - LINBODYACCY_OFFSET) / LINBODYACCY_SCALE;
        TxFrame.identifier = 0x256;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t4, &f1, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f1);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j] = t3;
        }
        //memcpy(&t4, &f2, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f2);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j + 4] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 7;
        break;
      case 7:
        xQueuePeek(QuatX_filt, &f1, portMAX_DELAY);
        f1 = (f1 - QUATX_OFFSET) / QUATX_SCALE;
        xQueuePeek(QuatY_filt, &f2, portMAX_DELAY);
        f2 = (f2 - QUATY_OFFSET) / QUATY_SCALE;
        TxFrame.identifier = 0x257;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t4, &f1, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f1);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j] = t3;
        }
        //memcpy(&t4, &f2, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f2);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j + 4] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 8;
        break;
      case 8:
        xQueuePeek(QuatZ_filt, &f1, portMAX_DELAY);
        f1 = (f1 - QUATZ_OFFSET) / QUATZ_SCALE;
        xQueuePeek(QuatS_filt, &f2, portMAX_DELAY);
        f2 = (f2 - QUATS_OFFSET) / QUATS_SCALE;
        TxFrame.identifier = 0x258;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t4, &f1, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f1);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j] = t3;
        }
        //memcpy(&t4, &f2, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f2);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j + 4] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 9;
        break;
      case 9:
        xQueuePeek(TimeUtcY, &t3, portMAX_DELAY);
        TxFrame.identifier = 0x259;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        TxFrame.data[0] = t3;
        xQueuePeek(TimeUtcMonth, &t3, portMAX_DELAY);
        TxFrame.data[1] = t3;
        xQueuePeek(TimeUtcD, &t3, portMAX_DELAY);
        TxFrame.data[2] = t3;
        xQueuePeek(TimeUtcH, &t3, portMAX_DELAY);
        TxFrame.data[3] = t3;
        xQueuePeek(TimeUtcMin, &t3, portMAX_DELAY);
        TxFrame.data[4] = t3;
        xQueuePeek(TimeUtcS, &t3, portMAX_DELAY);
        TxFrame.data[5] = t3;
        xQueuePeek(TimeUtcF, &t6, portMAX_DELAY);
        t3 = (uint8_t)((t6 >> 8) & 0x00FF);
        TxFrame.data[6] = t3;
        t3 = (uint8_t)(t6 & 0x00FF);
        TxFrame.data[7] = t3;
        ESP32Can.writeFrame(TxFrame, 0);
        i = 10;
        break;
      case 10:
        xQueuePeek(GyroBodyX_filt, &f1, portMAX_DELAY);
        f1 = ((f1 * RAD_TO_DEG) - GYROBODYX_OFFSET) / GYROBODYX_SCALE;
        xQueuePeek(GyroBodyY_filt, &f2, portMAX_DELAY);
        f2 = ((f2 * RAD_TO_DEG) - GYROBODYY_OFFSET) / GYROBODYY_SCALE;
        TxFrame.identifier = 0x25A;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        //memcpy(&t4, &f1, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f1);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j] = t3;
        }
        //memcpy(&t4, &f2, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f2);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j + 4] = t3;
        }
        ESP32Can.writeFrame(TxFrame, 0);
        i = 11;
        break;
      case 11:
        xQueuePeek(GyroBodyZ_filt, &f1, portMAX_DELAY);
        f1 = ((f1 * RAD_TO_DEG) - GYROBODYZ_OFFSET) / GYROBODYZ_SCALE;
        xQueuePeek(INS_status, &t6, portMAX_DELAY);
        TxFrame.identifier = 0x25B;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 6;
        //memcpy(&t4, &f1, sizeof(float));
        t4 = (uint32_t)(int32_t)lroundf(f1);
        for (j = 0; j < 4; j++) {
          t5 = (t4 >> ((3 - j) * 8)) & 0x000000FF;
          t3 = (uint8_t)(t5);
          TxFrame.data[j] = t3;
        }
        t3 = (uint8_t)((t6 >> 8) & 0x00FF);
        TxFrame.data[4] = t3;
        t3 = (uint8_t)(t6 & 0x00FF);
        TxFrame.data[5] = t3;
        ESP32Can.writeFrame(TxFrame, 0);
        //TelnetStream.print("/r/nHello/r/n");
        i = 12;
        break;
      case 12:
        {
          float slipFL = 0.0f;
          float slipFR = 0.0f;
          float slipRL = 0.0f;
          float slipRR = 0.0f;

          int16_t rawFL = 0;
          int16_t rawFR = 0;
          int16_t rawRL = 0;
          int16_t rawRR = 0;

          xQueuePeek(SlipAngleFL_deg, &slipFL, portMAX_DELAY);
          xQueuePeek(SlipAngleFR_deg, &slipFR, portMAX_DELAY);
          xQueuePeek(SlipAngleRL_deg, &slipRL, portMAX_DELAY);
          xQueuePeek(SlipAngleRR_deg, &slipRR, portMAX_DELAY);

          slipFL = constrain(slipFL, -45.0f, 45.0f);
          slipFR = constrain(slipFR, -45.0f, 45.0f);
          slipRL = constrain(slipRL, -45.0f, 45.0f);
          slipRR = constrain(slipRR, -45.0f, 45.0f);

          rawFL = (int16_t)lroundf((slipFL - SLIPANGLE_OFFSET) / SLIPANGLE_SCALE);
          rawFR = (int16_t)lroundf((slipFR - SLIPANGLE_OFFSET) / SLIPANGLE_SCALE);
          rawRL = (int16_t)lroundf((slipRL - SLIPANGLE_OFFSET) / SLIPANGLE_SCALE);
          rawRR = (int16_t)lroundf((slipRR - SLIPANGLE_OFFSET) / SLIPANGLE_SCALE);

          TxFrame.identifier = 0x25C;
          TxFrame.extd = 0;
          TxFrame.data_length_code = 8;

          TxFrame.data[0] = (uint8_t)(((uint16_t)rawFL >> 8) & 0xFF);
          TxFrame.data[1] = (uint8_t)((uint16_t)rawFL & 0xFF);

          TxFrame.data[2] = (uint8_t)(((uint16_t)rawFR >> 8) & 0xFF);
          TxFrame.data[3] = (uint8_t)((uint16_t)rawFR & 0xFF);

          TxFrame.data[4] = (uint8_t)(((uint16_t)rawRL >> 8) & 0xFF);
          TxFrame.data[5] = (uint8_t)((uint16_t)rawRL & 0xFF);

          TxFrame.data[6] = (uint8_t)(((uint16_t)rawRR >> 8) & 0xFF);
          TxFrame.data[7] = (uint8_t)((uint16_t)rawRR & 0xFF);

          ESP32Can.writeFrame(TxFrame, 0);

          i = 0;
          break;
        }
      default:
        i = 0;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void TaskCANRx(void *pvParameters) {
  CanFrame rxFrame = { 0 };

  while (1) {
    if (ESP32Can.readFrame(rxFrame, 10)) {

      canRxFrameCount++;
      canRxLastId = rxFrame.identifier;
      canRxLastDlc = rxFrame.data_length_code;
      canRxLastTime_ms = millis();

      if (rxFrame.identifier == 0x109) {
        canRx105Count++;
      } else if (rxFrame.identifier == MTF01_LEFT_CAN_ID) {
        canRx30ACount++;
      } else if (rxFrame.identifier == MTF01_RIGHT_CAN_ID) {
        canRx30BCount++;
      }

      if (rxFrame.identifier == 0x109 && !rxFrame.extd && rxFrame.data_length_code >= 4) {

        const uint16_t rawSteeringBits =
          ((uint16_t)rxFrame.data[2] << 8) | (uint16_t)rxFrame.data[3];
        const int16_t rawSteering = (int16_t)rawSteeringBits;

        const float steeringWheel_deg =
          ((float)rawSteering * STEERINGANGLEDEG_SCALE) + STEERINGANGLEDEG_OFFSET;

        if (isfinite(steeringWheel_deg)) {
          if (!steeringFiltInit) {
            steeringWheelFilt_deg = steeringWheel_deg;
            steeringFiltInit = true;
          } else {
            steeringWheelFilt_deg =
              k * steeringWheel_deg + (1.0f - k) * steeringWheelFilt_deg;
          }

          float steerFL_deg = 0.0f;
          float steerFR_deg = 0.0f;

          steering_wheel_to_front_angles(
            steeringWheelFilt_deg,
            &steerFL_deg,
            &steerFR_deg);

          xQueueOverwrite(SteeringAngle_deg, &steeringWheelFilt_deg);
          xQueueOverwrite(SteerAngleFL_deg, &steerFL_deg);
          xQueueOverwrite(SteerAngleFR_deg, &steerFR_deg);
        }
      }

      else if ((rxFrame.identifier == MTF01_LEFT_CAN_ID || rxFrame.identifier == MTF01_RIGHT_CAN_ID) && rxFrame.data_length_code == 8) {

        uint16_t distanceRaw =
          ((uint16_t)rxFrame.data[0]) | ((uint16_t)rxFrame.data[1] << 8);

        int16_t flowXRaw =
          (int16_t)(((uint16_t)rxFrame.data[2]) | ((uint16_t)rxFrame.data[3] << 8));

        int16_t flowYRaw =
          (int16_t)(((uint16_t)rxFrame.data[4]) | ((uint16_t)rxFrame.data[5] << 8));

        uint8_t squal = rxFrame.data[6];
        uint8_t flowQuality = rxFrame.data[7];

        float distance_m = (float)distanceRaw * 0.001f;

        /*
          MTF values are cm/s at 1 m.

          Dividing by 100 gives the angular-equivalent flow in rad/s.
          Multiplying by actual height gives linear velocity in m/s.
        */
        float flowX_radps = (float)flowXRaw * 0.01f;
        float flowY_radps = (float)flowYRaw * 0.01f;

        float vxMtf_mps = flowX_radps * distance_m;
        float vyMtf_mps = flowY_radps * distance_m;

        float vxBody_mps = 0.0f;
        float vyBody_mps = 0.0f;

        rotateMTFToBody(
          vxMtf_mps,
          vyMtf_mps,
          &vxBody_mps,
          &vyBody_mps);

        float flowMagnitude_radps =
          sqrtf(flowX_radps * flowX_radps + flowY_radps * flowY_radps);

        bool valid =
          isfinite(distance_m) && isfinite(vxBody_mps) && isfinite(vyBody_mps) && distance_m >= MTF01_MIN_HEIGHT_M && distance_m <= MTF01_MAX_HEIGHT_M && flowMagnitude_radps <= MTF01_MAX_FLOW_RADPS && flowQuality >= MTF01_MIN_FLOW_QUALITY;

        if (rxFrame.identifier == MTF01_LEFT_CAN_ID) {
          xQueueOverwrite(MTF01_Left_Distance_m, &distance_m);
          xQueueOverwrite(MTF01_Left_FlowX_raw, &flowXRaw);
          xQueueOverwrite(MTF01_Left_FlowY_raw, &flowYRaw);
          xQueueOverwrite(MTF01_Left_VxBody_mps, &vxBody_mps);
          xQueueOverwrite(MTF01_Left_VyBody_mps, &vyBody_mps);
          xQueueOverwrite(MTF01_Left_Valid, &valid);
          xQueueOverwrite(MTF01_Left_SQUAL, &squal);

          mtf01LeftLastRx_ms = millis();
        } else {
          xQueueOverwrite(MTF01_Right_Distance_m, &distance_m);
          xQueueOverwrite(MTF01_Right_FlowX_raw, &flowXRaw);
          xQueueOverwrite(MTF01_Right_FlowY_raw, &flowYRaw);
          xQueueOverwrite(MTF01_Right_VxBody_mps, &vxBody_mps);
          xQueueOverwrite(MTF01_Right_VyBody_mps, &vyBody_mps);
          xQueueOverwrite(MTF01_Right_Valid, &valid);
          xQueueOverwrite(MTF01_Right_SQUAL, &squal);

          mtf01RightLastRx_ms = millis();
        }
      }
    }

    taskYIELD();
  }
}

void TaskSlipAngle(void *pvParameters) {
  float vxVN = 0.0f;
  float vyVN = 0.0f;
  float yawRate = 0.0f;

  float steerFL_deg = 0.0f;
  float steerFR_deg = 0.0f;

  float mtfLeftVxBody = 0.0f;
  float mtfLeftVyBody = 0.0f;
  float mtfRightVxBody = 0.0f;
  float mtfRightVyBody = 0.0f;

  bool mtfLeftValid = false;
  bool mtfRightValid = false;

  float mtfLeftVxVN = 0.0f;
  float mtfLeftVyVN = 0.0f;
  float mtfRightVxVN = 0.0f;
  float mtfRightVyVN = 0.0f;

  float mtfVxVN = 0.0f;
  float mtfVyVN = 0.0f;

  float vxFused = 0.0f;
  float vyFused = 0.0f;

  float vyF = 0.0f;
  float vyR = 0.0f;

  float vxFL = 0.0f;
  float vxFR = 0.0f;
  float vxRL = 0.0f;
  float vxRR = 0.0f;

  float alphaFL_deg = 0.0f;
  float alphaFR_deg = 0.0f;
  float alphaRL_deg = 0.0f;
  float alphaRR_deg = 0.0f;

  const float a = VN300_A_FROM_FRONT_M;
  const float b = VN300_B_FROM_FL_M;

  const float rearLeftYFromFL =
    (FRONT_TRACKWIDTH_M - REAR_TRACKWIDTH_M) * 0.5f;

  const float rearRightYFromFL =
    rearLeftYFromFL + REAR_TRACKWIDTH_M;

  while (1) {
    xQueuePeek(VelBodyX, &vxVN, portMAX_DELAY);
    xQueuePeek(VelBodyY, &vyVN, portMAX_DELAY);
    xQueuePeek(GyroBodyZ_filt, &yawRate, portMAX_DELAY);

    xQueuePeek(SteerAngleFL_deg, &steerFL_deg, portMAX_DELAY);
    xQueuePeek(SteerAngleFR_deg, &steerFR_deg, portMAX_DELAY);

    xQueuePeek(MTF01_Left_VxBody_mps,
               &mtfLeftVxBody,
               portMAX_DELAY);

    xQueuePeek(MTF01_Left_VyBody_mps,
               &mtfLeftVyBody,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_VxBody_mps,
               &mtfRightVxBody,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_VyBody_mps,
               &mtfRightVyBody,
               portMAX_DELAY);

    xQueuePeek(MTF01_Left_Valid,
               &mtfLeftValid,
               portMAX_DELAY);

    xQueuePeek(MTF01_Right_Valid,
               &mtfRightValid,
               portMAX_DELAY);

    /*
      Invalidate stale CAN data.
    */
    uint32_t now_ms = millis();

    if ((now_ms - mtf01LeftLastRx_ms) > MTF01_TIMEOUT_MS) {
      mtfLeftValid = false;
    }

    if ((now_ms - mtf01RightLastRx_ms) > MTF01_TIMEOUT_MS) {
      mtfRightValid = false;
    }

    if (!isfinite(vxVN) || !isfinite(vyVN) || !isfinite(yawRate)) {

      vTaskDelay(20 / portTICK_PERIOD_MS);
      continue;
    }

    /*
      Rigid-body velocity at sensor:

        Vx_sensor = Vx_VN - r*y_sensor
        Vy_sensor = Vy_VN + r*x_sensor

      Translate sensor velocity back to VN300:

        Vx_VN = Vx_sensor + r*y_sensor
        Vy_VN = Vy_sensor - r*x_sensor

      Left MTF position  = (+x1, -y)
      Right MTF position = (+x1, +y)
    */

    mtfLeftVxVN =
      mtfLeftVxBody - yawRate * MTF01_Y_M;

    mtfLeftVyVN =
      mtfLeftVyBody - yawRate * MTF01_X1_M;

    mtfRightVxVN =
      mtfRightVxBody + yawRate * MTF01_Y_M;

    mtfRightVyVN =
      mtfRightVyBody - yawRate * MTF01_X1_M;

    xQueueOverwrite(MTF01_Left_VxVN_mps, &mtfLeftVxVN);
    xQueueOverwrite(MTF01_Left_VyVN_mps, &mtfLeftVyVN);
    xQueueOverwrite(MTF01_Right_VxVN_mps, &mtfRightVxVN);
    xQueueOverwrite(MTF01_Right_VyVN_mps, &mtfRightVyVN);

    /*
      Combine the two MTF estimates.

      If both are valid, average them.
      If only one is valid, use the valid one.
      If neither is valid, use VN300 only.
    */
    bool mtfAvailable = false;

    if (mtfLeftValid && mtfRightValid) {
      mtfVxVN = 0.5f * (mtfLeftVxVN + mtfRightVxVN);
      mtfVyVN = 0.5f * (mtfLeftVyVN + mtfRightVyVN);
      mtfAvailable = true;
    } else if (mtfLeftValid) {
      mtfVxVN = mtfLeftVxVN;
      mtfVyVN = mtfLeftVyVN;
      mtfAvailable = true;
    } else if (mtfRightValid) {
      mtfVxVN = mtfRightVxVN;
      mtfVyVN = mtfRightVyVN;
      mtfAvailable = true;
    }

    if (mtfAvailable && isfinite(mtfVxVN) && isfinite(mtfVyVN)) {

      vxFused =
        MTF01_FUSION_WEIGHT * mtfVxVN + (1.0f - MTF01_FUSION_WEIGHT) * vxVN;

      vyFused =
        MTF01_FUSION_WEIGHT * mtfVyVN + (1.0f - MTF01_FUSION_WEIGHT) * vyVN;
    } else {
      vxFused = vxVN;
      vyFused = vyVN;
    }

    xQueueOverwrite(VelBodyX_fused, &vxFused);
    xQueueOverwrite(VelBodyY_fused, &vyFused);

    /*
      Wheel velocities using fused velocity at VN300.
    */
    vyF = vyFused + yawRate * a;
    vyR = vyFused + yawRate * (a - WHEELBASE_M);

    vxFL = vxFused - yawRate * (-b);

    vxFR =
      vxFused - yawRate * (FRONT_TRACKWIDTH_M - b);

    vxRL =
      vxFused - yawRate * (rearLeftYFromFL - b);

    vxRR =
      vxFused - yawRate * (rearRightYFromFL - b);

    if (fabsf(vxFL) > MIN_SLIP_VX_MPS && fabsf(vxFR) > MIN_SLIP_VX_MPS && fabsf(vxRL) > MIN_SLIP_VX_MPS && fabsf(vxRR) > MIN_SLIP_VX_MPS) {

      const float steerFL_rad = steerFL_deg * DEG_TO_RAD;
      const float steerFR_rad = steerFR_deg * DEG_TO_RAD;

      const float betaFL = atan2f(vyF, vxFL);
      const float betaFR = atan2f(vyF, vxFR);
      const float betaRL = atan2f(vyR, vxRL);
      const float betaRR = atan2f(vyR, vxRR);

      alphaFL_deg =
        (betaFL + steerFL_rad) * RAD_TO_DEG;

      alphaFR_deg =
        (betaFR + steerFR_rad) * RAD_TO_DEG;

      alphaRL_deg = betaRL * RAD_TO_DEG;
      alphaRR_deg = betaRR * RAD_TO_DEG;

      if (isfinite(alphaFL_deg) && isfinite(alphaFR_deg) && isfinite(alphaRL_deg) && isfinite(alphaRR_deg) && fabsf(alphaFL_deg) <= MAX_ABS_SLIP_DEG && fabsf(alphaFR_deg) <= MAX_ABS_SLIP_DEG && fabsf(alphaRL_deg) <= MAX_ABS_SLIP_DEG && fabsf(alphaRR_deg) <= MAX_ABS_SLIP_DEG) {

        xQueueOverwrite(SlipAngleFL_deg, &alphaFL_deg);
        xQueueOverwrite(SlipAngleFR_deg, &alphaFR_deg);
        xQueueOverwrite(SlipAngleRL_deg, &alphaRL_deg);
        xQueueOverwrite(SlipAngleRR_deg, &alphaRR_deg);
      }
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

//hello
void loop() {
  vTaskDelay(10000 / portTICK_PERIOD_MS);
}
