#include <TelnetStream.h>
#include <TelnetPrint.h>
#include "OTA.h"

#define BINARY_OUTPUT_RATE_DIVIDER 10
float RFR[3][3] = { { 1.0, 0, 0 }, { 0, 1.0, 0 }, { 0, 0, 1.0 } };
float GNSSA_offset[3] = { 0.0, 1.0, 0 };
float GNSSB_baseline[6] = { -1.0, 1.0, 0, 0.0254, 0.0254, 0.0254 };

void TaskVN300Config(void *pvParameters);
void TaskVN300CheckStatus(void *pvParameters);
void TaskVN300Update(void *pvParameters);
void TaskOTAHandleAlive(void *pvParameters);
void TaskTelnetPrint(void *pvParameters);

TaskHandle_t VN300_Config_Task_Handle;
TaskHandle_t VN300_Check_Status_Task_Handle;
TaskHandle_t VN300_Update_Task_Handle;
TaskHandle_t OTA_Handle_Alive_Task_Handle;
TaskHandle_t Telnet_Print_Task_Handle;

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


void setup() {
  pinMode(2, OUTPUT);
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
  TimeUtcMin = xQueueCreate(1, sizeof(int8_t));
  TimeUtcS = xQueueCreate(1, sizeof(uint8_t));
  TimeUtcF = xQueueCreate(1, sizeof(uint16_t));
  GyroBodyX = xQueueCreate(1, sizeof(float));
  GyroBodyY = xQueueCreate(1, sizeof(float));
  GyroBodyZ = xQueueCreate(1, sizeof(float));

  uint8_t t1 = 0;
  float t2 = 0.0;
  double t3 = 0.0;
  uint16_t t4 = 0;

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

  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
  xTaskCreatePinnedToCore(TaskVN300Config, "Initial config task", 2048, NULL, 1, &VN300_Config_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskVN300CheckStatus, "Check INS status", 2048, NULL, 1, &VN300_Check_Status_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskVN300Update, "VN300 Update values", 4096, NULL, 1, &VN300_Update_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskOTAHandleAlive, "OTA handle", 16384, NULL, 1, &OTA_Handle_Alive_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskTelnetPrint, "Debug print", 8192, NULL, 1, &Telnet_Print_Task_Handle, 1);

  delay(1000);
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
        vTaskDelay(20/portTICK_PERIOD_MS);
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

  Serial.println(rx);
  return true;
}

bool crc_calc(String &cmd, uint16_t crc_rec) {
  uint8_t i = 0;
  uint16_t crc = 0;
  for (i = 0; i < cmd.length(); i++) {
    crc = (uint8_t)(crc >> 8) | (crc << 8);
    crc ^= cmd[i];
    crc ^= (uint8_t)(crc & 0xFF) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00FF) << 5;
  }
  return (crc == crc_rec);
}

void TaskVN300Config(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  String c[9];
  c[0] = "$VNWRG,06,0";
  c[1] = "$VNWRG,05,115200";
  c[2] = "$VNWRG,26";
  c[3] = "$VNWRG,57";
  c[4] = "$VNWRG,93";
  c[5] = "$VNRRG,04";
  c[6] = "$VNWRG,75,2,";
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
      Serial.println("READY");
      xTaskNotifyGive(VN300_Update_Task_Handle);
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
        if (temp = 0x00001) {
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
        ins_status &= 0x0177;
        //Serial.println(ins_status,BIN);
        c = Serial2.read();
        crc = (uint16_t)(c & 0x00FF);
        c = Serial2.read();
        crc |= (uint16_t)(c << 8);
        xQueueOverwrite(INS_status, &ins_status);
        if (((ins_status & 0x0003) >= 0) && ((ins_status | 0x018B) == 0x018F)) {
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
  String cm[6];
  char c = 0x00;
  uint8_t i = 0;
  String msg = "";
  uint8_t t1 = 0;
  uint16_t t2 = 0;
  uint8_t buf[8] = {0,0,0,0,0,0,0,0};
  uint16_t type = 0;
  uint16_t crc = 0;
  double t5 = 0.0;
  float t6 = 0.0;
  bool check = false;
  cm[1] = "$VNWRG,75,2,";
  cm[1] += String(BINARY_OUTPUT_RATE_DIVIDER) + ",20,000B";
  cm[2] = "$VNWRG,76,2,";
  cm[2] += String(BINARY_OUTPUT_RATE_DIVIDER) + ",10,0046";
  cm[3] = "$VNWRG,77,2,";
  cm[3] += String(BINARY_OUTPUT_RATE_DIVIDER) + ",06,0040,0400";
  cm[0] = "$VNWRG,06,0";
  cm[4] = "$VNWNV";
  cm[5] = "$VNRST";
  for (uint8_t j = 0; j < 6; j++) {
    append_ascii_checksum(cm[j]);
  }
  for (uint8_t j = 0; j < 6; j++) {
    bool txs = send_message(cm[j]);
  }
  Serial.println("Binary registers configured");
  vTaskDelay(10 / portTICK_PERIOD_MS);
  while (1) {
    switch (i) {
      case 0:
        msg = "";
        if(Serial2.available()){
        c = Serial2.read();
        }
        if (c == 0xFA) {
          i = 1;
          msg += String(c);
          vTaskDelay(10/portTICK_PERIOD_MS);
        } else {
          i = 0;
        }
        break;
      case 1:
        c = Serial2.read();
        if (c == 0x20) {
          i = 2;
          msg += String(c);
        } else if (c == 0x10) {
          i = 3;
          msg += String(c);
        } else if (c == 0x06) {
          i = 4;
          msg += String(c);
        } else {
          i = 0;
          msg = "";
        }
        break;
      case 2:
        c = Serial2.read();
        type = (uint16_t)(c & 0x00FF);
        c = Serial2.read();
        type |= (uint16_t)(c << 8);
        if (type == 0x000B) {
          msg += String((char)(type & 0x00FF));
          msg += String((char)((type >> 8) & 0x00FF));
          for (uint8_t j = 0; j < 2; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t2,buf,sizeof(uint16_t));
          t2 &= 0x0177;
          xQueueOverwrite(INS_status, &t2);
          for (uint8_t j = 0; j < 8; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t5,buf,sizeof(double));
          xQueueOverwrite(Latitude, &t5);
          for (uint8_t j = 0; j < 8; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t5,buf,sizeof(double));
          xQueueOverwrite(Longitude, &t5);
          for (uint8_t j = 0; j < 8; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t5,buf,sizeof(double));
          xQueueOverwrite(Altitude, &t5);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(VelBodyX, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(VelBodyY, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(VelBodyZ, &t6);
          c = Serial2.read();
          crc = (uint16_t)(c << 8) & 0xFF00;
          c = Serial2.read();
          crc |= (uint16_t)(c);
          check = crc_calc(msg, crc);
        } else {
          while (Serial2.available()) {
            c = Serial2.read();
          }
        }
        i = 0;
        break;
      case 3:
        c = Serial2.read();
        type = (uint16_t)(c & 0x00FF);
        c = Serial2.read();
        type |= (uint16_t)(c << 8);
        if (type == 0x0046) {
          msg += String((char)(type & 0x00FF));
          msg += String((char)((type >> 8) & 0x00FF));
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(Yaw, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(Pitch, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(Roll, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(QuatX, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(QuatY, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(QuatZ, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(QuatS, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(LinBodyAccX, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(LinBodyAccY, &t6);
          for (uint8_t j = 0; j < 4; j++) {
            c = Serial2.read();
            msg += String(c);
            buf[j] = c;
          }
          memcpy(&t6,buf,sizeof(float));
          xQueueOverwrite(LinBodyAccZ, &t6);

          c = Serial2.read();
          crc = (uint16_t)(c << 8) & 0xFF00;
          c = Serial2.read();
          crc |= (uint16_t)(c);
          check = crc_calc(msg, crc);
        } else {
          while (Serial2.available()) {
            c = Serial2.read();
          }
        }
        i = 0;
        break;
      case 4:
        c = Serial2.read();
        type = (uint16_t)(c & 0x00FF);
        c = Serial2.read();
        type |= (uint16_t)(c << 8);
        if (type == 0x0040) {
          msg += String((char)(type & 0x00FF));
          msg += String((char)((type >> 8) & 0x00FF));
          c = Serial2.read();
          type = (uint16_t)(c & 0x00FF);
          c = Serial2.read();
          type |= (uint16_t)(c << 8);
          if (type == 0x0400) {
            msg += String((char)(type & 0x00FF));
            msg += String((char)((type >> 8) & 0x00FF));
            c = Serial2.read();
            xQueueOverwrite(TimeUtcY, &c);
            msg += String(c);
            c = Serial2.read();
            xQueueOverwrite(TimeUtcMonth, &c);
            msg += String(c);
            c = Serial2.read();
            xQueueOverwrite(TimeUtcD, &c);
            msg += String(c);
            c = Serial2.read();
            xQueueOverwrite(TimeUtcH, &c);
            msg += String(c);
            c = Serial2.read();
            xQueueOverwrite(TimeUtcMin, &c);
            msg += String(c);
            c = Serial2.read();
            xQueueOverwrite(TimeUtcS, &c);
            msg += String(c);
            for (uint8_t j = 0; j < 2; j++) {
              c = Serial2.read();
              msg += String(c);
              buf[j] = c;
            }
            memcpy(&t2,buf,sizeof(uint16_t));
            xQueueOverwrite(TimeUtcF, &t2);
            for (uint8_t j = 0; j < 4; j++) {
              c = Serial2.read();
              msg += String(c);
              buf[j] = c;
            }
            memcpy(&t6,buf,sizeof(float));
            xQueueOverwrite(GyroBodyX, &t6);
            for (uint8_t j = 0; j < 4; j++) {
              c = Serial2.read();
              msg += String(c);
              buf[j] = c;
            }
            memcpy(&t6,buf,sizeof(float));
            xQueueOverwrite(GyroBodyY, &t6);
            for (uint8_t j = 0; j < 4; j++) {
              c = Serial2.read();
              msg += String(c);
              buf[j] = c;
            }
            memcpy(&t6,buf,sizeof(float));
            xQueueOverwrite(GyroBodyZ, &t6);
            c = Serial2.read();
            crc = (uint16_t)(c << 8) & 0xFF00;
            c = Serial2.read();
            crc |= (uint16_t)(c);
            check = crc_calc(msg, crc);
          } else {
            while (Serial2.available()) {
              c = Serial2.read();
            }
          }
        } else {
          while (Serial2.available()) {
            c = Serial2.read();
          }
        }
        i = 0;
        break;
      default:
        i = 0;
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
  xTaskNotifyGive(VN300_Config_Task_Handle);
  while (1) {
    ArduinoOTA.handle();
    vTaskDelay(93 / portTICK_PERIOD_MS);
    digitalWrite(2, !digitalRead(2));
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
    while(TelnetStream.available()){
      temp = TelnetStream.read();
      if ((uint8_t)(temp) >= 48 && (uint8_t)(temp) <= 51) {
        c = temp;
        i = ((uint8_t)(c)-48);
      }
    }
    switch (i) {
      case 0:
        TelnetStream.print("Enter 1 for INS info\r\n");
        TelnetStream.print("Enter 2 for Attitude\r\n");
        TelnetStream.print("Enter 3 for Time and IMU\r\n");
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
        TelnetStream.print(t4,7);
        TelnetStream.print(" deg\r\n");
        xQueuePeek(Longitude, &t4, portMAX_DELAY);
        TelnetStream.print("Longitude: ");
        TelnetStream.print(t4,7);
        TelnetStream.print(" deg\r\n");
        xQueuePeek(Altitude, &t4, portMAX_DELAY);
        TelnetStream.print("Altitude: ");
        TelnetStream.print(t4,7);
        TelnetStream.print(" m\r\n");
        xQueuePeek(VelBodyX, &t3, portMAX_DELAY);
        TelnetStream.print("Vx: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" m/s\r\n");
        xQueuePeek(VelBodyY, &t3, portMAX_DELAY);
        TelnetStream.print("Vy: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" m/s\r\n");
        xQueuePeek(VelBodyZ, &t3, portMAX_DELAY);
        TelnetStream.print("Vz: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" m/s\r\n");
        break;
      case 2:
        xQueuePeek(Yaw, &t3, portMAX_DELAY);
        TelnetStream.print("Yaw: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" deg\r\n");
        xQueuePeek(Pitch, &t3, portMAX_DELAY);
        TelnetStream.print("Pitch: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" deg\r\n");
        xQueuePeek(Roll, &t3, portMAX_DELAY);
        TelnetStream.print("Roll: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" deg\r\n");
        TelnetStream.print("Quat: ");
        xQueuePeek(QuatX, &t3, portMAX_DELAY);
        TelnetStream.print(t3,4);
        TelnetStream.print(" ");
        xQueuePeek(QuatY, &t3, portMAX_DELAY);
        TelnetStream.print(t3,4);
        TelnetStream.print(" ");
        xQueuePeek(QuatZ, &t3, portMAX_DELAY);
        TelnetStream.print(t3,4);
        TelnetStream.print(" ");
        xQueuePeek(QuatS, &t3, portMAX_DELAY);
        TelnetStream.print(t3,4);
        TelnetStream.print("\r\n");
        xQueuePeek(LinBodyAccX, &t3, portMAX_DELAY);
        TelnetStream.print("Ax: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" m/s^2\r\n");
        xQueuePeek(LinBodyAccY, &t3, portMAX_DELAY);
        TelnetStream.print("Ay: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" m/s^2\r\n");
        xQueuePeek(LinBodyAccZ, &t3, portMAX_DELAY);
        TelnetStream.print("Az: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" m/s^2\r\n");
        break;
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
        xQueuePeek(GyroBodyX, &t3, portMAX_DELAY);
        TelnetStream.print("Gx: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" rad/s\r\n");
        xQueuePeek(GyroBodyY, &t3, portMAX_DELAY);
        TelnetStream.print("Gy: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" rad/s\r\n");
        xQueuePeek(GyroBodyZ, &t3, portMAX_DELAY);
        TelnetStream.print("Gz: ");
        TelnetStream.print(t3,4);
        TelnetStream.print(" rad/s\r\n");
        break;
      default:ss
        i = 0;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
//hello
void loop() {
}
