#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// GPIO untuk UART ke STM32
#define STM32_RX 16  // GPIO16 -> STM32 TX
#define STM32_TX 17  // GPIO17 -> STM32 RX

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);
  SerialBT.begin("ESP32_LineFollower");
  
  Serial.println("ESP32 Bridge Ready");
  Serial.println("STM32 ↔ ESP32 ↔ Laptop");
}

void loop() {
  // Bridge data dari STM32 ke Bluetooth
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    if (SerialBT.hasClient()) {
      SerialBT.println(data);
    }
    Serial.print("STM32→BT: ");
    Serial.println(data);
  }
  
  // Bridge data dari Bluetooth ke STM32
  if (SerialBT.available()) {
    String data = SerialBT.readStringUntil('\n');
    Serial2.println(data);
    Serial.print("BT→STM32: ");
    Serial.println(data);
  }
  
  delay(10);
}
