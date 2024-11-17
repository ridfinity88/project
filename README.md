#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

float Kp = 1.0;    // ค่ากำลังของส่วน Proportional
float Ki = 0.01;   // ค่ากำลังของส่วน Integral
float Kd = 0.1;    // ค่ากำลังของส่วน Derivative

float Target = 180;      // ค่าเป้าหมายของ RiD_Finity
float RiD_Finity = 0;    // ค่า PWM ที่จะส่งออก
float previousError = 0; // ค่า Error ก่อนหน้า
float integral = 0;      // ค่า Sum ของ Error ทั้งหมด

const int pwmPin1 = 12;          // ขา PWM สำหรับมอเตอร์ตัวที่ 1
const int pwmPin2 = 13;          // ขา PWM สำหรับมอเตอร์ตัวที่ 2
const int pwmChannel1 = 0;       // ช่องสัญญาณ PWM สำหรับมอเตอร์ตัวที่ 1
const int pwmChannel2 = 1;       // ช่องสัญญาณ PWM สำหรับมอเตอร์ตัวที่ 2
const int freq = 50;             // ความถี่ PWM ที่ 50Hz สำหรับ ESC
const int resolution = 10;       // ความละเอียดของ PWM (0-1023 สำหรับ 10 บิต)

int cm = 0; // ตัวแปรสำหรับเก็บค่าระยะทาง

HardwareSerial tofSerial(2); // ใช้ Serial2 สำหรับเชื่อมต่อกับ TOF400F

// ตั้งเวลาการส่งคำสั่งอ่านค่าระยะทางทุก ๆ 100 ms
unsigned long previousMillis = 0;
const long interval = 100; // กำหนดระยะเวลาห่างในการส่งคำสั่งอ่านค่า

void setup() {
  Serial.begin(115200);          // Serial Monitor
  tofSerial.begin(115200, SERIAL_8N1, 16, 17); // Serial2 ที่ baud rate 115200, GPIO16=RX, GPIO17=TX

  Serial.println("เริ่มการวัดระยะทางจาก TOF400F...");

  // ตั้งค่า PWM สำหรับมอเตอร์
  ledcSetup(pwmChannel1, freq, resolution);
  ledcAttachPin(pwmPin1, pwmChannel1);

  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(pwmPin2, pwmChannel2);

  // เซ็ตค่าความเร็วเริ่มต้น
  ledcWrite(pwmChannel1, 100);
  ledcWrite(pwmChannel2, 100);
}

void loop() {
  unsigned long currentMillis = millis();

  // อ่านค่าระยะทางทุกๆ interval
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // ส่งคำสั่งอ่านค่าระยะทางไปที่เซ็นเซอร์
    byte readCommand[] = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF};
    tofSerial.write(readCommand, sizeof(readCommand));
  }

  // อ่านค่าระยะทางจากเซ็นเซอร์
  if (tofSerial.available() >= 7) {
    byte response[7];
    for (int i = 0; i < 7; i++) {
      response[i] = tofSerial.read();
    }

    if (response[0] == 0x01 && response[1] == 0x03) {
      int distance = (response[3] << 8) | response[4];
      cm = distance / 10; // แปลงระยะทางจาก mm เป็น cm
      Serial.print("ระยะทางที่วัดได้: ");
      Serial.println(cm);

      // คำนวณ PID
      float error = Target - cm;
      integral += error;
      float derivative = error - previousError;
      RiD_Finity = Kp * error + Ki * integral + Kd * derivative;
      RiD_Finity = constrain(RiD_Finity, 150, 220); // จำกัดค่าให้อยู่ในช่วง 150-220

      previousError = error;

      // แปลงค่าจาก RiD_Finity ไปยัง PWM
      int speed = map(RiD_Finity, 150, 220, 0, 1023);

      // ส่งค่า PWM ไปยังมอเตอร์
      ledcWrite(pwmChannel1, speed);
      ledcWrite(pwmChannel2, speed);

      Serial.print("ความเร็วที่ส่งไปมอเตอร์: ");
      Serial.println(speed);
    }
  }
}
