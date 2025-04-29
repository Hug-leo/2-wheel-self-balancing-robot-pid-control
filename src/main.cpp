// #include <Arduino.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>
// Adafruit_MPU6050 mpu;

// // Motor control pins
// const int enA = 5;
// const int in1 = 6;
// const int in2 = 7;

// const int enB = 10;
// const int in3 = 8;
// const int in4 = 9;

// // Encoder Motor 1 (D12, D13)
// const int encoder1_A = 12;
// const int encoder1_B = 13;

// // Encoder Motor 2 (D2, D3)
// const int encoder2_A = 2;
// const int encoder2_B = 3;

// // Encoder counters
// volatile long encoderCount1 = 0;
// volatile long encoderCount2 = 0;

// // RPM values
// volatile float rpm_motor1 = 0;
// volatile float rpm_motor2 = 0;

// // Pulses per revolution
// const int pulsesPerRevolution = 330;

// // Lưu trạng thái cũ
// volatile bool lastState1_A = 0;
// volatile bool lastState2_A = 0;

// // ===== Prototype khai báo trước =====
// void runMotorA(int speed, bool forward);
// void runMotorB(int speed, bool forward);
// void readEncoder2();

// void setup() {
//   Serial.begin(9600);
//   Wire.begin();

// //  // Try to initialize!
// //  if (!mpu.begin()) {
// //   Serial.println("Failed to find MPU6050 chip");
// //   while (1) {
// //     delay(10);
// //   }
// // }
// // Serial.println("MPU6050 Found!");
// // // set accelerometer range to +-8G
// // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
// // // set gyro range to +- 500 deg/s
// // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
// // // set filter bandwidth to 21 Hz
// // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


//   // Motor output pins
//   pinMode(enA, OUTPUT);
//   pinMode(in1, OUTPUT);
//   pinMode(in2, OUTPUT);

//   pinMode(enB, OUTPUT);
//   pinMode(in3, OUTPUT);
//   pinMode(in4, OUTPUT);

//   // Encoder input pins
//   pinMode(encoder1_A, INPUT);
//   pinMode(encoder1_B, INPUT);
//   pinMode(encoder2_A, INPUT);
//   pinMode(encoder2_B, INPUT);

//   // Attach interrupts
//   attachInterrupt(digitalPinToInterrupt(encoder2_A), readEncoder2, CHANGE);

//   // Enable Pin Change Interrupt for Motor 1
//   PCICR |= (1 << PCIE0);        // Enable PCINT for D8–D13
//   PCMSK0 |= (1 << PCINT4);      // Enable interrupt for D12
//   PCMSK0 |= (1 << PCINT5);      // Enable interrupt for D13

//   lastState1_A = digitalRead(encoder1_A);
//   lastState2_A = digitalRead(encoder2_A);

//   // Setup Timer2 to generate interrupt every 4ms
//   TCCR2A = 0;
//   TCCR2B = 0;
//   TCNT2  = 0;
//   OCR2A = 249;                // (16MHz / 64 prescaler / 250) = 1kHz
//   TCCR2A |= (1 << WGM21);     // CTC mode
//   TCCR2B |= (1 << CS22);      // Prescaler 64
//   TIMSK2 |= (1 << OCIE2A);    // Enable Timer2 Compare Match Interrupt
// }

// void loop() {
//   runMotorA(90, false);
//   runMotorB(70, true);
//   static unsigned long lastPrint = 0;
//   if (millis() - lastPrint > 500) {
    
//     Serial.print("RPM Motor 1: "); Serial.print(rpm_motor1);
//     Serial.print("\t RPM Motor 2: "); Serial.println(rpm_motor2);

//     // // /* Get new sensor events with the readings */
//     // sensors_event_t a, g, temp;
//     // mpu.getEvent(&a, &g, &temp);
//     // /* Print out the values */
//     // Serial.print("Acceleration X: ");
//     // Serial.print(a.acceleration.x);
//     // Serial.print(", Y: ");
//     // Serial.print(a.acceleration.y);
//     // Serial.print(", Z: ");
//     // Serial.print(a.acceleration.z);
//     // Serial.println(" m/s^2");
//     // Serial.print("Rotation X: ");
//     // Serial.print(g.gyro.x);
//     // Serial.print(", Y: ");
//     // Serial.print(g.gyro.y);
//     // Serial.print(", Z: ");
//     // Serial.print(g.gyro.z);
//     // Serial.println(" rad/s");
//     // Serial.print("Temperature: ");
//     // Serial.print(temp.temperature);
//     // Serial.println(" degC");
//     // Serial.println("");
//     lastPrint = millis();
//   }
// }

// // ISR encoder motor 2 (D2, D3)
// void readEncoder2() {
//   bool stateA = digitalRead(encoder2_A);
//   bool stateB = digitalRead(encoder2_B);

//   if (stateA == stateB)
//     encoderCount2++;
//   else
//     encoderCount2--;
// }

// // ISR encoder motor 1 (D12, D13) - Pin Change Interrupt
// ISR(PCINT0_vect) {
//   bool stateA = digitalRead(encoder1_A);
//   bool stateB = digitalRead(encoder1_B);

//   if (stateA != lastState1_A) {
//     if (stateA == stateB)
//       encoderCount1++;
//     else
//       encoderCount1--;
//     lastState1_A = stateA;
//   }
// }

// // Timer2 interrupt routine
// ISR(TIMER2_COMPA_vect) {
//   static int counter = 0;
//   counter++;
//   if (counter >= 25) { // 4ms * 25 = 100ms
//     rpm_motor1 = (float)encoderCount1 * 600.0 / pulsesPerRevolution;
//     rpm_motor2 = (float)encoderCount2 * 600.0 / pulsesPerRevolution;

//     encoderCount1 = 0;
//     encoderCount2 = 0;
//     counter = 0;
//   }
// }

// // Motor control
// void runMotorA(int speed, bool forward) {
//   digitalWrite(in1, forward ? HIGH : LOW);
//   digitalWrite(in2, forward ? LOW : HIGH);
//   analogWrite(enA, constrain(speed, 0, 255));
// }

// void runMotorB(int speed, bool forward) {
//   digitalWrite(in3, forward ? HIGH : LOW);
//   digitalWrite(in4, forward ? LOW : HIGH);
//   analogWrite(enB, constrain(speed, 0, 255));
// }

#include <Arduino.h>
#include <Wire.h>
#include "PID.h"
#include <MPU6050_tockn.h>

// ===== Khai báo cảm biến và điều khiển =====
MPU6050 mpu6050(Wire);
// Motor và Encoder
const int enA = 5, in1 = 6, in2 = 7;
const int enB = 10, in3 = 8, in4 = 9;
const int encoder1_A = 12, encoder1_B = 13;
const int encoder2_A = 2, encoder2_B = 3;
volatile long encoderCount1 = 0, encoderCount2 = 0;
volatile float rpm_motor1 = 0, rpm_motor2 = 0;
volatile bool lastState1_A = 0, lastState2_A = 0;
const int pulsesPerRevolution = 330;

// PID Controllers
PID pidAngle(40.0, 0.0, 0.8);  // PID cho góc Pitch
PID pidSpeed(0.6, 0.0, 0.0);   // PID cho tốc độ

float pitch = 0; 
float targetRPM = 0;
unsigned long lastTime;

// Hệ số cân chỉnh PWM cho 2 motor
const float motor1_scale = 1.395;  // Motor 1 cần boost 40%
const float motor2_scale = 1.00;  // Motor 2 giữ nguyên

// ===== Prototype khai báo trước =====
void readEncoder2();
void runMotorA(int speed, bool forward);
void runMotorB(int speed, bool forward);

// ====== SETUP ======
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // ===== Khởi động MPU6050 tockn =====
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);  // <<< Tự động calibrate gyro
  lastTime = millis();

  // Motor pins
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  // Encoder pins
  pinMode(encoder1_A, INPUT);
  pinMode(encoder1_B, INPUT);
  pinMode(encoder2_A, INPUT);
  pinMode(encoder2_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder2_A), readEncoder2, CHANGE);

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT4) | (1 << PCINT5);

  lastState1_A = digitalRead(encoder1_A);
  lastState2_A = digitalRead(encoder2_A);

  // Timer2 setup
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 249;
  TCCR2A |= (1 << WGM21); 
  TCCR2B |= (1 << CS22);
  TIMSK2 |= (1 << OCIE2A);
}

// ====== MAIN LOOP ======
void loop() {

  mpu6050.update();  // Cập nhật dữ liệu cảm biến

  pitch = mpu6050.getAngleX() ;  // Lấy góc Pitch theo X

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // Đổi sang giây
  lastTime = now;
 
  // pitch = kalmanPitch.getAngle(accPitch, gyroY, dt);
  // ===== PID1: Điều khiển góc Pitch =====
  float outputAngle = pidAngle.compute(0.0, pitch, dt); // mong muốn pitch = 0 độ
  targetRPM = constrain(outputAngle, -400, 400);

  // ===== PID2: Điều khiển tốc độ RPM =====
  float currentRPM = (rpm_motor1 + rpm_motor2) / 2.0; // RPM trung bình
  float outputSpeed = pidSpeed.compute(targetRPM, currentRPM, dt);

  // ===== Điều khiển động cơ =====
  int pwmOutput = constrain((int)outputSpeed, -255, 255);

  int pwmMotorA = constrain(abs(pwmOutput) * motor1_scale, 0, 255);
  int pwmMotorB = constrain(abs(pwmOutput) * motor2_scale, 0, 255);

  if (pwmOutput > 0) {
    runMotorA(pwmMotorA, true);
    runMotorB(pwmMotorB, false);
  } else {
    runMotorA(pwmMotorA, false);
    runMotorB(pwmMotorB, true);
  }
  delay(5); // vòng lặp ~200Hz
  //   runMotorA(70, false);
  //   runMotorB(70, true);
  // In dữ liệu
  // Serial.print("Kalman Pitch Angle: ");
  // Serial.print(pitch);
  // Serial.println(" deg");
  // Serial.print("RPM Motor 1: "); Serial.print(rpm_motor1);
  // Serial.print("\t RPM Motor 2: "); Serial.println(rpm_motor2);
  // Serial.println("-----------------------------------");
  // delay(1000);
}

// ===== ISR: Encoder đọc xung motor =====
void readEncoder2() {
  bool stateA = digitalRead(encoder2_A);
  bool stateB = digitalRead(encoder2_B);
  if (stateA == stateB) encoderCount2++;
  else encoderCount2--;
}

ISR(PCINT0_vect) {
  bool stateA = digitalRead(encoder1_A);
  bool stateB = digitalRead(encoder1_B);
  if (stateA != lastState1_A) {
    if (stateA == stateB) encoderCount1++;
    else encoderCount1--;
    lastState1_A = stateA;
  }
}

ISR(TIMER2_COMPA_vect) {
  static int counter = 0;
  counter++;
  if (counter >= 25) { // mỗi 100ms
    rpm_motor1 = (float)encoderCount1 * 600.0 / pulsesPerRevolution;
    rpm_motor2 = (float)encoderCount2 * 600.0 / pulsesPerRevolution;
    encoderCount1 = 0;
    encoderCount2 = 0;
    counter = 0;
  }
}

// ===== Hàm chạy motor A/B =====
void runMotorA(int speed, bool forward) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(enA, constrain(speed, 0, 255));
}

void runMotorB(int speed, bool forward) {
  digitalWrite(in3, forward ? HIGH : LOW);
  digitalWrite(in4, forward ? LOW : HIGH);
  analogWrite(enB, constrain(speed, 0, 255));
}

// #include <Wire.h>
// #include <Arduino.h>

// void setup() {
//   Wire.begin();
//   Serial.begin(9600);
//   Serial.println("Quet I2C...");
// }

// void loop() {
//   byte error, address;
//   int nDevices = 0;

//   for(address = 1; address < 127; address++ ) {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("Tim thay thiet bi I2C tai dia chi 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println();
//       nDevices++;
//     }
//   }

//   if (nDevices == 0)
//     Serial.println("Khong tim thay thiet bi I2C nao");
//   else
//     Serial.println("Hoan thanh quet.");

//   delay(2000); // quét lại mỗi 5 giây
// }
