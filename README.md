# Two-Wheel Self-Balancing Robot - PID Control
<img width="1170" height="654" alt="image" src="https://github.com/user-attachments/assets/52f49c9d-f3d3-4da4-adc7-33d4598ef8c5" />
A self-balancing robot project using dual PID controllers implemented in C++ for Arduino. The robot uses an MPU6050 gyroscope/accelerometer sensor and motor encoders to maintain balance through cascaded PID control loops.

## 🎯 Project Overview

This project implements a two-wheel self-balancing robot (similar to a Segway) that uses sensor fusion and PID control algorithms to maintain an upright position. The robot successfully balances and has been enhanced with Fuzzy PID control capabilities.

## 🔧 Hardware Components

- **Microcontroller**: Arduino Uno
- **Sensors**:
  - MPU6050 6-axis IMU (gyroscope + accelerometer)
  - 2x Motor encoders (330 pulses per revolution)
- **Actuators**:
  - 2x DC motors with L298N motor driver
- **Communication**: I2C (Wire library)

## 📐 Pin Configuration

### Motor Control Pins
| Motor | Enable | IN1 | IN2 |
|-------|--------|-----|-----|
| Motor A | Pin 5 (PWM) | Pin 6 | Pin 7 |
| Motor B | Pin 10 (PWM) | Pin 8 | Pin 9 |

### Encoder Pins
| Encoder | Channel A | Channel B |
|---------|-----------|-----------|
| Motor 1 | Pin 12 | Pin 13 |
| Motor 2 | Pin 2 | Pin 3 |

### I2C Communication
- SDA: A4
- SCL: A5

## 🧮 Control Algorithm

### Cascaded PID Control System

The robot uses two PID controllers working in cascade:

1. **Angle PID Controller** (Primary Loop)
   - **Purpose**: Controls the tilt angle (pitch) of the robot
   - **Setpoint**: 0° (upright position)
   - **Input**: Pitch angle from MPU6050
   - **Output**: Target RPM for wheels
   - **Tuning Parameters**: Kp=40.0, Ki=0.0, Kd=0.8

2. **Speed PID Controller** (Secondary Loop)
   - **Purpose**: Controls motor speed to achieve target RPM
   - **Setpoint**: Target RPM from angle controller
   - **Input**: Average RPM from both motors
   - **Output**: PWM signal to motors
   - **Tuning Parameters**: Kp=0.6, Ki=0.0, Kd=0.0

### Kalman Filter

The project includes a Kalman filter implementation (`Kalman.h`) for sensor fusion, combining accelerometer and gyroscope data to obtain a more accurate and stable pitch angle estimation.

### Motor Calibration

Individual motor scaling factors are implemented to compensate for hardware differences:
- Motor 1: 1.395x scaling (40% boost)
- Motor 2: 1.0x scaling (baseline)

## 📁 Project Structure

```
2-wheel-self-balancing-robot-pid-control/
├── src/
│   ├── main.cpp          # Main control loop and setup
│   ├── PID.h             # PID controller class
│   └── Kalman.h          # Kalman filter implementation
├── include/
│   └── README
├── lib/
│   └── README
├── test/
│   └── README
├── platformio.ini        # PlatformIO configuration
└── README.md
```

## 🚀 Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) or Arduino IDE
- Arduino Uno board
- USB cable for programming

### Required Libraries

The following libraries are automatically installed via PlatformIO:
- `MPU6050_tockn` v1.5.2
- `Adafruit MPU6050` v2.2.6
- `MPU6050` by ElectronicCats v1.4.3
- `Wire` (built-in)

### Installation

1. Clone this repository:
```bash
git clone https://github.com/Hug-leo/2-wheel-self-balancing-robot-pid-control.git
cd 2-wheel-self-balancing-robot-pid-control
```

2. Open the project in PlatformIO or Arduino IDE

3. Connect your Arduino Uno via USB

4. Build and upload:
```bash
pio run --target upload
```

Or use the PlatformIO IDE upload button.

### Calibration

On startup, the MPU6050 will automatically calibrate the gyroscope offsets. **Keep the robot stationary during this process** (indicated by the LED blinking).

## ⚙️ Configuration & Tuning

### PID Parameters

Adjust the PID parameters in `main.cpp`:

```cpp
PID pidAngle(40.0, 0.0, 0.8);  // Angle PID: Kp, Ki, Kd
PID pidSpeed(0.6, 0.0, 0.0);   // Speed PID: Kp, Ki, Kd
```

### Motor Scaling

If your motors have different characteristics, adjust the scaling factors:

```cpp
const float motor1_scale = 1.395;  // Adjust for Motor 1
const float motor2_scale = 1.00;   // Adjust for Motor 2
```

### Encoder Configuration

Update if using different encoders:

```cpp
const int pulsesPerRevolution = 330;  // Change based on your encoder
```

## 📊 How It Works

1. **Sensor Reading**: MPU6050 reads acceleration and gyroscope data at ~200Hz
2. **Angle Calculation**: The pitch angle is calculated from sensor data
3. **Angle Control**: First PID controller computes target RPM based on angle error
4. **Speed Control**: Second PID controller adjusts motor PWM to achieve target RPM
5. **Motor Drive**: PWM signals are sent to L298N driver to control motor speed and direction
6. **Feedback Loop**: Encoders measure actual motor RPM, closing the control loop

## 🎓 Key Features

- ✅ Dual cascaded PID control loops
- ✅ Kalman filter for sensor fusion
- ✅ Interrupt-driven encoder reading
- ✅ Individual motor calibration
- ✅ Real-time RPM calculation
- ✅ Automatic gyroscope calibration
- ✅ Hardware timer-based sampling (Timer2)

## 🔄 Future Enhancements

- [ ] Implement Fuzzy PID controller
- [ ] Add Bluetooth/WiFi control
- [ ] Implement smartphone app for remote control
- [ ] Add battery voltage monitoring
- [ ] Implement emergency stop mechanism
- [ ] Add data logging and visualization

## 📝 Notes

- The main control loop runs at approximately 200Hz (5ms delay)
- RPM is calculated every 100ms using Timer2 interrupts
- PWM output is constrained to ±255 to prevent overflow
- The robot requires proper mechanical balance and weight distribution

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is open source and available under the MIT License.

## 👤 Author

**Hug-leo**

## 🙏 Acknowledgments

- MPU6050 library by tockn
- Arduino and PlatformIO communities
- Self-balancing robot community for inspiration and resources

---

**⚠️ Safety Warning**: Always test your robot in a safe environment. The robot can move unpredictably during initial testing and tuning.
