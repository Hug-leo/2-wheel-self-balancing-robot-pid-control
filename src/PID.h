#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        integral = 0;
        previousError = 0;
    }

    float compute(float setpoint, float measured, float dt) {
        float error = setpoint - measured;
        integral += error * dt;
        float derivative = (error - previousError) / dt;
        previousError = error;

        return kp * error + ki * integral + kd * derivative;
    }

    void reset() {
        integral = 0;
        previousError = 0;
    }

private:
    float kp, ki, kd;
    float integral;
    float previousError;
};

#endif
