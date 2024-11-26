#include <ECE3.h>

const double sensorWeight[8] = { -15.0, -14.0, -12.0, 8.0, 8.0, 12.0, 14.0, 15.0 };
uint16_t min[8] = { 667,	620,	714, 620, 550,	573,	550,	738 };
uint16_t max[8] = { 1833, 1377,	1786,	1229,	1370,	1158,	896, 1762 };

uint16_t sensorReadings[8] = {0};

// Motor constants
const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;
const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

float Kp = 0.03; // Proportional gain, adjust as necessary
float Kd = 0.01;  // Derivative gain, adjust as necessary
float Ki = 0.0;  // Integral gain, adjust as necessary

int P, I, D, previousError = 0, error;

// Additional variables for line detection and turning
int lineCount = 0;

void setup() {
    ECE3_Init();

    pinMode(left_nslp_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);
    pinMode(right_nslp_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);
    pinMode(right_pwm_pin, OUTPUT);

    digitalWrite(left_nslp_pin, HIGH);
    digitalWrite(left_dir_pin, LOW);
    digitalWrite(right_nslp_pin, HIGH);
    digitalWrite(right_dir_pin, LOW);

    // Set initial speed directly
    delay(2000);
    analogWrite(left_pwm_pin, 40);
    analogWrite(right_pwm_pin, 40);
    delay(100);
}

void loop() {
      ECE3_read_IR(sensorReadings);

    analogWrite(right_pwm_pin, 0);
    analogWrite(left_pwm_pin, 0);

  // Calculate PID values
  error = sensorFusion(sensorReadings);
  P = error;
  I += error;
  D = error - previousError;
  int PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  int leftSpd = constrain(60 - PIDvalue, 0, 60);
  int rightSpd = constrain(60 + PIDvalue, 0, 60);

  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);

    int Black = 0;
    for (int i = 0; i < 8; i++)
    {
        if (sensorReadings[i] > 950) 
        {
            ++Black;
        }
    }

    if (Black > 6)
    {
        lineCount++;
        analogWrite(right_pwm_pin, 50);
        analogWrite(left_pwm_pin, 50);
        delay(500);
    }

        switch (lineCount) {
        // case 1: // First encounter with the line
        //     analogWrite(right_pwm_pin, 50);
        //     analogWrite(left_pwm_pin, 50);
        //     delay(250);
        //     analogWrite(right_pwm_pin, 15);
        //     digitalWrite(right_dir_pin, HIGH);
        //     analogWrite(left_pwm_pin, 50);
        //     delay(5000);
        //     digitalWrite(right_dir_pin, LOW);
        //     break;
        case 3: // Second encounter with the line
            analogWrite(right_pwm_pin, 50);
            digitalWrite(right_dir_pin, LOW);
            analogWrite(left_pwm_pin, 50);
            digitalWrite(left_dir_pin, HIGH);
            delay(1750);
            analogWrite(right_pwm_pin, 50);
            digitalWrite(right_dir_pin, LOW);
            analogWrite(left_pwm_pin, 50);
            digitalWrite(left_dir_pin, LOW);
            delay(1000);
            break;
        // case 4:
        //     analogWrite(right_pwm_pin, 50);
        //     analogWrite(left_pwm_pin, 50);
        //     delay(250);
        //     analogWrite(left_pwm_pin, 15);
        //     digitalWrite(left_dir_pin, HIGH);
        //     analogWrite(right_pwm_pin, 50);
        //     delay(5000);
        //     digitalWrite(left_dir_pin, LOW);
        //     break;
        case 6:
            analogWrite(right_pwm_pin, 0);
            analogWrite(left_pwm_pin, 0);
            delay(10000);
            break;
        default:
            break;
        }
}

int sensorFusion(uint16_t sensorValues[]) {
    double fusion_value  = 0;
    const int numSensors = 8; // 

    for (int i = 0; i < numSensors; i++) {
        //double denom = max[i] - min[i];
        //fusion_value += val * 1000 * weighting[i]/ denom ;
        fusion_value += (sensorValues[i] - min[i]) * 1000/(max[i] - min[i]) *sensorWeight[i];
    }
  return fusion_value;
}