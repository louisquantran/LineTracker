#include <ECE3.h>

const double sensorWeight[8] = { -15.0, -14.0, -12.0, 8.0, 8.0, 12.0, 14.0, 15.0 };
uint16_t mins[8] = { 808,	644,	667,	621,	644,	621,	668,	768 };
uint16_t maxs[8] = { 1692,	1856,	1569,	1566,	1567,	1732,	1832,	1732 };

uint16_t sensorReadings[8] = {0};
const int baseSpd = 40;

int turnAround = 0;

// Motor constants
const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;
const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

float Kp = 0.03; // Proportional gain, adjust as necessary
float Kd = 0.15;  // Derivative gain, adjust as necessary
float Ki = 0.0;  // Integral gain, adjust as necessary

int P, I, D, previousError = 0, error;

// Additional variables for line detection and turning
int lineCount = 0;
int black = 0;
int turn = 0;

void setup() {
    Serial.begin(9600);
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
  previousError = error;
  int leftSpd = baseSpd;
  int rightSpd = baseSpd;

 

    int Black = 0;
    for (int i = 0; i < 8; i++)
    {
        if (sensorReadings[i] > 950) 
        {
            ++Black;
        }
    }

    bool finishTurn = false;
    int final_leftSpd = leftSpd - (Kp * error);
    int final_rightSpd = rightSpd + (Kp * error); 
    if (Black > 6 && turnAround == 0)
    {
        ++lineCount;
        digitalWrite(left_dir_pin, LOW);
        analogWrite(left_pwm_pin, 50);
        digitalWrite(right_dir_pin, LOW);
        analogWrite(right_pwm_pin, 50);
        delay(175);
    }
    else if (Black > 6 && turnAround == 1)
    {
        ++lineCount;
    }
    if (lineCount == 2) turnAround = 1;
        if (lineCount == 3 && turn == 0)
        {
            ++turn;
            turnAround = 0;
            analogWrite(left_pwm_pin, 0);
            analogWrite(right_pwm_pin, 0);
            delay(500);
            donut();
        }
        else if (lineCount == 6 && turn == 1)
        {
            analogWrite(left_pwm_pin, 0);
            analogWrite(right_pwm_pin, 0);
            delay(10000);
            turn = 0;
        }
        else
        {
            analogWrite(left_pwm_pin, abs(final_leftSpd));
            analogWrite(right_pwm_pin, abs(final_rightSpd));
        }
        if (final_leftSpd < 0) {
          digitalWrite(left_dir_pin, HIGH);  
        }
        else {
          digitalWrite(left_dir_pin, LOW);  
        }

        if (final_rightSpd < 0) {
          digitalWrite(right_dir_pin, HIGH);  
        }
        else {
          digitalWrite(right_dir_pin, LOW);  
        }
}

int sensorFusion(uint16_t sensorValues[]) {
    double fusion_value  = 0;
    const int numSensors = 8; 

    for (int i = 0; i < numSensors; i++) {
        fusion_value += ((sensorValues[i] - mins[i]) * 1000/(maxs[i])) * (sensorWeight[i]/8.0);
    }
  return fusion_value;
}

// Testing purposes
// int newFusion(uint16_t arr[]) {
//   int i_arr[8] = {0};
//   for (int i = 0; i < 8; i++) {
//     i_arr[i] = arr[i];  
//   }
//   double error = 0;
//   int weights[] = {-4, -3, -2, -1, 1, 2, 3, 4};
//   for (int i = 0; i < 8; i++) {
//       error += i_arr[i] * weights[i];
//   }
//   return error;
// }
void donut() {
    analogWrite(right_pwm_pin, 50);
    digitalWrite(right_dir_pin, LOW);
    analogWrite(left_pwm_pin, 50);
    digitalWrite(left_dir_pin, HIGH);
    delay(1000);
    // analogWrite(right_pwm_pin, 50);
    // digitalWrite(right_dir_pin, LOW);
    // analogWrite(left_pwm_pin, 50);
    // digitalWrite(left_dir_pin, LOW);
    // delay(200);
}
