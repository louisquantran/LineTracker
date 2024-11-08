const int sensorWeight[8] = { -15, -14, -12, 8, 8, 12, 14, 15 };

// Motor constant
const int left_nslp_pin  = 31;  // nslp = not sleep --> set high --> digitalWrite
const int left_dir_pin   = 29;  // control direction of motor --> digitalWrite
const int left_pwm_pin   = 40;  // use to control motor speed --> analogWrite
const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// Sensor Pins
const int sensor1 = 65; // left side of car
const int sensor2 = 48;
const int sensor3 = 64;
const int sensor4 = 47;
const int sensor5 = 52;
const int sensor6 = 68;
const int sensor7 = 53;
const int sensor8 = 69; // right side of car

// LED Constants
const int IR_LED_odd = 45;
const int IR_LED_even = 61;

// Speed Constants
const int speed = 75;
const int turnSpeed = 75;
const int dirScale = 5;

// Delay Constants
const int delayA = 50;
const int delayB = 75;
const int timeToTurn = 700;

// Past Directions
const int errorThreshold = 50;

// PID Constants
const int kP = 10;
const int kD = 50;

// Variables
float pastDir;
float pastMotorSpeed;
int lineCnt;
int pastReadingCnt;
int base[8] = {0};
void setup() {
    for(int i = 0; i < 5; i++)
    {
        base[0] = digitalRead(sensor1);
        base[1] = digitalRead(sensor2);
        base[2] = digitalRead(sensor3);
        base[3] = digitalRead(sensor4);
        base[4] = digitalRead(sensor5);
        base[5] = digitalRead(sensor6);
        base[6] = digitalRead(sensor7);
        base[7] = digitalRead(sensor8);
        delay(5);
    }

    for(int i = 0; i < 8; i++)
    {
        base[i] /= 5;
    }

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

    pinMode(IR_LED_odd, OUTPUT);
    pinMode(IR_LED_even, OUTPUT);

    digitalWrite(IR_LED_odd, HIGH);
    digitalWrite(IR_LED_even, HIGH);
    
    pastDir = 0;
    lineCnt = 0;
    pastReadingCnt = 0;
}

void loop() {
    pinMode(sensor1, OUTPUT);
    pinMode(sensor2, OUTPUT);
    pinMode(sensor3, OUTPUT);
    pinMode(sensor4, OUTPUT);
    pinMode(sensor5, OUTPUT);
    pinMode(sensor6, OUTPUT);
    pinMode(sensor7, OUTPUT);
    pinMode(sensor8, OUTPUT);

    delay(delayA);

    digitalWrite(sensor1, HIGH);
    digitalWrite(sensor2, HIGH);
    digitalWrite(sensor3, HIGH);
    digitalWrite(sensor4, HIGH);
    digitalWrite(sensor5, HIGH);
    digitalWrite(sensor6, HIGH);
    digitalWrite(sensor7, HIGH);
    digitalWrite(sensor8, HIGH);

    delay(delayB);
    
    pinMode(sensor1, INPUT);
    pinMode(sensor2, INPUT);
    pinMode(sensor3, INPUT);
    pinMode(sensor4, INPUT);
    pinMode(sensor5, INPUT);
    pinMode(sensor6, INPUT);
    pinMode(sensor7, INPUT);
    pinMode(sensor8, INPUT);

    // Calculate the position based on the weights
    int sensorReading1 = digitalRead(sensor1);
    int sensorReading2 = digitalRead(sensor2);
    int sensorReading3 = digitalRead(sensor3);
    int sensorReading4 = digitalRead(sensor4);
    int sensorReading5 = digitalRead(sensor5);
    int sensorReading6 = digitalRead(sensor6);
    int sensorReading7 = digitalRead(sensor7);
    int sensorReading8 = digitalRead(sensor8);

    sensorReading1 -= base[0];
    sensorReading2 -= base[1];
    sensorReading3 -= base[2];
    sensorReading4 -= base[3];
    sensorReading5 -= base[4];
    sensorReading6 -= base[5];
    sensorReading7 -= base[6];
    sensorReading8 -= base[7];

    float error = sensorReading1 * sensorWeight[0] + sensorReading2 * sensorWeight[1]
                    + sensorReading3 * sensorWeight[2] + sensorReading4 * sensorWeight[3]
                    + sensorReading5 * sensorWeight[4] + sensorReading6 * sensorWeight[5]
                    + sensorReading7 * sensorWeight[6] + sensorReading8 * sensorWeight[7];
        
    int readingCnt = sensorReading1 + sensorReading2 + sensorReading3 + sensorReading4 
                    + sensorReading5 + sensorReading6 + sensorReading7 + sensorReading8;

    float motorSpeed = kP * error + kD * (error - pastDir);

    // if the car is seeing the line, so all the sensors are lit up
    if(readingCnt >= 6 && pastReadingCnt < 6)
    {
        lineCnt++;
        switch(lineCnt)
        {
            // Seeing the line at the end of the track and turning around
            case 1:
                digitalWrite(left_dir_pin, HIGH);
                analogWrite(right_pwm_pin, turnSpeed);
                analogWrite(left_pwm_pin, turnSpeed);
                delay(timeToTurn);
                digitalWrite(left_dir_pin, LOW);
                break;
            // Seeing the line at the end of the track and stopping
            case 2:
                analogWrite(right_pwm_pin, 0);
                analogWrite(left_pwm_pin, 0);
                pastMotorSpeed = 0;
                delay(50);
                break;
            default:
                break;
        }
    }

    // if the car is seeing more than 3 sensor values, it is probably bogus and should not be counted
    else if(readingCnt > 4)
    {
        motorSpeed = pastMotorSpeed;
    }

    // the car sees the right line or none at all
    else if(lineCnt != 2)
    {
        // if the car is turning to the left a significant amount
        if(motorSpeed < -30)
        {
            analogWrite(right_pwm_pin, speed + motorSpeed);
            analogWrite(left_pwm_pin, 0.6 * (speed - motorSpeed));
        }
        // if the car is turning to the right a signifcant amount
        else if(motorSpeed > 30)
        {
            analogWrite(right_pwm_pin, 0.6 * (speed + motorSpeed));
            analogWrite(left_pwm_pin, speed - motorSpeed);
        }
        // if the car is not making a sharp turn
        else
        {
            analogWrite(right_pwm_pin, speed + motorSpeed);
            analogWrite(left_pwm_pin, speed - motorSpeed);
        }
    }

    pastDir = error;
    pastMotorSpeed = motorSpeed;
    pastReadingCnt = readingCnt;
}
