#include <Adafruit_NeoPixel.h>
#include "SparkFun_LIS331.h"
#include <SPI.h>

#define DRIVE_LEFT_PIN 5      // Pin number for sending singal to the Left Drive ESC. 
#define DRIVE_RIGHT_PIN 6     // Pin number for sending singal to the Right Drive ESC.

#define LED_PIN 7             // Pin number for sending data to the LED.
#define LED_AMOUNT 2          // Amount of LEDs used
#define LED_RED 0             // Hue of the color red for the led
#define LED_YELLOW 10500      // Hue of the color yellow for the led
#define LED_BLUE 38000        // Hue of the color blue for the led

#define CS_PIN 10               // Pin number for SPI Chip Select to accelerometer.
#define MOSI_PIN 11             // Pin number for SPI Master Out Slave In to accelerometer.
#define MISO_PIN 12             // Pin number for SPI Master In Slave Out to accelerometer.
#define SCK_PIN 13              // Pin number for SPI Serial Clock to accelerometer.

#define RECEIVER_FB_PIN 14      // Pin number for accessing the receiver's forwards and backwards data.
#define RECEIVER_LR_PIN 15      // Pin number for accessing the receiver's forwards and backwards data.
#define RECEIVER_SPIN_PIN 16    // Pin number for accessing the receiver's forwards and backwards data.

Adafruit_NeoPixel leds(LED_AMOUNT, LED_PIN, NEO_GRB + NEO_KHZ800);    // Neopixel controller

LIS331 accelerometer;         // H3LIS331DL accelerometer. 
int16_t accelX;               // H3LIS331DL x-axis reading
int16_t accelY;               // H3LIS331DL y-axis reading
int16_t accelZ;               // H3LIS331DL z-axis reading
float accelXToG;              // X-axis reading to G values
float accelYToG;              // Y-axis reading to G values
float accelZToG;              // Z-axis reading to G values
const float k_gravityAccel = 9.81;
const double k_accelRadius = 0.0635;      // Distance between the center of the robot and the accelerometer. In m.

double robot_angVelocity;     // Angular velocity of the robot.
double robot_rpm;             // RPM of the robot.
const int k_maxRpm = 2000;    // Max RPM of the robot.

unsigned long calcTime;       // Time read right after calculations. Gathers how many clock cycles between 
unsigned long prevTime;       // Stored time from the previous clock cycle. Used to find the difference between cycles 
int betweenTimes;             // Variable between current clock cycle and calcTime to use in calculations
const double k_clockCycleToSeconds = 1.0 / 600000000;     // constant variable to store the conversion factor to seconds

uint16_t leftDrivePwm;        // Left-side drive power. Tells the Left-side drive how fast to spin.
uint16_t rightDrivePwm;       // Right-side drive power. Tells the Right-side drive how fast to spin.
double meltyMoveAngle;        // Angle at which the meltybrain needs to move.
const double k_meltyAdjustFactor = 0.25;

uint16_t rx_forwardsBackwards;     // Forwards/Backwards control. Receiver signal for gathering forwards and backwards movement. -100 if backwards, 100 is forwards.
uint16_t rx_leftRight;             // Left/Right control. Receiver signal for gathering left and right movement. -100 is left, 100 is right.
uint16_t rx_spinSpeed;             // Spin control. Receiver signal for gathering what speed the robot should spin at. -100 up to 0 is arcade/calibrate drive, 0 to 100 is spin drive.
const int k_pwmMax = 2000;    // Maximum read pwm signal. Used to round down if pwm is higher.
const int k_pwmMin = 1000;    // Minimum read pwm signal. Used to round up if pwm is lower.
const int k_pwmCenter = 1500; // Neutral value of the pwm signal.
const int k_pwmDeadzone = 20; // Deadzone for pwm. Used to not take input if pwm is within +/- of center pwm.
const int k_meltyStartValue = 1050;

#define ANALOG_MAX 8191

// PWM FREQ must not be >= 500
#define PWM_FREQ 490
const int k_analogMin = (int)round(ANALOG_MAX * 0.000001 * PWM_FREQ * k_pwmMin);
const int k_analogMax = (int)round(ANALOG_MAX * 0.000001 * PWM_FREQ * k_pwmMax);

/*
 * calculateTimeCycle
 * 
 * Timing function. finds the difference between a previous and the current cycles. Converts cycle difference from clock speed to seconds.
 * Sets the new cycle to be used next loop.
 */
void calculateTimeCycle() {
  calcTime = ARM_DWT_CYCCNT;
  betweenTimes = (calcTime - prevTime) * k_clockCycleToSeconds;
  prevTime = calcTime;
}

/*
 * pwmCheck
 * 
 * Checks the pwm signal given and bounds the signal if needed
 * 
 * @param rx_signal - The signal to check in bounds 
 * @return - The signal if it needed to be bound
 */
int pwmCheck(int rx_signal) {
  if (rx_signal > k_pwmMax) {
    return k_pwmMax;
  } else if (rx_signal < k_pwmMin) {
    return k_pwmMin;
  } else if ((rx_signal > k_pwmCenter - k_pwmDeadzone) && (rx_signal < k_pwmCenter + k_pwmDeadzone)) {
    return k_pwmCenter;
  } else {
    return rx_signal;
  }
}

/* 
 *  readReceiver
 *  
 *  Reads each analog pin and sets them to their respective variable
 */
void readReceiver() {
  rx_forwardsBackwards = pwmCheck(pulseIn(RECEIVER_FB_PIN, HIGH));
  rx_leftRight = pwmCheck(pulseIn(RECEIVER_LR_PIN, HIGH));
  rx_spinSpeed = pwmCheck(pulseIn(RECEIVER_SPIN_PIN, HIGH));
}

/*
 * readAndConvertAccel
 * 
 * Passes accelerometer axis reading variables to be updated. Converts each value into Gs
 */
void readAndConvertAccel() {
  accelerometer.readAxes(accelX, accelY, accelZ);
  accelXToG = abs(accelerometer.convertToG(100, accelX));
  accelYToG = abs(accelerometer.convertToG(100, accelY));
  accelZToG = abs(accelerometer.convertToG(100, accelZ));
}

/*
 * driveMotors
 * 
 * Sends the PWM signals to drive each motor
 */
void driveMotors() {
  int leftAnalogOut = map(leftDrivePwm, k_pwmMin, k_pwmMax, k_analogMin, k_analogMax);
  int rightAnalogOut = map(rightDrivePwm, k_pwmMin, k_pwmMax, k_analogMin, k_analogMax);
  Serial.print(leftDrivePwm);
  Serial.print(" (");
  Serial.print(leftAnalogOut);
  Serial.print("), ");
  Serial.print(rightDrivePwm);
  Serial.print(" (");
  Serial.print(rightAnalogOut);
  Serial.println(")");
  analogWrite(DRIVE_LEFT_PIN, leftAnalogOut);
  analogWrite(DRIVE_RIGHT_PIN, rightAnalogOut);
}

/* 
 * arcadeState
 *  
 * Robot control if the robot is in arcade mode.
 */
void arcadeState() {
  leftDrivePwm = pwmCheck(rx_forwardsBackwards + rx_leftRight - k_pwmCenter);
  rightDrivePwm = pwmCheck(rx_leftRight + k_pwmCenter - rx_forwardsBackwards);
}

void meltyState() {
  robot_angVelocity = sqrt((accelXToG * k_gravityAccel) / k_accelRadius);
  robot_rpm = robot_angVelocity * (30 / PI);
//  Serial.println("accelXToG: ");
//  Serial.println(accelXToG);
//  Serial.print("robot_rpm: ");
//  Serial.println(robot_rpm);
  double x = (((double) rx_leftRight) - k_pwmCenter) / 500.0;
  double y = (((double) rx_forwardsBackwards) - k_pwmCenter) / 500.0;
  double mag_pow = min(max(-1.0, sqrt(x*x+y*y)), 1.0);
  double ang_rad = atan2(y, x); // Maybe negate?
  double timeScalar = PI * robot_rpm / 30.0;
  double t_seconds = calcTime * k_clockCycleToSeconds;
  double adj = (mag_pow * sin(timeScalar * t_seconds + ang_rad)) * k_meltyAdjustFactor;
  double spinPower = min(max(0.0, (rx_spinSpeed - 1000.0) / 1000.0), 1.0);
  leftDrivePwm = pwmCheck((spinPower + adj) * 500.0 + 1500.0);
  rightDrivePwm = pwmCheck((spinPower - adj) * 500.0 + 1500.0);
}

void setup() {
  // Configures PWM output pins to control drive and LED
  pinMode(DRIVE_LEFT_PIN, OUTPUT);
  pinMode(DRIVE_RIGHT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Configures Analog input pins to read receiver data
  pinMode(RECEIVER_FB_PIN, INPUT);
  pinMode(RECEIVER_LR_PIN, INPUT);
  pinMode(RECEIVER_SPIN_PIN, INPUT);

  analogWriteResolution(13);
  analogWriteFrequency(DRIVE_LEFT_PIN, PWM_FREQ);
  analogWriteFrequency(DRIVE_RIGHT_PIN, PWM_FREQ);

  // Configues SPI pins for communication with the H3LIS331DL
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);

  // Starts SPI
  SPI.begin();
  accelerometer.setSPICSPin(CS_PIN);
  accelerometer.begin(LIS331::USE_SPI);
  
  // Starts clock cycle timing
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  calcTime = ARM_DWT_CYCCNT;

  Serial.begin(152000);
}

void loop() {
  readReceiver();
  calculateTimeCycle();
  if (rx_spinSpeed < k_meltyStartValue) {
    Serial.println("ARCA: ");
    arcadeState();
  } else {
    readAndConvertAccel();
    rx_spinSpeed = map(rx_spinSpeed, k_meltyStartValue, 2000, 1000, 2000);
    Serial.println("MELT: ");
    meltyState();
  }
  Serial.println();
  driveMotors();
}           
