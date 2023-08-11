#include <SparkFun_LIS331.h>
#include <SPI.h>

#define DRIVE_LEFT_PIN 5      // Pin number for sending signal to the Left Drive ESC. 
#define DRIVE_RIGHT_PIN 6     // Pin number for sending signal to the Right Drive ESC.

#define LED_PIN 3             // Pin number for powering LEDs

#define CS_PIN_ACCEL_1 9        // Pin number for Accelerometer 1 Chip Select
#define CS_PIN_ACCEL_2 10       // Pin number for Accelerometer 2 Chip Select
#define MOSI_PIN 11             // Pin number for SPI Master Out Slave In to accelerometer.
#define MISO_PIN 12             // Pin number for SPI Master In Slave Out to accelerometer.
#define SCK_PIN 13              // Pin number for SPI Serial Clock to accelerometer.

#define RECEIVER_FB_PIN 14      // Pin number for accessing the receiver's forwards and backwards data.
#define RECEIVER_LR_PIN 15      // Pin number for accessing the receiver's forwards and backwards data.
#define RECEIVER_SPIN_PIN 16    // Pin number for accessing the receiver's forwards and backwards data.

LIS331 accelerometer_1;         // H3LIS331DL accelerometer 1.
int16_t accel_1_X;              // H3LIS331DL 1 x-axis reading
int16_t accel_1_Y;              // H3LIS331DL 1 y-axis reading
int16_t accel_1_Z;              // H3LIS331DL 1 z-axis reading
LIS331 accelerometer_2;         // H3LIS331DL accelerometer 2.
int16_t accel_2_X;              // H3LIS331DL 2 x-axis reading
int16_t accel_2_Y;              // H3LIS331DL 2 y-axis reading
int16_t accel_2_Z;              // H3LIS331DL 2 z-axis reading
int16_t accel_axes_sensor_avg;  // Average acceleration of both accelerometer's x and y axes 
float accel_g;                 // Acceleration, in Gs, of the averaged acelerometer readings
const float k_gravityAccel = 9.80665;
const double k_accelRadius = 0.017387755674;      // Distance between the center of the robot and the accelerometers. In meters.

double robot_angVelocity;     // Angular velocity of the robot.
double robot_heading;         // Direction the "front" of the robot is facing (radians)
const int k_maxRpm = 2000;    // Max RPM of the robot.
const float k_headingLEDHigh = 1.95;  // Higher angle, in radians, the robot has to be facing in order to turn on LEDs
const float k_headingLEDLow = 0.05;   // Lower angle, in radians, the robot has to be facing in order to turn on LEDs

unsigned long calcTime_cyccnt;       // Time read right after calculations. Gathers how many clock cycles between 
unsigned long prevTime_cyccnt;       // Stored time from the previous clock cycle. Used to find the difference between cycles 
int betweenTimes_cyccnt;             // Variable between current clock cycle and calcTime_cyccnt to use in calculations
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
  calcTime_cyccnt = ARM_DWT_CYCCNT;
  betweenTimes_cyccnt = (calcTime_cyccnt - prevTime_cyccnt);
  prevTime_cyccnt = calcTime_cyccnt;
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
  digitalWrite(CS_PIN_ACCEL_1, LOW);
  accelerometer_1.readAxes(accel_1_X, accel_1_Y, accel_1_Z);
  digitalWrite(CS_PIN_ACCEL_1, HIGH);
  // Next 3 lines are to calibrate sensors to an average of 0 sensor units in both positive and negative directions in regards to gravity;
  accel_1_X += (23 - 17) / 2;
  accel_1_Y += (37 - 5) / 2;
  accel_1_Z += (5 - 37) / 2;
  digitalWrite(CS_PIN_ACCEL_2, LOW);
  accelerometer_2.readAxes(accel_2_X, accel_2_Y, accel_2_Z);
  digitalWrite(CS_PIN_ACCEL_2, HIGH);
  // Next 3 lines are to calibrate sensors to an average of 0 sensor units in both positive and negative directions in regards to gravity;
  accel_2_X += (18 - 24) / 2;
  accel_2_Y += (20 - 22) / 2;
  accel_2_Z += (-14 - 58) / 2;
  calculateTimeCycle();
  accel_axes_sensor_avg = (uint16_t)(sqrt(2) * ((abs(accel_1_X) + abs(accel_1_Y) + abs(accel_2_X) + abs(accel_2_Y)) / 4.0)); // sqrt((((X1 + Y1 + X2 + Y2) / 4) ^ 2) * 2) = sqrt(2X^2) = sqrt(2) * X
  accel_g = accelerometer_1.convertToG(100, accel_axes_sensor_avg);
}

/*
 * driveMotors
 * 
 * Sends the PWM signals to drive each motor
 */
void driveMotors() {
  int leftAnalogOut = map(leftDrivePwm, k_pwmMin, k_pwmMax, k_analogMin, k_analogMax);
  int rightAnalogOut = map(rightDrivePwm, k_pwmMin, k_pwmMax, k_analogMin, k_analogMax);
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

/*
 * meltyState
 * 
 * The logic and movement designed to control the robot as a meltybrain
 */
void meltyState() {
  robot_angVelocity = sqrt((accel_g * k_gravityAccel) / k_accelRadius);
  double x = (((double) rx_leftRight) - k_pwmCenter) / 500.0;
  double y = (((double) rx_forwardsBackwards) - k_pwmCenter) / 500.0;
  double mag_pow = min(max(-1.0, sqrt(x*x+y*y)), 1.0);
  double ang_rad = atan2(y, x) / PI; // Maybe negate?
  double t_seconds = betweenTimes_cyccnt * k_clockCycleToSeconds;
  double spinPower = min(max(0.0, (rx_spinSpeed - 1000.0) / 1000.0), 1.0);
  robot_heading = robot_heading + ((robot_angVelocity * t_seconds) / (2.00 * PI));
//  while(robot_heading > 2.00) {
//    robot_heading -= 2.00;
//  }
  if (robot_heading >= k_headingLEDHigh) {
    digitalWrite(LED_PIN, HIGH);
    if (robot_heading >= 2.00) {
      robot_heading -= 2.00;
    }
  } else if (robot_heading >= k_headingLEDLow) {
    digitalWrite(LED_PIN, LOW);
  }
  double angleDelta_cos = cos((ang_rad - robot_heading) * PI);
  double adj = mag_pow * abs(angleDelta_cos) * k_meltyAdjustFactor;
  if (angleDelta_cos >= 0.00) {
    leftDrivePwm = pwmCheck((spinPower - adj) * 500.0 + 1500.0);
    rightDrivePwm = pwmCheck((spinPower + adj) * 500.0 + 1500.0);
  } else {
    leftDrivePwm = pwmCheck((spinPower + adj) * 500.0 + 1500.0);
    rightDrivePwm = pwmCheck((spinPower - adj) * 500.0 + 1500.0);
  }
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
  pinMode(CS_PIN_ACCEL_1, OUTPUT);
  digitalWrite(CS_PIN_ACCEL_1, HIGH);
  pinMode(CS_PIN_ACCEL_2, OUTPUT);
  digitalWrite(CS_PIN_ACCEL_2, HIGH);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);

  // Starts SPI
  SPI.begin();
  accelerometer_1.setPowerMode(LIS331::NORMAL);
  accelerometer_1.setSPICSPin(CS_PIN_ACCEL_1);
  accelerometer_1.begin(LIS331::USE_SPI);
  
  accelerometer_2.setPowerMode(LIS331::NORMAL);
  accelerometer_2.setSPICSPin(CS_PIN_ACCEL_2);
  accelerometer_2.begin(LIS331::USE_SPI);

  
  // Starts clock cycle timing
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  calcTime_cyccnt = ARM_DWT_CYCCNT;
}

void loop() {
  readReceiver();
  if (rx_spinSpeed < k_meltyStartValue) {
    arcadeState();
    robot_heading = 0;
    digitalWrite(LED_PIN, HIGH);
  } else {
    readAndConvertAccel();
    rx_spinSpeed = map(rx_spinSpeed, k_meltyStartValue, k_pwmMax, k_pwmMin, k_pwmMax);
    meltyState();
  }
  driveMotors();
}           
