#include "SparkFun_LIS331.h"
#include <SPI.h>

#define DRIVE_LEFT_PIN 5        // Pin number for sending singal to the Left Drive ESC. 
#define DRIVE_RIGHT_PIN 6       // Pin number for sending singal to the Right Drive ESC.

#define LED_PIN 7               // Pin number for sending data to the LED.

#define CS_PIN 10               // Pin number for SPI Chip Select to accelerometer.
#define MOSI_PIN 11             // Pin number for SPI Master Out Slave In to accelerometer.
#define MISO_PIN 12             // Pin number for SPI Master In Slave Out to accelerometer.
#define SCL_PIN 13              // Pin number for SPI Serial Clock to accelerometer.

#define RECEIVER_FB_PIN 14      // Pin number for accessing the receiver's forwards and backwards data.
#define RECEIVER_LR_PIN 15      // Pin number for accessing the receiver's forwards and backwards data.
#define RECEIVER_SPIN_PIN 16    // Pin number for accessing the receiver's forwards and backwards data.

LIS331 accelerometer;         // H3LIS331DL accelerometer. 
int accelX;                   // H3LIS331DL x-axis reading
int accelY;                   // H3LIS331DL y-axis reading
int accelZ;                   // H3LIS331DL z-axis reading

double robotAngle;            // Calculated angle of the robot.

double leftDrivePower;        // Left-side drive power. Tells the Left-side drive how fast to spin.
double rightDrivePower;       // Right-side drive power. Tells the Right-side drive how fast to spin.

int rx_forwardsBackwards;     // Forwards/Backwards control. Receiver signal for gathering forwards and backwards movement. -100 if backwards, 100 is forwards.
int rx_leftRight;             // Left/Right control. Receiver signal for gathering left and right movement. -100 is left, 100 is right.
int rx_spinSpeed;             // Spin control. Receiver signal for gathering what speed the robot should spin at. -100 up to 0 is arcade/calibrate drive, 0 to 100 is spin drive.

enum DriveState {
  Disabled,                   // Robot is disabled. Will not move.
  Arcade,                     // Robot is in arcade drive. Spin will not work. Used to calibrate augmented forwards.
  Meltybrain                  // Robot is in meltybrain drive. Spin will work and uses a calculated augmented robot centric drive.
};

/* 
 *  readReceiver
 *  
 *  Reads each analog pin and sets them to their respective variable
 */
void readReceiver() {
  rx_forwardsBackwards = analogRead(RECEIVER_FB_PIN);
  rx_leftRight = analogRead(RECEIVER_LR_PIN);
  rx_spinSpeed = analogRead(RECEIVER_SPIN_PIN);
}

void calculateRotation() {

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

  // Configues SPI pins for communication with the H3LIS331DL
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(SCL_PIN, OUTPUT);
  
  SPI.begin();
  accelerometer.setSPICSPin(CS_PIN);

  accelerometer.begin(LIS331::USE_SPI);
}

void loop() {
}           
