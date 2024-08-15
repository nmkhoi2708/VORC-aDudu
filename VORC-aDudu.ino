#include <PS2X_lib.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PS2X instance
PS2X ps2x;

// Define motor and servo pins
const int leftMotorPin1 = 8;
const int leftMotorPin2 = 9;
const int rightMotorPin1 = 10;
const int rightMotorPin2 = 11;
const int middleMotorPin1 = 12;
const int middleMotorPin2 = 13;
const int intakeMotorPin1 = 14;
const int intakeMotorPin2 = 15;
const int servoPin = 2;

// Initialize servo object
Servo servo1;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Function to set motor power
void setMotorPower(int motorPin1, int motorPin2, double power) {
  if(power >= 0 ) {
    pwm.setPWM(motorPin1, motorPin2, power* 4096); // power multiplied by 4096 because its range is 0 -> 1 
    pwm.setPWM(motorPin1, motorPin2, 0);
  }
  else if ( power < 0 ) {
    pwm.setPWM(motorPin1, motorPin2, 0);
    pwm.setPWM(motorPin1, motorPin2, power * -4096); // if power < 0, go reverse
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize PS2 controller
  int error = -1;
  for (int i = 0; i < 10; i++) {
    error = ps2x.config_gamepad(12, 11, 10, 13, true, true); // Data, Command, Attention, Clock, Pressures, Rumble
    if (error == 0) break;
    delay(100);
  }
  if (error) {
    Serial.println("PS2 Controller not found, check wiring!");
    while (1); // Halt the program
  }
  
  pwm.begin(); //Initialize PCA9685 
  pwm.setOscillatorFrequency(27000000); // Initialize Oscillator Frequency
  pwm.setPWMFreq(50);// Initialize PWM Frequency
  Wire.setClock(400000); // Initialize i2c communicate speed

  // Initialize motor pins as outputs
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(middleMotorPin1, OUTPUT);
  pinMode(middleMotorPin2, OUTPUT);
  pinMode(intakeMotorPin1, OUTPUT);
  pinMode(intakeMotorPin2, OUTPUT);

  // Attach servo to pin
  servo1.attach(servoPin);
  
  // Set initial servo position
  servo1.write(180); // Neutral position
}

void loop() {
  // Read gamepad inputs
  ps2x.read_gamepad(false, 0);

  // Set speed ratio ( normal or boosted )
  double spdRatio = 0.6; //normal speed
  if (ps2x.Button(PSB_R1)) spdRatio = 1.0; // when PSB_R1 is holded, spdRatio become boosted

  // Calculate the ratio to prevent the motor power value to go over 255
  double y = sense(-ps2x.Analog(PSS_LY)) ; // imput analog value from the y axis of the left joystick 
  double x = sense(ps2x.Analog(PSS_LX)) ; // imput analog value from, the x axis of left joystick
  double rx = sense(ps2x.Analog(PSS_RX)) ; // imput analog value from the x axis of the right joystick
  double ratio = max(abs(y) + abs(rx), 255); // if a motor power go pass 255, ratio = 255 

  // Calculate motor power by inputs from PS2 gamepad
  double leftPower = (y + rx) / ratio * spdRatio; // multiplied by spdRatio to set normal or boosted speed
  double rightPower = (y - rx) / ratio * spdRatio; // divided by ratio to keep these value equal or lower than 1 
  double midPower = x * spdRatio; // doesn't need to be divided by ratio because it is not controlled by any values except from x so it cannot be over 1 or under -1

  // Set motor power
  setMotorPower(leftMotorPin1, leftMotorPin2, leftPower); // set left motor power
  setMotorPower(rightMotorPin1, rightMotorPin2, rightPower); // set right motor power
  setMotorPower(middleMotorPin1, middleMotorPin2, midPower); // set middle motor power

  // Control intake motor
  if (ps2x.Button(PSB_PAD_UP)) { // if the dpad up button is pressed
    setMotorPower(intakeMotorPin1, intakeMotorPin2, 1); // turn on the intake
  } else if (ps2x.Button(PSB_PAD_DOWN)) { // if the dpad down button is pressed
    setMotorPower(intakeMotorPin1, intakeMotorPin2, 0); // turn off the intake
  }

  //Control Servo
  if (ps2x.Button(PSB_TRIANGLE)) {pwm.writeMicroseconds(2,1000); }// Set servo to 0 degree angle
  if (ps2x.Button(PSB_CIRCLE )) {pwm.writeMicroseconds(2,1500);} // Set servo to 90 degree angle 
  if (ps2x.Button(PSB_CROSS)) {pwm.writeMicroseconds(2,2000); }// Set servo to 180 degree angle

  delay(50); // Small delay to prevent overwhelming the PS2 controller
}

double sense ( double x )
{
  return ( abs( x - 128.0 ) < 10  ) ? 0 : x ; // return 0 if the joystick value is lower than 10, to eliminate errors while robot is not moving
}
