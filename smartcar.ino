#include <IRremote.h>
#include <avr/interrupt.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define ENABLE_LEFT_FRONT 6
#define ENABLE_RIGHT_FRONT 5
#define ENABLE_LEFT_REAR 2
#define ENABLE_RIGHT_REAR 4
#define INPUT_1 11
#define INPUT_2 9
#define INPUT_3 8
#define INPUT_4 7
#define INPUT_R1 28
#define INPUT_R2 26
#define INPUT_R3 24
#define INPUT_R4 22

#define RECV_PIN 12

#define CONTINUE_CODE 0
#define FORWARD_CODE 3108437760
#define REVERSE_CODE 3927310080
#define LEFT_CODE 3141861120
#define RIGHT_CODE 3158572800
#define OK_CODE 3208707840
#define STAR_CODE 2417041970
#define POUND_CODE 3041591040
#define ONE_CODE 3910598400 
#define THREE_CODE 4061003520 
#define SEVEN_CODE 4144561920

unsigned long lastIRCode = 0;
unsigned long lastIRCodeTime = 0;
unsigned long currentIRCode = 0;
unsigned long currentIRCodeTime = 0;
int motorPower = 0;

// Motors

#define MOTOR_COUNT 4

int enablePinArr[MOTOR_COUNT] = {ENABLE_LEFT_FRONT, ENABLE_RIGHT_FRONT, ENABLE_LEFT_REAR, ENABLE_RIGHT_REAR};
int inputPin1Arr[MOTOR_COUNT] = {INPUT_2, INPUT_3, INPUT_R1, INPUT_R4};
int inputPin2Arr[MOTOR_COUNT] = {INPUT_1, INPUT_4, INPUT_R2, INPUT_R3};

int motorPowerArr[MOTOR_COUNT] = {0, 0, 0, 0};

// PID

#define Kp 1.0
#define Ki 10.0
#define Kd 0.0
#define FF 0.2
#define FF_NOPID 0.9

#define PID_INTERVAL_MILLIS 10
#define MAX_POWER 255

int motorSpeedSetpointsRpm[MOTOR_COUNT] = {0, 0, 0, 0};

#define ERROR_HISTORY_COUNT 3
float errorArr[MOTOR_COUNT][ERROR_HISTORY_COUNT] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

bool pidEnable = true;

// Encoders

#define ENCODER_COUNTS_PER_REV 40

#define ENCODER_COUNT 4
unsigned long encoderIntervalMicros[ENCODER_COUNT] = { UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX };
unsigned long encoderLastChangeTimeMicros[ENCODER_COUNT];
int encoderSpeedRpm[ENCODER_COUNT];

uint8_t lastPinK = 0;

volatile bool encodersUpdated = false;

// Interrupt for encoders
// Port K pin change interrupt
// Analog pins 8-16
ISR(PCINT2_vect)
{
  unsigned long timeMicros = micros();
  uint8_t pinK = PINK;
  uint8_t change = lastPinK ^ pinK;

  for (int i = 0; i < ENCODER_COUNT; i++)
  {
    if (change & (1 << i))
    {
      // Exponential smoothing
      encoderIntervalMicros[i] = (encoderIntervalMicros[i] / 2) + ((timeMicros - encoderLastChangeTimeMicros[i]) / 2);
      encoderLastChangeTimeMicros[i] = timeMicros;
    }
  }

  lastPinK = pinK;
  encodersUpdated = true;
}

// IMU
MPU6050 mpu;
#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// IMU Interrupt
// TODO: is this needed?
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  // put your setup code here, to run once:

  pinMode(ENABLE_LEFT_FRONT, OUTPUT);
  pinMode(ENABLE_RIGHT_FRONT, OUTPUT);
  pinMode(INPUT_1, OUTPUT);
  pinMode(INPUT_2, OUTPUT);
  pinMode(INPUT_3, OUTPUT);
  pinMode(INPUT_4, OUTPUT);
  pinMode(INPUT_R1, OUTPUT);
  pinMode(INPUT_R2, OUTPUT);
  pinMode(INPUT_R3, OUTPUT);
  pinMode(INPUT_R4, OUTPUT);

  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);
  Serial.begin(9600);  // debug output at 9600 baud

  // Pin change interrupt
  PCICR |= 1 << PCIE2;
  PCMSK2 = 0b00001111;

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Initializing DMP..."));

  uint8_t devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}

void setMotorPower(int motor, int power)
{
  motorPowerArr[motor] = power;
  analogWrite(enablePinArr[motor], abs(power));
  if (power > 0)
  {
    digitalWrite(inputPin1Arr[motor], HIGH);
    digitalWrite(inputPin2Arr[motor], LOW);
  }
  else if (power < 0)
  {
    digitalWrite(inputPin1Arr[motor], LOW);
    digitalWrite(inputPin2Arr[motor], HIGH);
  }
  else
  {
    // Brake
    digitalWrite(inputPin1Arr[motor], HIGH);
    digitalWrite(inputPin2Arr[motor], HIGH);
  }
}

int getMotorDirection(int motor)
{
  // TODO: make this more accurate when changing direction

  int pin1 = digitalRead(inputPin1Arr[motor]);
  int pin2 = digitalRead(inputPin2Arr[motor]);
  if (pin1 == pin2)
  {
    return 0;
  }
  else if (pin1 == HIGH && pin2 == LOW)
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

void setPower() {
  analogWrite(ENABLE_LEFT_FRONT, motorPower);
  analogWrite(ENABLE_RIGHT_FRONT, motorPower);
  analogWrite(ENABLE_LEFT_REAR, motorPower);
  analogWrite(ENABLE_RIGHT_REAR, motorPower);
}

void setMotorSpeed(int motor, int rpm)
{
  for (int i = 0; i < ERROR_HISTORY_COUNT; i++)
  {
    errorArr[motor][i] = 0;
  }
  if (pidEnable)
  {
    setMotorPower(motor, FF * rpm);
  }
  else
  {
    setMotorPower(motor, FF_NOPID * rpm);
  }
  motorSpeedSetpointsRpm[motor] = rpm;
}

void forward() {
  setMotorSpeed(0, motorPower);
  setMotorSpeed(1, motorPower);
  setMotorSpeed(2, motorPower);
  setMotorSpeed(3, motorPower);
}

void reverse() {
  setMotorSpeed(0, -motorPower);
  setMotorSpeed(1, -motorPower);
  setMotorSpeed(2, -motorPower);
  setMotorSpeed(3, -motorPower);
}

void brake() {
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  setMotorSpeed(2, 0);
  setMotorSpeed(3, 0);
}

void right() {
  setMotorSpeed(0, motorPower);
  setMotorSpeed(1, -motorPower);
  setMotorSpeed(2, motorPower);
  setMotorSpeed(3, -motorPower);
}

void left() {
  setMotorSpeed(0, -motorPower);
  setMotorSpeed(1, motorPower);
  setMotorSpeed(2, -motorPower);
  setMotorSpeed(3, motorPower);
}

void strafeLeft() {
  setMotorSpeed(0, -motorPower);
  setMotorSpeed(1, motorPower);
  setMotorSpeed(2, motorPower);
  setMotorSpeed(3, -motorPower);
}

void strafeRight() {
  setMotorSpeed(0, motorPower);
  setMotorSpeed(1, -motorPower);
  setMotorSpeed(2, -motorPower);
  setMotorSpeed(3, motorPower);
}

void printEncoders()
{
  Serial.print("Encoder  ");
  for (int i = 0; i < ENCODER_COUNT; i++)
  {
    Serial.print(encoderSpeedRpm[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void printSetpoints()
{
  Serial.print("Setpoint ");
  for (int i = 0; i < ENCODER_COUNT; i++)
  {
    Serial.print(motorSpeedSetpointsRpm[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void printErrors(int j = 0)
{
  Serial.print("Error    ");
  for (int i = 0; i < ENCODER_COUNT; i++)
  {
    Serial.print(errorArr[i][j]);
    Serial.print(" ");
  }
  Serial.println();
}

void printPower()
{ 
  Serial.print("Power    ");
  for (int i = 0; i < ENCODER_COUNT; i++)
  {
    Serial.print(motorPowerArr[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void loop() {
  
  if (IrReceiver.decode()) {
    lastIRCode = IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume();
    
    lastIRCodeTime = millis();
    //Serial.println(lastIRCode);

    if (lastIRCode == CONTINUE_CODE) {
      
    } else {
      currentIRCode = lastIRCode;
      currentIRCodeTime = millis();
    }
    
    if (lastIRCode == FORWARD_CODE) {
      motorPower = 100;
      forward();
    } else if (lastIRCode == REVERSE_CODE) {
      motorPower = 100;
      reverse();
    } else if (lastIRCode == LEFT_CODE) {
      motorPower = 150;
      strafeLeft();
    } else if (lastIRCode == RIGHT_CODE) {
      motorPower = 150;
      strafeRight();
    } else if (lastIRCode == ONE_CODE) {
      motorPower = 150;
      left();
    } else if (lastIRCode == THREE_CODE) {
      motorPower = 150;
      right();
    } else if (lastIRCode == OK_CODE) {
      motorPower = 0;
      brake();
    } else if (lastIRCode == SEVEN_CODE)
    {
      pidEnable = !pidEnable;
    }


  }
  
  if (currentIRCodeTime > 0) {
    unsigned long commandTime = millis() - currentIRCodeTime;
    unsigned long irCodeTime = millis() - lastIRCodeTime;

    // Command continuation
    if (!(irCodeTime < 200 && (lastIRCode == CONTINUE_CODE || lastIRCode == currentIRCode))) {
      //Serial.println("Cancelling command");
      currentIRCode = 0;
      currentIRCodeTime = 0;
      motorPower = 0;
      commandTime = 0;
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        setMotorSpeed(i, 0);
      }
    }
    
    // Power ramp up
    /*if (commandTime > 800 && motorPower < 250) {
      Serial.println("Increasing power to 250");
      motorPower = 250;
      setPower();
    } else if (commandTime > 600 && motorPower < 200) {
      Serial.println("Increasing power to 200");
      motorPower = 200;
      setPower();
    } else if (commandTime > 400 && motorPower < 150) {
      Serial.println("Increasing power to 150");
      motorPower = 150;
      setPower();
    }*/
  }

  if (encodersUpdated)
  {
    unsigned long localEncoderIntervalMicros[ENCODER_COUNT];
    unsigned long localEncoderLastChangeTimeMicros[ENCODER_COUNT];

    noInterrupts();
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      localEncoderIntervalMicros[i] = encoderIntervalMicros[i];
      localEncoderLastChangeTimeMicros[i] = encoderLastChangeTimeMicros[i];
    }
    interrupts();

    // Process encoders
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      // Encoder timeout
      if (localEncoderLastChangeTimeMicros[i] + 10 * localEncoderIntervalMicros[i] < micros())
      {
        localEncoderIntervalMicros[i] = encoderIntervalMicros[i] = UINT32_MAX;
      }

      // Encoder stopped
      if (UINT32_MAX == localEncoderIntervalMicros[i])
      {
        encoderSpeedRpm[i] = 0;
        continue;
      }

      encoderSpeedRpm[i] = (60 * 1000000 / ENCODER_COUNTS_PER_REV) / localEncoderIntervalMicros[i];
    }
  }

  // PID control
  static unsigned long lastPidUpdateMillis = 0;
  unsigned long curMillis = millis();
  unsigned long dtMillis = curMillis - lastPidUpdateMillis;

  // Try to keep a consistent interval
  if (dtMillis >= PID_INTERVAL_MILLIS)
  {
    float dtSeconds = dtMillis / 1000.0;

    float A0 = Kp + Ki*dtSeconds + Kd/dtSeconds;
    float A1 = -Kp - 2*Kd/dtSeconds;
    float A2 = Kd/dtSeconds;

    /*printPower();
    printSetpoints();
    printEncoders();*/

    for (int i = 0; i < MOTOR_COUNT; i++)
    {

      errorArr[i][2] = errorArr[i][1];
      errorArr[i][1] = errorArr[i][0];
      errorArr[i][0] = motorSpeedSetpointsRpm[i] - getMotorDirection(i) * encoderSpeedRpm[i];

      int newMotorPower = motorPowerArr[i]
        + A0 * errorArr[i][0]
        + A1 * errorArr[i][1]
        + A2 * errorArr[i][2];

      // Set motor power to zero if it's in the opposite direction as the setpoint
      if (motorSpeedSetpointsRpm[i] > 0 && newMotorPower < 0 || motorSpeedSetpointsRpm[i] < 0 && newMotorPower > 0)
      {
        newMotorPower = 0;
      }

      // Clip max power
      if (newMotorPower > MAX_POWER)
      {
        newMotorPower = MAX_POWER;
      }

      if (pidEnable)
      {
        setMotorPower(i, newMotorPower);
      }
    }

    /*printErrors();
    printPower();
    Serial.println();
    Serial.println();*/

    lastPidUpdateMillis = curMillis;
  }
  
  static unsigned long lastPrintMillis = 0;
  unsigned long curPrintMillis = millis();

  if (curPrintMillis - lastPrintMillis > 100)
  {
    /*printSetpoints();
    printEncoders();
    printErrors();
    printPower();
    Serial.println();
    Serial.println();
    */

    /*
    // Display left wheel PID info
    Serial.print(errorArr[0][0]);
    Serial.print(" ");
    Serial.print(motorPowerArr[0]);
    Serial.print(" ");
    Serial.print(motorSpeedSetpointsRpm[0]);
    Serial.print(" ");
    Serial.println(getMotorDirection(0) * encoderSpeedRpm[0]);*/

    if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      Quaternion q;
      VectorFloat gravity;
      float ypr[3];
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
    }

    lastPrintMillis = curPrintMillis;
  }

  
}
