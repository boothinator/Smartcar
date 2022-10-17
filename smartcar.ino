#include <IRremote.h>
#include <avr/interrupt.h>

#define ENABLE_LEFT_FRONT 5
#define ENABLE_RIGHT_FRONT 6
#define ENABLE_LEFT_REAR 4
#define ENABLE_RIGHT_REAR 2
#define INPUT_1 7
#define INPUT_2 8
#define INPUT_3 9
#define INPUT_4 11
#define INPUT_R1 22
#define INPUT_R2 24
#define INPUT_R3 26
#define INPUT_R4 28

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

float Kp = 0.1, Ki = 0.1, Kd = 0.0;

#define PID_INTERVAL_MILLIS 1000

int motorSpeedSetpointsRpm[MOTOR_COUNT] = {0, 0, 0, 0};

float errorArr[MOTOR_COUNT][3] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

// Encoders

#define ENCODER_COUNTS_PER_REV 40

#define ENCODER_COUNT 4
unsigned long encoderIntervalMicros[ENCODER_COUNT] = { UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX };
unsigned long encoderLastChangeTimeMicros[ENCODER_COUNT];
int encoderSpeedRpm[ENCODER_COUNT];

uint8_t lastPinK = 0;

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
  motorSpeedSetpointsRpm[motor] = rpm;
  //setMotorPower(motor, rpm);
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
  setMotorSpeed(0, -motorPower);
  setMotorSpeed(1, motorPower);
  setMotorSpeed(2, -motorPower);
  setMotorSpeed(3, motorPower);
}

void left() {
  setMotorSpeed(0, motorPower);
  setMotorSpeed(1, -motorPower);
  setMotorSpeed(2, motorPower);
  setMotorSpeed(3, -motorPower);
}

void strafeLeft() {
  setMotorSpeed(0, motorPower);
  setMotorSpeed(1, -motorPower);
  setMotorSpeed(2, -motorPower);
  setMotorSpeed(3, motorPower);
}

void strafeRight() {
  setMotorSpeed(0, -motorPower);
  setMotorSpeed(1, motorPower);
  setMotorSpeed(2, motorPower);
  setMotorSpeed(3, -motorPower);
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
    }


  }
  
  if (currentIRCodeTime > 0) {
    unsigned long commandTime = millis() - currentIRCodeTime;
    unsigned long irCodeTime = millis() - lastIRCodeTime;

    // Command continuation
    if (!(irCodeTime < 200 && (lastIRCode == CONTINUE_CODE || lastIRCode == currentIRCode))) {
      Serial.println("Cancelling command");
      currentIRCode = 0;
      currentIRCodeTime = 0;
      motorPower = 0;
      commandTime = 0;
      setPower();
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        motorSpeedSetpointsRpm[i] = 0;
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

  // Process encoders
  for (int i = 0; i < ENCODER_COUNT; i++)
  {
    // Encoder timeout
    if (encoderLastChangeTimeMicros[i] + 10 * encoderIntervalMicros[i] < micros())
    {
      encoderIntervalMicros[i] = UINT32_MAX;
    }

    // Encoder stopped
    if (UINT32_MAX == encoderIntervalMicros[i])
    {
      encoderSpeedRpm[i] = 0;
      continue;
    }

    encoderSpeedRpm[i] = (60 * 1000000 / ENCODER_COUNTS_PER_REV) / encoderIntervalMicros[i];
  }

  // PID control
  static unsigned long lastPidUpdateMillis = 0;
  unsigned long curMillis = millis();
  unsigned long dtMillis = curMillis - lastPidUpdateMillis;

  // Try to keep a consistent interval
  if (dtMillis >= PID_INTERVAL_MILLIS)
  {
    float dtSeconds = dtMillis / 1000;

    Serial.print("PrvPower ");
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(motorPowerArr[i]);
      Serial.print(" ");
    }
    Serial.println();
    float A0 = Kp + Ki*dtSeconds + Kd/dtSeconds;
    float A1 = -Kp - 2*Kd/dtSeconds;
    float A2 = Kd/dtSeconds;

    Serial.println(A0);
    Serial.println(A1);
    Serial.println(A2);
    
    Serial.print("A0       ");
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(A0 * (motorSpeedSetpointsRpm[i] - encoderSpeedRpm[i]));
      Serial.print(" ");
    }
    Serial.println();
    
    Serial.print("A1       ");
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(A1 * errorArr[i][0]);
      Serial.print(" ");
    }
    Serial.println();
    
    Serial.print("A2       ");
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(A2 * errorArr[i][1]);
      Serial.print(" ");
    }
    Serial.println();

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      errorArr[i][2] = errorArr[i][1];
      errorArr[i][1] = errorArr[i][0];
      errorArr[i][0] = motorSpeedSetpointsRpm[i] - encoderSpeedRpm[i];

      setMotorPower(i, motorPowerArr[i]
        + A0 * errorArr[i][0]
        + A1 * errorArr[i][1]
        + A2 * errorArr[i][2]);
    }

    lastPidUpdateMillis = curMillis;
  }

  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs > 1000)
  {
    Serial.print("Encoder  ");
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(encoderSpeedRpm[i]);
      Serial.print(" ");
    }
    Serial.println();

    Serial.print("Setpoint ");
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(motorSpeedSetpointsRpm[i]);
      Serial.print(" ");
    }
    Serial.println();

    Serial.print("Error    ");
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(errorArr[i][0]);
      Serial.print(" ");
    }
    Serial.println();
    
    Serial.print("Power    ");
    for (int i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(motorPowerArr[i]);
      Serial.print(" ");
    }
    Serial.println();
    Serial.println();

    lastPrintMs = millis();
  }
  
}
