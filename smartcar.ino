#include <IRremote.h>


#define ENABLE_LEFT 5
#define ENABLE_RIGHT 6
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

void setup() {
  // put your setup code here, to run once:

  pinMode(ENABLE_LEFT, OUTPUT);
  pinMode(ENABLE_RIGHT, OUTPUT);
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
}

void setPower() {
  analogWrite(ENABLE_LEFT, motorPower);
  analogWrite(ENABLE_RIGHT, motorPower);
}

void forward() {
  analogWrite(ENABLE_LEFT, motorPower);
  digitalWrite(INPUT_1, LOW);
  digitalWrite(INPUT_2, HIGH);
  digitalWrite(INPUT_R1, HIGH);
  digitalWrite(INPUT_R2, LOW);
  
  analogWrite(ENABLE_RIGHT, motorPower);
  digitalWrite(INPUT_3, HIGH);
  digitalWrite(INPUT_4, LOW);
  digitalWrite(INPUT_R3, LOW);
  digitalWrite(INPUT_R4, HIGH);
}

void reverse() {
  analogWrite(ENABLE_LEFT, motorPower);
  digitalWrite(INPUT_1, HIGH);
  digitalWrite(INPUT_2, LOW);
  digitalWrite(INPUT_R1, LOW);
  digitalWrite(INPUT_R2, HIGH);
  
  analogWrite(ENABLE_RIGHT, motorPower);
  digitalWrite(INPUT_3, LOW);
  digitalWrite(INPUT_4, HIGH);
  digitalWrite(INPUT_R3, HIGH);
  digitalWrite(INPUT_R4, LOW);
}

void brake() {
  analogWrite(ENABLE_LEFT, 0);
  digitalWrite(INPUT_1, HIGH);
  digitalWrite(INPUT_2, HIGH);
  digitalWrite(INPUT_R1, HIGH);
  digitalWrite(INPUT_R2, HIGH);
  
  analogWrite(ENABLE_RIGHT, 0);
  digitalWrite(INPUT_3, HIGH);
  digitalWrite(INPUT_4, HIGH);
  digitalWrite(INPUT_R3, HIGH);
  digitalWrite(INPUT_R4, HIGH);
}

void coast() {
  analogWrite(ENABLE_LEFT, 0);
  digitalWrite(INPUT_1, LOW);
  digitalWrite(INPUT_2, LOW);
  digitalWrite(INPUT_R1, LOW);
  digitalWrite(INPUT_R2, LOW);
  
  analogWrite(ENABLE_RIGHT, 0);
  digitalWrite(INPUT_3, LOW);
  digitalWrite(INPUT_4, LOW);
  digitalWrite(INPUT_R3, LOW);
  digitalWrite(INPUT_R4, LOW);
}

void right() {
  analogWrite(ENABLE_LEFT, motorPower);
  digitalWrite(INPUT_1, HIGH);
  digitalWrite(INPUT_2, LOW);
  digitalWrite(INPUT_R1, LOW);
  digitalWrite(INPUT_R2, HIGH);
  
  analogWrite(ENABLE_RIGHT, motorPower);
  digitalWrite(INPUT_3, HIGH);
  digitalWrite(INPUT_4, LOW);
  digitalWrite(INPUT_R3, LOW);
  digitalWrite(INPUT_R4, HIGH);
}

void left() {
  analogWrite(ENABLE_LEFT, motorPower);
  digitalWrite(INPUT_1, LOW);
  digitalWrite(INPUT_2, HIGH);
  digitalWrite(INPUT_R1, HIGH);
  digitalWrite(INPUT_R2, LOW);
  
  analogWrite(ENABLE_RIGHT, motorPower);
  digitalWrite(INPUT_3, LOW);
  digitalWrite(INPUT_4, HIGH);
  digitalWrite(INPUT_R3, HIGH);
  digitalWrite(INPUT_R4, LOW);
}

void strafeLeft() {
  analogWrite(ENABLE_LEFT, motorPower);
  digitalWrite(INPUT_1, LOW);
  digitalWrite(INPUT_2, HIGH);
  digitalWrite(INPUT_R1, LOW);
  digitalWrite(INPUT_R2, HIGH);
  
  analogWrite(ENABLE_RIGHT, motorPower);
  digitalWrite(INPUT_3, LOW);
  digitalWrite(INPUT_4, HIGH);
  digitalWrite(INPUT_R3, LOW);
  digitalWrite(INPUT_R4, HIGH);
}

void strafeRight() {
  analogWrite(ENABLE_LEFT, motorPower);
  digitalWrite(INPUT_1, HIGH);
  digitalWrite(INPUT_2, LOW);
  digitalWrite(INPUT_R1, HIGH);
  digitalWrite(INPUT_R2, LOW);
  
  analogWrite(ENABLE_RIGHT, motorPower);
  digitalWrite(INPUT_3, HIGH);
  digitalWrite(INPUT_4, LOW);
  digitalWrite(INPUT_R3, HIGH);
  digitalWrite(INPUT_R4, LOW);
}

void loop() {
  
  if (IrReceiver.decode()) {
    lastIRCode = IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume();
    
    lastIRCodeTime = millis();
    Serial.println(lastIRCode);

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
      coast();
    }
    
    // Power ramp up
    if (commandTime > 800 && motorPower < 250) {
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
    }
  }
}
