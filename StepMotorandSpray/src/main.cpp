#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <AccelStepper.h>

const int pwmPin = 9;
const unsigned int desiredFrequency = 10000;

volatile int flow_frequency;
float vol = 0.0, l_minute;

unsigned char flowsensor = 2;
unsigned long currentTime;
unsigned long cloopTime;

double Setpoint;
double Input;
double Output;
double Kp = 1, Ki = 0.01, Kd = 10;

#define STEPS_PER_REVOLUTION 200
#define STEP_PIN 4
#define DIR_PIN 3
#define ENABLE_PIN 7

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setupTimer2() {
  TCCR2A = _BV(COM2A1) | _BV(WGM20) | _BV(WGM21);
  TCCR2B = (TCCR2B & B11111000) | B00000010; // Added parentheses
  OCR2A = (16000000 / 8 / desiredFrequency) - 1;
}

void flow() {
  flow_frequency++;
}

void rotateDegrees(float degrees); // Function prototype

void setup() {
  pinMode(pwmPin, OUTPUT);
  Serial.begin(9600);
  setupTimer2();
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);
  attachInterrupt(digitalPinToInterrupt(flowsensor), flow, RISING);
  Serial.begin(9600);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Water Flow Meter");
  lcd.setCursor(0, 1);
  lcd.print("Circuit Digest");
  currentTime = millis();
  cloopTime = currentTime;
  Input = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  Serial.begin(9600);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
}

void loop() {
  currentTime = millis();
  if (currentTime >= (cloopTime + 1000)) {
    cloopTime = currentTime;
    if (flow_frequency != 0) {
      l_minute = (flow_frequency / 98.00);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Rate: ");
      lcd.print(l_minute);
      lcd.print(" L/M");
      l_minute = l_minute / 60;
      lcd.setCursor(0, 1);
      vol = vol + l_minute;
      lcd.print("Vol:");
      lcd.print(vol);
      lcd.print(" L");
      flow_frequency = 0;
      Serial.print(l_minute, DEC);
      Serial.println(" L/Min");
    } else {
      Serial.println(" flow rate = 0 ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Rate: ");
      lcd.print(flow_frequency);
      lcd.print(" L/M");
      lcd.setCursor(0, 1);
      lcd.print("Vol:");
      lcd.print(vol);
      lcd.print(" L");
    }
  }

  if (Serial.available() > 0) {
    char signal = Serial.read();

    if (signal == 'M') {
      rotateDegrees(-90);
      delay(5000);
      rotateDegrees(90);
      delay(1000);
      rotateDegrees(90);
      delay(5000);
      rotateDegrees(-90);
      delay(1000);
      Serial.println(" okkkkkk");
    }

    if (signal == 'S') {
      Setpoint = 200;
      myPID.Compute();
      analogWrite(pwmPin, Output);
      delay(5000);
      analogWrite(pwmPin, 0);
    }
  }
}

void rotateDegrees(float degrees) {
  long targetPosition = stepper.currentPosition() + (degrees * STEPS_PER_REVOLUTION / 360);
  stepper.moveTo(targetPosition);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    delay(20);
  }
}

