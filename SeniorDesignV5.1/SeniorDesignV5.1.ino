#include <PID_v1.h>

/**
 * Author: Elena Chong
 * Date: 4/26/2016
 * Description:
 * Read data from 10k NTC thermistor; Its resistance decrements as temperature increases.
 * Read encoder position for DC gearmotor with quadrature encoder chip
 * Power 3 12V fans with PWM
 * Control servo motor with PID controller
 * Contact: elenachong93@gmail.com
 */

/**
 * Connect V+ = 5v
 * Fan pin from transistor to PWM 9
 * Resistor pin from thermistor to Analog 0
 * Fan positive lead to power supply positive
 *
 */
//////////////// static variables /////////////////
const unsigned long baudrate = 115200;
#define encoder0dir  21
#define encoder0clk  20
#define motorEnable 10
#define logicPin1 2 // for controlling motor direction with Hbridge
#define logicPin2 3 // for controlling motor direction with Hbridge
#define hbridgeD1 4
#define hbridgeD2 5
#define SERIESRESISTOR 10000 // the value of the resistor used for voltage divider
#define AMBIENTTEMP A0 // thermistor analog pin
#define PRIMARYTEMP A1 // thermistor analog pin
#define SECONDARYTEMP A2 // thermistor analog pin
#define THERMISTORNOMINAL 10000 // thermistor resistance at 25 degrees C
#define TEMPERATURENOMINAL 25 // temp. for nominal resistance (almost always room temperature 25 C)
#define BCOEFFICIENT 3950 // Beta Coefficient from thermistor datasheet
#define FANSPIN 9
#define FANSPOWER 200 // 0 TO 255, but keep it midway. No need to run above 150. If do, the power fluctuate!
const int N = 5; // number of samples for averaging
//////////////// define variables /////////////////
volatile int encoderACount = 0;
volatile boolean changeFlag = false;
//////////////// global variables /////////////////
int raw = 0;
int samples[N];
double analog;
float vout = 0;
float R2 = 0;
float buffer = 0;
float T;
String message;
double motorSpeed = 0;
float kp = 1.8;
float ki = 1.95;
float kd = 0.65;
double travelStep = 0;
double encoderACount0 = 0; // could have just cast volatile int to double for PID
unsigned long timeoff = 0;
PID myPID(&encoderACount0, &motorSpeed, &travelStep, kp, ki, kd, DIRECT);
//PID myPID(&travelStep, &motorSpeed, &encoderACount0, kp, ki, kd, DIRECT);
boolean rot;
int fansPower;
boolean motorON = false;

void setup() { // This happens just once during the entire program
  Serial.begin(baudrate);

  pinMode(FANSPIN, OUTPUT);  // Set output pin for sending signal to the TIP31

  // Check MC33887-783025 datasheet - Hbridge to drive servo motor
  pinMode(motorEnable, OUTPUT);  // Set output pin for sending signal to motor speed
  pinMode(logicPin1, OUTPUT); // set output pin for sending signal to Hbridge direction
  pinMode(logicPin2, OUTPUT);// set output pin for sending signal to Hbridge direction
  pinMode(hbridgeD1, OUTPUT); // set output pin for sending signal to Hbridge D1
  pinMode(hbridgeD2, OUTPUT); // set output pin for sending signal to Hbridge D2
  pinMode(encoder0clk, INPUT); // get motor position from encoder
  pinMode(encoder0dir, INPUT); // get motor position from encoder
  attachInterrupt(3, encoderIntA, RISING);// encoder pin on interrupt 3 (pin 20)

  // Reading all three 10k NTC thermistors
  float aTemp = getTemperature(AMBIENTTEMP);
  float pTemp = getTemperature(PRIMARYTEMP);
  float sTemp = getTemperature(SECONDARYTEMP);
  Serial.print("a ");
  Serial.print("\t");
  Serial.println(aTemp);
  
  //Specify the links and initial tuning parameters
  myPID.SetMode(AUTOMATIC); // turn on PID controller for servo driving
  myPID.SetOutputLimits(50, 155);
  myPID.SetTunings(kp,ki,kd);
//  motorSpeed = 0;
}

void loop() { // This happens continuously during the entire program
  // two way serial communication between arduino and GUI
  serialSend();
  serialReceive();

  // flag for encoder to check position and direction
  if (changeFlag) {
    changeFlag = false;
    Serial.println(encoderACount);
  }

  // Set motor to stop at target step.
  myPID.Compute();
  reachStepTarget();
//  Serial.print("Motor Speed:");
//  Serial.print("\t");
//  Serial.println(motorSpeed);
}

/**
 * Function that control the motor movement to reach target step value
 */
void reachStepTarget() {
  if (motorON) {
    if (encoderACount != travelStep) {
      if (travelStep > 0 && encoderACount < travelStep) {
        myPID.Compute(); // Set the motorSpeed to reach travelStep.
        Serial.println(motorSpeed);
        motorReverse();
      }
      else if (travelStep < 0 && travelStep < encoderACount) {
        myPID.Compute();
        Serial.println(motorSpeed);
        motorForward();
      }
      else {
        motorStop();
      }
      //        Serial.print("x ");
      //        Serial.print("\t");
      //        Serial.println(encoderACount);
    }
  }
}


/*
 * Function that does the data communication between the Arduino and the GUI
 * The arduino is sending the temperature data to the GUI
 */
void serialSend() {
  float aTemp = getTemperature(AMBIENTTEMP);
  float pTemp = getTemperature(PRIMARYTEMP);
  float sTemp = getTemperature(SECONDARYTEMP);
  if (aTemp == -1) {
    Serial.println("Error reading temperature!");
    return;
  }
  if (pTemp == -1) {
    Serial.println("Error reading temperature!");
    return;
  }
  if (sTemp == -1) {
    Serial.println("Error reading temperature!");
    return;
  }
  if (millis() - timeoff >= 2000) { // send temp every two seconds
    Serial.print("a ");
    Serial.print("\t");
    Serial.println(aTemp);
    Serial.print("p ");
    Serial.print("\t");
    Serial.println(pTemp);
    Serial.print("s ");
    Serial.print("\t");
    Serial.println(sTemp);
    timeoff = millis();
  }
}

/*
 * Read what is received from the serial communication. GUI sending data to Arduino.
 */
void serialReceive() {

  while (Serial.available() > 0)
  {
    message = Serial.readString();
  }
  if (message.equals("FN")) {
//    fansPower = 150;
    analogWrite(FANSPIN, fansPower);
  }
  if (message.equals("FF")) {
    fansPower = 0;
    analogWrite(FANSPIN, fansPower);
  }
  if (message.equals("MF")) {
    motorStop();
  }
  if (message.equals("MN")) {
    motorON = true;
  }
  if (message.startsWith("l")) {
    message.replace("l ", "");
    travelStep = message.toInt();
    Serial.println(travelStep);
  }
  if (message.startsWith("p")) {
    message.replace("p ", "");
    fansPower = message.toInt();
    fansPower = map(fansPower, 0, 100, 40, 200);  // fans are limited to pwm 50 to 200
    Serial.println(fansPower);
  }
  Serial.flush();     // * clear any random data from the serial buffer
}

/**
 * Reading analog temperature from thermistor
 */
float getTemperature(int THERMISTORPIN) {
  // Average the samples
  float average;
  int i;
  // Take N samples and store in array
  for (i = 0; i < N; i++) {
    samples[i] = analogRead(THERMISTORPIN);
  }
  // Average all the samples
  average = 0;
  for (i = 0; i < N; i++) {
    average = average + samples[i];
  }
  average = average / N;

  if (average) {
    // convert the value to resistance
    average = (1023 / average)  - 1;
    average = SERIESRESISTOR / average;

    // Calculate Temperature
    float steinhart;
    steinhart = average / SERIESRESISTOR;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    return steinhart;
  }
  return -1;
}

/*
 *  read the encoder direction from the quadrature encoder LS7184 chip
 */
void encoderIntA() {
  if (digitalRead(encoder0dir) == HIGH) {
    encoderACount++;
    encoderACount0++;
  }
  else {
    encoderACount--;
    encoderACount0--;
    changeFlag = true;
  }

}

void motorForward() {
//  motorSpeed = 155;
  digitalWrite(motorEnable, HIGH); // Enable Pin high for the motor to work.
  analogWrite(logicPin1, motorSpeed); // PWM
  digitalWrite(logicPin2, HIGH); // Digital
  digitalWrite(hbridgeD1, LOW);
  digitalWrite(hbridgeD2, HIGH);
}

void motorReverse() {
//  motorSpeed = 155;
  digitalWrite(motorEnable, HIGH); // Enable Pin high for the motor to work.
  digitalWrite(logicPin1, HIGH); // Digital
  analogWrite(logicPin2, motorSpeed); // PWM
  digitalWrite(hbridgeD1, LOW);
  digitalWrite(hbridgeD2, HIGH);
}
void motorStop() {
  motorON = false;
  digitalWrite(logicPin2, LOW);
  digitalWrite(logicPin1, LOW);
  digitalWrite(motorEnable, LOW); // Enable Pin high for the motor to work.
}

