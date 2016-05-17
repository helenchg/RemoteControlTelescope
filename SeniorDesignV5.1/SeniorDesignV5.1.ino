#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
/**
 * Author: Elena Chong & Toba Faseru
 * Date: 5/14/2016
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

#define ledPin 13
#define encoder0dir  21
#define encoder0clk  20
#define motorEnable 10
#define logicPin1 2 //
#define logicPin2 3 //
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
#define HEATERPIN 12
#define HEATERPOWER 255
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
//float kp = 0.15;
float kp =1.65;
float ki =0.0;
float kd =0;
//float ki = 0.02;
//float kd = 0;
double travelStep = 0;
double encoderACount0 = 0; // could have just cast volatile int to double for PID
unsigned long timeoff = 0;
//PID myPID(&travelStep, &motorSpeed, &encoderACount0, kp, ki, kd, DIRECT);
boolean rot;
int fansPower;
boolean motorON = false;
boolean heaterON = false;
//////////////// Variables for interrupt /////////////////
unsigned volatile long viCount, viCount2; // interrupt memory variables
unsigned long iCountH, iCountL;
unsigned volatile long c;
unsigned long freq;
double freqAvg = 0;
int fN =10;
//////////////// PID controllers /////////////////
double setPoint = 400; // desired frequency Hz.
double sds = 400;
//PID myPID(&encoderACount0, &motorSpeed, &travelStep, kp, ki, kd, DIRECT);
PID speedPID(&freqAvg, &motorSpeed, &setPoint, kp, ki, kd, DIRECT);
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;


void setup() { // This happens just once during the entire program
  Serial.begin(baudrate);
  pinMode(ledPin, OUTPUT); // LED pin on arduino
  pinMode(FANSPIN, OUTPUT);  // Set output pin for sending signal to the TIP31
  pinMode(HEATERPIN, OUTPUT);  // Set output pin for sending signal to the TIP31
  // Check MC33887-783025 datasheet - Hbridge to drive servo motor
  pinMode(motorEnable, OUTPUT);  // Set output pin for sending signal to motor speed
  pinMode(logicPin1, OUTPUT); // set output pin for sending signal to Hbridge direction
  pinMode(logicPin2, OUTPUT);// set output pin for sending signal to Hbridge direction
  pinMode(hbridgeD1, OUTPUT); // set output pin for sending signal to Hbridge D1
  pinMode(hbridgeD2, OUTPUT); // set output pin for sending signal to Hbridge D2
  pinMode(encoder0clk, INPUT); // get motor position from encoder
  pinMode(encoder0dir, INPUT); // get motor position from encoder
  attachInterrupt(3, encoderIntA, RISING);// encoder pin on interrupt 3 (pin 20)

  // initialize timer4
  TCCR4A = 0;
  TCCR4B = 1 << ICNC4 | 1 << ICES4 | 1 << CS42 | 0 << CS41 | 0 << CS40; // SET PRESCALER TO 256
  TIMSK4 |= (1 << ICIE4);   // enable timer overflow interrupt PIN 49 AS INTERRUPT
  delay(200);
  
  // Reading all three 10k NTC thermistors
  float aTemp = getTemperature(AMBIENTTEMP);
  float pTemp = getTemperature(PRIMARYTEMP);
  float sTemp = getTemperature(SECONDARYTEMP);
  Serial.print("a ");
  Serial.print("\t");
  Serial.println(aTemp);
  
  //Specify the output limits and initial tuning parameters
//  speedPID.SetMode(AUTOMATIC); // turn on PID controller for servo driving // Comment out if want speed PID
//  speedPID.SetOutputLimits(50, 250);  // Comment out if want speed PID
//  speedPID.SetTunings(kp,ki,kd);      // Comment out if want speed PID
  motorSpeed = 100;
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

  if(heaterON){
    analogWrite(HEATERPIN, HEATERPOWER);
  }
  // Set motor to stop at target step.
  speedPID.Compute();
  reachStepTarget();
//  Serial.print("Motor Speed:");
//  Serial.print("\t");
//  Serial.println(motorSpeed);
}

/**
 * Function that control the motor movement to reach target step value
 */
void reachStepTarget() {
//  frequencyMeasurement();  // Comment out if want speed PID
  double error = abs(freqAvg - setPoint);
  if (motorON) {
    if (encoderACount != travelStep) {
      if (travelStep > 0 && encoderACount < travelStep) {
        motorSpeed = 100;     // Comment out if want speed PID
//        speedPID.Compute(); // Comment out if want speed PID
        Serial.println(motorSpeed);
//        Serial.print("Avg Freq: ");   // Comment out if want speed PID
//        Serial.println(freqAvg, DEC); // Comment out if want speed PID 
        motorForward();

      }
      else if (travelStep < 0 && travelStep < encoderACount) {
        motorSpeed = 100;     // Comment out if want speed PID
//        speedPID.Compute(); // Comment out if want speed PID
        Serial.println(motorSpeed);
//        Serial.print("Avg Freq: ");   // Comment out if want speed PID
//        Serial.println(freqAvg, DEC); // Comment out if want speed PID
        motorReverse();
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
    if (message.equals("HN")) {
    heaterON = true;
  }
    if (message.equals("HF")) {
    heaterON = false;
  }
  if (message.startsWith("l")) {
    message.replace("l ", "");
    travelStep = message.toInt();
    Serial.println(travelStep);
  }
  if (message.startsWith("p")) {
    message.replace("p ", "");
    fansPower = message.toInt();
    fansPower = map(fansPower, 0, 100, 50, 255);  // fans are limited to pwm 50 to 255
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
//  digitalWrite(logicPin2, HIGH); // Digital
  digitalWrite(hbridgeD1, LOW);
  digitalWrite(hbridgeD2, HIGH);
}

void motorReverse() {
//  motorSpeed = 155;
  digitalWrite(motorEnable, HIGH); // Enable Pin high for the motor to work.
//  digitalWrite(logicPin1, HIGH); // Digital
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

void frequencyMeasurement(){
    cli();
  iCountH = viCount;
  iCountL = viCount2;
  c = (iCountH << 8) | iCountL;
  if(c == 0x0){
    freq = 0;
  }
  else{
    freq = 62500 / c;
  }
  for (int i = 0; i < fN; i++) {
    freqAvg = freqAvg + freq;
  }
  freqAvg = freqAvg / (fN+1);
  
  sei();

//  Serial.print("OLD: ");
//  Serial.print(iCountH, HEX);
//  Serial.println(iCountL, HEX );
//  Serial.print("hex:");
//  Serial.println(c, HEX);
//  Serial.print("Freq:");
//  Serial.print(freq, DEC);
//  Serial.println("Hz");
//  Serial.print("Avg Freq: ");
//  Serial.println(freqAvg, DEC);
}

ISR(TIMER4_CAPT_vect) // interrupt service routine
{
  TCNT4 = 0;   // preload timer
  viCount2 = ICR4L;
  viCount = ICR4H;
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
}

