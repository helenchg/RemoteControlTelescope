#include <PID_v1.h>

/**
 * Author: Elena Chong
 * Date: 4/19/2016
 * Description:
 * Read data from 10k NTC thermistor
 * Its resistance decrements as temperature increases.
 * Read encoder position for DC gearmotor with quadrature encoder chip
 * Power 3 12V fans
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
const unsigned long baudrate = 9600;
#define encoder0dir  21
#define encoder0clk  20
#define motorPWM 10
#define logicPin1 2 // for controlling motor direction with Hbridge
#define logicPin2 3 // for controlling motor direction with Hbridge
#define SERIESRESISTOR 10000 // the value of the resistor used for voltage divider
#define THERMISTORPIN A0 // thermistor analog pin
#define THERMISTORNOMINAL 10000 // thermistor resistance at 25 degrees C
#define TEMPERATURENOMINAL 25 // temp. for nominal resistance (almost always room temperature 25 C)
#define BCOEFFICIENT 3950 // Beta Coefficient from thermistor datasheet
#define FANSPIN 9
#define FANSPOWER 120 // 0 TO 255, but keep it midway. No need to run above 150. If do, the power fluctuate!
const int N = 5; // number of samples for averaging
int motorStop = 0;
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
boolean CCW = true;
int motorSpeed = 0;

void setup() {
  // This happens just once during the entire program
  Serial.begin(baudrate);
  pinMode(FANSPIN, OUTPUT);  // Set output pin for sending signal to the TIP31

  pinMode(motorPWM, OUTPUT);  // Set output pin for sending signal to motor speed
  pinMode(logicPin1, OUTPUT); // set output pin for sending signal to Hbridge direction
  pinMode(logicPin2, OUTPUT);// set output pin for sending signal to Hbridge direction

  pinMode(encoder0clk, INPUT); // get motor position from encoder
  pinMode(encoder0dir, INPUT); // get motor position from encoder
  attachInterrupt(3, encoderIntA, RISING);// encoder pin on interrupt 3 (pin 20)
  
  //Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);
  
}

void loop() {
  // This happens continuously during the entire program
  float temp = getTemperature();
  //  float temp1 = getTemperatureNom();
  if (temp == -1) {
    Serial.println("Error reading temperature!");
    return;
  }
  Serial.print("Temp: ");
  Serial.print("\t");
  Serial.println(temp);
  delay(1000);

  while (Serial.available() > 0)
  {
    message = Serial.readString();
  }
  if (message.equals("FANS ON")) {
    analogWrite(FANSPIN, FANSPOWER); // pwn for controlling the speed of fans. Do no exceed 150.
  }
  if (message.equals("FANS OFF")) {
    analogWrite(FANSPIN, 0); // pwn for controlling the speed of fans. Do no exceed 150.
  }
  if (message.equals("MOTOR OFF")) {
    motorSpeed = 0; // pwn for controlling the speed of fans. Do no exceed 150.
  }
  if (message.equals("MOTOR ON")) {
    motorSpeed = 200; // pwn for controlling the speed of fans. Do no exceed 150.
  }
  if (message.equals("CW")) {
    CCW = false;
  }
  if (message.equals("CCW")) {
    CCW = true;
  }
  Serial.flush();                         // * clear any random data from the serial buffer

  if(changeFlag){
    changeFlag = false;
    Serial.println(encoderACount);
  }
//  motorStop = 132;
//  if(encoderACount > motorStop){
//    motorSpeed = 0;
//  }
  setMotor(motorSpeed, CCW);
}
/**
 * Reading analog temperature from thermistor
 */
float getTemperature() {
  // Average the samples
  float average;
  int i;
  // Take N samples and store in array
  for (i = 0; i < N; i++) {
    samples[i] = analogRead(THERMISTORPIN);
//    delay(10);
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

// read the encoder direction from the quadrature encoder encoder LS7184 chip
void encoderIntA(){
    if (digitalRead(encoder0dir) == HIGH)
    encoderACount++;
  else
    encoderACount--;
  changeFlag = true;
}

void setMotor(int motorSpeed, boolean CCW)
{
  analogWrite(motorPWM, motorSpeed);
  digitalWrite(logicPin1, !CCW);
  digitalWrite(logicPin2, CCW);
}

