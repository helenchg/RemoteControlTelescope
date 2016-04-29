/**
 * Author: Elena Chong
 * Date: 3/19/2016
 * Description:
 * Read data from 10k NTC thermistor
 * Its resistance decrements as temperature increases.
 * Read encoder position for DC gearmotor
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
const unsigned long baudrate = 115200;
#define encoder0PinA  21
#define encoder0PinB  20
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
//////////////// define variables /////////////////

//////////////// global variables /////////////////
int raw = 0;
int samples[N];
double analog;
float vout = 0;
float R2 = 0;
float buffer = 0;
float T;
String message;
volatile int encoder0Pos = 0;
boolean CCW = true;
int motorSpeed = 0;

void setup() {
  // This happens just once during the entire program
  Serial.begin(baudrate);
  pinMode(FANSPIN, OUTPUT);  // Set output pin for sending signal to the TIP31

  pinMode(motorPWM, OUTPUT);  // Set output pin for sending signal to motor speed
  pinMode(logicPin1, OUTPUT); // set output pin for sending signal to Hbridge direction
  pinMode(logicPin2, OUTPUT);// set output pin for sending signal to Hbridge direction

  pinMode(encoder0PinA, INPUT); // get motor position from encoder
  pinMode(encoder0PinB, INPUT); // get motor position from encoder

  attachInterrupt(2, doEncoderA, FALLING);// encoder pin on interrupt 0 (pin 21)
  attachInterrupt(3, doEncoderB, FALLING);// encoder pin on interrupt 1 (pin 20)
//  Serial.begin (9600); // set Baudrate
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
//  delay(1000);

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
  // TODO: get the value from the textfield to set the motorSpeed.
  // TODO: get the value for the motor position. Not implemented yet.
//   motorSpeed = 255;
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
    delay(10);
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

///**
// * Reading analog temperature from thermistor #2
// */
//float getTemperatureNom() {
//  // Average the samples
//  float averageNom;
//  int i;
//  // Take N samples and store in array
//  for (i = 0; i < N; i++) {
//    samples[i] = analogRead(A1);
//    delay(10);
//  }
//  // Average all the samples
//  averageNom = 0;
//  for (i = 0; i < N; i++) {
//    averageNom = averageNom + samples[i];
//  }
//  averageNom = averageNom / N;
//
//  if (averageNom) {
//    // convert the value to resistance
//    averageNom = (1023 / averageNom)  - 1;
//    averageNom = SERIESRESISTOR / averageNom;
//
//    // Calculate Temperature
//    float steinhart;
//    steinhart = averageNom / SERIESRESISTOR;     // (R/Ro)
//    steinhart = log(steinhart);                  // ln(R/Ro)
//    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
//    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
//    steinhart = 1.0 / steinhart;                 // Invert
//    steinhart -= 273.15;                         // convert to C
//    return steinhart;
//  }
//  return -1;
//}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
//  Serial.println (encoder0Pos);
//  delay(100);
  // use for debugging - remember to comment out
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}

void setMotor(int motorSpeed, boolean CCW)
{
  analogWrite(motorPWM, motorSpeed);
  digitalWrite(logicPin1, !CCW);
  digitalWrite(logicPin2, CCW);
}

