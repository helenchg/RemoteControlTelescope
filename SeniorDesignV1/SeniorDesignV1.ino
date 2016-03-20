/**
 * Author: Elena Chong
 * Date: 3/19/2016
 * Description:
 * Read data from 10k NTC thermistor
 * Its resistance decrements as temperature increases.
 */

//////////////// static variables /////////////////
const unsigned long baudrate = 9600;
#define SERIESRESISTOR 10000 // the value of the resistor used for voltage divider
#define THERMISTORPIN A0 // thermistor analog pin
#define THERMISTORNOMINAL 10000 // thermistor resistance at 25 degrees C
#define TEMPERATURENOMINAL 25 // temp. for nominal resistance (almost always room temperature 25 C)
#define BCOEFFICIENT 3950 // Beta Coefficient from thermistor datasheet
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

void setup() {
  Serial.begin(baudrate);

}

void loop() {

  float temp = getTemperature();

  if (temp == -1) {
    Serial.println("Error reading temperature!");
    return;
  }
  Serial.print("Ambient Temperature: ");
  Serial.print("\t");
  Serial.println(temp);
  delay(1000);
}

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
