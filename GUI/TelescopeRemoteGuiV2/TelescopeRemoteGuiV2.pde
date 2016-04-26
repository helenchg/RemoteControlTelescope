// Need G4P library
import g4p_controls.*;
import processing.serial.*;
import java.util.*;

Serial myPort; // Create object from Serial class
String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
Float ambTemp;    // Data received from the serial port
Float priTemp; 
Float secTemp; 
String message;
//String message2;
PFont f;


public void setup() {
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n'); 
  ambTemp = 0.0;
  priTemp = 0.0;
  secTemp = 0.0;
  f = createFont("Open Sans", 150, true);
  size(600, 500, JAVA2D);
  createGUI();
  customGUI();
  // Place your setup code here
}

public void draw() {
  background(230);
  customGUI();
  textFont(f, 150);
  fill(0, 0, 0);
  textSize(12);
  text("Developed by ECE Senior Design Team 17: \nElena Chong, Harlan Dupree, Toba Faseru, and Panpan Yuan", 50, 20);
  fill(0, 0, 0);
  text("April 2016", 500, 20);
  textSize(20);
  fill(200, 0, 0);
  text("TELESCOPE INTERFACE MODULE", 150, 80);
}
// Use this method to add additional statements
// to customise the GUI controls
public void customGUI() {
  aTemp.setText(ambTemp+ " °C");
  pTemp.setText(priTemp+ " °C");
  sTemp.setText(secTemp+ " °C");
}

void serialEvent (Serial s) {
  message = s.readStringUntil('\n');       //read serial data and store it in "message" variable.
  message = message.trim();                //cut out the white space
  println(message);                      // SOMETIMES GETTING GIBBERISH AT THE BEGINNING. NULL EXCEPTION!

  if (message.contains("a")) {           // check if the message contains the string "Temp"
    String temp = message;                  // store the message in variable temp 
    temp = temp.replace("a ", "");      // cut out the string "Temp: "
    ambTemp = parseFloat(temp);         // convert the string to float and store the data in temperature
  }
  if (message.contains("p")) {           // check if the message contains the string "Temp"
    String temp = message;                  // store the message in variable temp 
    temp = temp.replace("p ", "");      // cut out the string "Temp: "
    priTemp = parseFloat(temp);         // convert the string to float and store the data in temperature
  }
  if (message.contains("s")) {           // check if the message contains the string "Temp"
    String temp = message;                  // store the message in variable temp 
    temp = temp.replace("s ", "");      // cut out the string "Temp: "
    secTemp = parseFloat(temp);         // convert the string to float and store the data in temperature
  }
}

