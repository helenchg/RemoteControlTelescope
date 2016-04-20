// Need G4P library
import g4p_controls.*;
import processing.serial.*;
import java.util.*;

Serial myPort; // Create object from Serial class
String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
Float temperature;    // Data received from the serial port
String message;
//String message2;
PFont f;


public void setup() {
  myPort = new Serial(this, portName, 9600);
  myPort.bufferUntil('\n'); 
  temperature = 0.0;
  f = createFont("Open Sans", 150, true);
  size(480, 320, JAVA2D);
  createGUI();
  customGUI();
  // Place your setup code here
}

public void draw() {
  background(230);
  textFont(f, 150);
  textSize(20);
  fill(0);
  text("Ambient Temperature: " + temperature + " °C", 50, 50 );
}

// Use this method to add additional statements
// to customise the GUI controls
public void customGUI() {
}

void serialEvent (Serial s) {
  message = s.readStringUntil('\n');       //read serial data and store it in "message" variable.
  message = message.trim();                //cut out the white space
  println(message);                      // SOMETIMES GETTING GIBBERISH AT THE BEGINNING. NULL EXCEPTION!

  if (message.contains("Temp")) {           // check if the message contains the string "Temp"
    String temp = message;                  // store the message in variable temp 
    temp = temp.replace("Temp: ", "");      // cut out the string "Temp: "
    temperature = parseFloat(temp);         // convert the string to float and store the data in temperature
  }
  
//    message = s.readStringUntil('\n');       //read serial data and store it in "message" variable.
//  message = message.trim();                //cut out the white space
//  println(message);                      // SOMETIMES GETTING GIBBERISH AT THE BEGINNING. NULL EXCEPTION!
//
//  if (message.contains("Temp1")) {           // check if the message contains the string "Temp"
//    String temp = message;                  // store the message in variable temp 
//    temp = temp.replace("Temp1: ", "");      // cut out the string "Temp: "
//    temperature = parseFloat(temp);         // convert the string to float and store the data in temperature
//  }
}

