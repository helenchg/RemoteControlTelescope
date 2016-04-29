// Need G4P library
import g4p_controls.*;
import processing.serial.*;
import java.util.*;

// WORK WITH SeniorDesignV4 and SeniorDesignV5

Serial myPort; // Create object from Serial class
String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
Float temperature;    // Data received from the serial port
String message;
//String message2;
PFont f;


public void setup() {
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n'); 
  temperature = 0.0;
  f = createFont("Open Sans", 150, true);
  size(400, 300, JAVA2D);
  createGUI();
  customGUI();
  // Place your setup code here
}

public void draw() {
  background(230);
  textFont(f, 150);
  textSize(14);
  fill(0);
  text("Ambient Temp: " + temperature + " °C", 220, 270 );
  fill(0,0,0);
  textSize(12);
  text("Developed by ECE Senior Design Team 17: \nElena Chong, Harlan Dupree, Toba Faseru, and Panpan Yuan", 10, 20);
  fill(0,0,0);
  text("April 2016", 330, 20);
}

// Use this method to add additional statements
// to customise the GUI controls
public void customGUI() {

}

void serialEvent (Serial s) {
  message = s.readStringUntil('\n');       //read serial data and store it in "message" variable.
  message = message.trim();                //cut out the white space
  println(message);                      // SOMETIMES GETTING GIBBERISH AT THE BEGINNING. NULL EXCEPTION!

  if (message.contains("T")) {           // check if the message contains the string "Temp"
    String temp = message;                  // store the message in variable temp 
    temp = temp.replace("T ", "");      // cut out the string "Temp: "
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

