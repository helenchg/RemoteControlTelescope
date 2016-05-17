import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import g4p_controls.*; 
import processing.serial.*; 
import java.util.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class TelescopeRemoteGuiV3 extends PApplet {

// Need G4P library




Serial myPort; // Create object from Serial class
String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
Float ambTemp;    // Data received from the serial port
Float priTemp; 
Float secTemp; 
String message;
//String message2;
PFont f;
int arCurrentStep;

public void setup() {

  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n'); 
  ambTemp = 0.0f;
  priTemp = 0.0f;
  secTemp = 0.0f;
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
  aTemp.setText(ambTemp+ " \u00b0C");
  pTemp.setText(priTemp+ " \u00b0C");
  sTemp.setText(secTemp+ " \u00b0C");
  label_currentStep.setText(arCurrentStep + "");
}

public void serialEvent (Serial s) {
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
  if( message.contains("x")){
    String step = message;
    step = step.replace("x ", "");
    arCurrentStep = Integer.parseInt(step);
  }
}

/* =========================================================
 * ====                   WARNING                        ===
 * =========================================================
 * The code in this tab has been generated from the GUI form
 * designer and care should be taken when editing this file.
 * Only add/edit code inside the event handlers i.e. only
 * use lines between the matching comment tags. e.g.

 void myBtnEvents(GButton button) { //_CODE_:button1:12356:
     // It is safe to enter your event code here  
 } //_CODE_:button1:12356:
 
 * Do not rename this tab!
 * =========================================================
 */

public void panel1_Click1(GPanel source, GEvent event) { //_CODE_:tempPanel:710606:
  //  println("panel1 - GPanel >> GEvent." + event + " @ " + millis());
} //_CODE_:tempPanel:710606:

public void panel2_Click1(GPanel source, GEvent event) { //_CODE_:fansPanel:801525:
  //  println("panel2 - GPanel >> GEvent." + event + " @ " + millis());
} //_CODE_:fansPanel:801525:

public void slider1_change1(GSlider source, GEvent event) { //_CODE_:slider1:500067:
  //  println("slider1 - GSlider >> GEvent." + event + " @ " + millis());
  int sliderValue = slider1.getValueI();
  fansPWM.setText(sliderValue + "%");
} //_CODE_:slider1:500067:

public void button1_click1(GButton source, GEvent event) { //_CODE_:button1:285800:
  //  println("button1 - GButton >> GEvent." + event + " @ " + millis());
  println("Fans: ON");
  myPort.write("FN");
} //_CODE_:button1:285800:

public void button2_click1(GButton source, GEvent event) { //_CODE_:button2:394538:
  //  println("button2 - GButton >> GEvent." + event + " @ " + millis());
  println("Fans: OFF");
  myPort.write("FF");
} //_CODE_:button2:394538:

public void button8_click1(GButton source, GEvent event) { //_CODE_:button8:386555:
  //  println("button8 - GButton >> GEvent." + event + " @ " + millis());
  int fanpower = slider1.getValueI();
  myPort.write("p " + fanpower);
} //_CODE_:button8:386555:

public void panel4_Click1(GPanel source, GEvent event) { //_CODE_:motorPanel:823926:
  //  println("panel4 - GPanel >> GEvent." + event + " @ " + millis());
} //_CODE_:motorPanel:823926:

public void button3_click1(GButton source, GEvent event) { //_CODE_:button3:779259:
  //  println("button3 - GButton >> GEvent." + event + " @ " + millis());
  println("MOTOR OFF");
  myPort.write("MF");
} //_CODE_:button3:779259:

public void button4_click1(GButton source, GEvent event) { //_CODE_:button4:775226:
  //  println("button4 - GButton >> GEvent." + event + " @ " + millis());
  println("MOTOR ON");
  myPort.write("MN");
} //_CODE_:button4:775226:

public void textfield1_change1(GTextField source, GEvent event) { //_CODE_:textfield1:720791:
  //  println("textfield1 - GTextField >> GEvent." + event + " @ " + millis());
} //_CODE_:textfield1:720791:

public void button7_click1(GButton source, GEvent event) { //_CODE_:button7:636961:
  //  println("button7 - GButton >> GEvent." + event + " @ " + millis());
  String toSend;
  toSend = textfield1.getText();
  int toSendValue = Integer.parseInt(toSend);
  if (toSendValue > 30000 || toSendValue < -30000) {
    javax.swing.JOptionPane.showMessageDialog(null, "input steps between -30000 and 30000");
  } else {
    myPort.write("l " + toSend);
    label_targetStep.setText(toSend);
  }
} //_CODE_:button7:636961:

public void button9_click1(GButton source, GEvent event) { //_CODE_:button_home:632843:
  //  println("button9 - GButton >> GEvent." + event + " @ " + millis());
  String toSend = "1";
  myPort.write("l " + toSend);
  label_targetStep.setText(toSend);
} //_CODE_:button_home:632843:

public void panel5_Click1(GPanel source, GEvent event) { //_CODE_:heaterPanel:307794:
  //  println("panel5 - GPanel >> GEvent." + event + " @ " + millis());
} //_CODE_:heaterPanel:307794:

public void button5_click1(GButton source, GEvent event) { //_CODE_:button5:465117:
  //  println("button5 - GButton >> GEvent." + event + " @ " + millis());
  println("HEATER OFF");
  myPort.write("HF");
} //_CODE_:button5:465117:

public void button6_click1(GButton source, GEvent event) { //_CODE_:button6:614468:
  //  println("button6 - GButton >> GEvent." + event + " @ " + millis());
  println("HEATER ON");
  myPort.write("HN");
} //_CODE_:button6:614468:



// Create all the GUI controls. 
// autogenerated do not edit
public void createGUI(){
  G4P.messagesEnabled(false);
  G4P.setGlobalColorScheme(GCScheme.BLUE_SCHEME);
  G4P.setCursor(ARROW);
  if(frame != null)
    frame.setTitle("Sketch Window");
  tempPanel = new GPanel(this, 360, 100, 200, 125, "  TEMPERATURE");
  tempPanel.setCollapsible(false);
  tempPanel.setDraggable(false);
  tempPanel.setText("  TEMPERATURE");
  tempPanel.setTextBold();
  tempPanel.setLocalColorScheme(GCScheme.CYAN_SCHEME);
  tempPanel.setOpaque(true);
  tempPanel.addEventHandler(this, "panel1_Click1");
  label1 = new GLabel(this, 10, 90, 80, 20);
  label1.setText("Secondary:");
  label1.setOpaque(false);
  label2 = new GLabel(this, 10, 60, 80, 20);
  label2.setText("Primary:");
  label2.setOpaque(false);
  label3 = new GLabel(this, 10, 30, 80, 20);
  label3.setText("Ambient:");
  label3.setOpaque(false);
  pTemp = new GLabel(this, 100, 60, 80, 20);
  pTemp.setText("P Temp");
  pTemp.setOpaque(false);
  sTemp = new GLabel(this, 100, 90, 80, 20);
  sTemp.setText("S Temp");
  sTemp.setOpaque(false);
  aTemp = new GLabel(this, 100, 30, 80, 20);
  aTemp.setText("A Temp");
  aTemp.setOpaque(false);
  tempPanel.addControl(label1);
  tempPanel.addControl(label2);
  tempPanel.addControl(label3);
  tempPanel.addControl(pTemp);
  tempPanel.addControl(sTemp);
  tempPanel.addControl(aTemp);
  fansPanel = new GPanel(this, 360, 230, 200, 250, "  FANS");
  fansPanel.setCollapsible(false);
  fansPanel.setDraggable(false);
  fansPanel.setText("  FANS");
  fansPanel.setTextBold();
  fansPanel.setLocalColorScheme(GCScheme.CYAN_SCHEME);
  fansPanel.setOpaque(true);
  fansPanel.addEventHandler(this, "panel2_Click1");
  slider1 = new GSlider(this, 75, 50, 175, 30, 20.0f);
  slider1.setShowValue(true);
  slider1.setShowLimits(true);
  slider1.setRotation(PI/2, GControlMode.CORNER);
  slider1.setLimits(0, 0, 100);
  slider1.setShowTicks(true);
  slider1.setNumberFormat(G4P.INTEGER, 0);
  slider1.setLocalColorScheme(GCScheme.RED_SCHEME);
  slider1.setOpaque(false);
  slider1.addEventHandler(this, "slider1_change1");
  fansPWM = new GLabel(this, 23, 225, 80, 20);
  fansPWM.setText("0%");
  fansPWM.setOpaque(false);
  button1 = new GButton(this, 110, 30, 80, 30);
  button1.setText("FANS ON");
  button1.setLocalColorScheme(GCScheme.CYAN_SCHEME);
  button1.addEventHandler(this, "button1_click1");
  button2 = new GButton(this, 110, 75, 80, 30);
  button2.setText("FANS OFF");
  button2.setLocalColorScheme(GCScheme.CYAN_SCHEME);
  button2.addEventHandler(this, "button2_click1");
  label4 = new GLabel(this, 0, 50, 50, 20);
  label4.setText("0%");
  label4.setOpaque(false);
  label5 = new GLabel(this, 0, 205, 50, 20);
  label5.setText("100%");
  label5.setOpaque(false);
  label6 = new GLabel(this, 10, 25, 80, 20);
  label6.setText("FANS SPEED");
  label6.setOpaque(false);
  button8 = new GButton(this, 110, 120, 80, 30);
  button8.setText("SET SPEED");
  button8.setLocalColorScheme(GCScheme.RED_SCHEME);
  button8.addEventHandler(this, "button8_click1");
  fansPanel.addControl(slider1);
  fansPanel.addControl(fansPWM);
  fansPanel.addControl(button1);
  fansPanel.addControl(button2);
  fansPanel.addControl(label4);
  fansPanel.addControl(label5);
  fansPanel.addControl(label6);
  fansPanel.addControl(button8);
  motorPanel = new GPanel(this, 40, 100, 300, 210, "  MOTOR");
  motorPanel.setCollapsible(false);
  motorPanel.setDraggable(false);
  motorPanel.setText("  MOTOR");
  motorPanel.setTextBold();
  motorPanel.setOpaque(true);
  motorPanel.addEventHandler(this, "panel4_Click1");
  button3 = new GButton(this, 170, 170, 80, 30);
  button3.setText("MOTOR OFF");
  button3.addEventHandler(this, "button3_click1");
  button4 = new GButton(this, 50, 170, 80, 30);
  button4.setText("MOTOR ON");
  button4.addEventHandler(this, "button4_click1");
  label8 = new GLabel(this, 10, 80, 80, 20);
  label8.setText("Step");
  label8.setOpaque(false);
  textfield1 = new GTextField(this, 90, 80, 100, 30, G4P.SCROLLBARS_NONE);
  textfield1.setOpaque(true);
  textfield1.addEventHandler(this, "textfield1_change1");
  button7 = new GButton(this, 205, 80, 80, 30);
  button7.setText("SET VALUE");
  button7.setLocalColorScheme(GCScheme.ORANGE_SCHEME);
  button7.addEventHandler(this, "button7_click1");
  targetStep = new GLabel(this, 10, 115, 80, 20);
  targetStep.setText("Target Step:");
  targetStep.setOpaque(false);
  currentStep = new GLabel(this, 10, 140, 80, 20);
  currentStep.setText("Current Step:");
  currentStep.setOpaque(false);
  label_targetStep = new GLabel(this, 100, 115, 80, 20);
  label_targetStep.setText("0");
  label_targetStep.setOpaque(false);
  label_currentStep = new GLabel(this, 100, 140, 80, 20);
  label_currentStep.setText("0");
  label_currentStep.setOpaque(false);
  button_home = new GButton(this, 205, 120, 80, 30);
  button_home.setText("HOME");
  button_home.setLocalColorScheme(GCScheme.ORANGE_SCHEME);
  button_home.addEventHandler(this, "button9_click1");
  motorPanel.addControl(button3);
  motorPanel.addControl(button4);
  motorPanel.addControl(label8);
  motorPanel.addControl(textfield1);
  motorPanel.addControl(button7);
  motorPanel.addControl(targetStep);
  motorPanel.addControl(currentStep);
  motorPanel.addControl(label_targetStep);
  motorPanel.addControl(label_currentStep);
  motorPanel.addControl(button_home);
  heaterPanel = new GPanel(this, 40, 320, 300, 160, "  HEATER");
  heaterPanel.setCollapsible(false);
  heaterPanel.setDraggable(false);
  heaterPanel.setText("  HEATER");
  heaterPanel.setTextBold();
  heaterPanel.setLocalColorScheme(GCScheme.PURPLE_SCHEME);
  heaterPanel.setOpaque(true);
  heaterPanel.addEventHandler(this, "panel5_Click1");
  button5 = new GButton(this, 170, 80, 80, 30);
  button5.setText("HEATER OFF");
  button5.setLocalColorScheme(GCScheme.PURPLE_SCHEME);
  button5.addEventHandler(this, "button5_click1");
  button6 = new GButton(this, 50, 80, 80, 30);
  button6.setText("HEATER ON");
  button6.setLocalColorScheme(GCScheme.PURPLE_SCHEME);
  button6.addEventHandler(this, "button6_click1");
  label7 = new GLabel(this, 90, 30, 120, 40);
  label7.setText("Heater may not be present in system");
  label7.setOpaque(false);
  heaterPanel.addControl(button5);
  heaterPanel.addControl(button6);
  heaterPanel.addControl(label7);
}

// Variable declarations 
// autogenerated do not edit
GPanel tempPanel; 
GLabel label1; 
GLabel label2; 
GLabel label3; 
GLabel pTemp; 
GLabel sTemp; 
GLabel aTemp; 
GPanel fansPanel; 
GSlider slider1; 
GLabel fansPWM; 
GButton button1; 
GButton button2; 
GLabel label4; 
GLabel label5; 
GLabel label6; 
GButton button8; 
GPanel motorPanel; 
GButton button3; 
GButton button4; 
GLabel label8; 
GTextField textfield1; 
GButton button7; 
GLabel targetStep; 
GLabel currentStep; 
GLabel label_targetStep; 
GLabel label_currentStep; 
GButton button_home; 
GPanel heaterPanel; 
GButton button5; 
GButton button6; 
GLabel label7; 

  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "TelescopeRemoteGuiV3" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}