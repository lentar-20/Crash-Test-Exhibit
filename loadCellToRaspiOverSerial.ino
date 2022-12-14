
// include the library code:
#include <HX711_ADC.h> // https://github.co1m/olkal/HX711_ADC
#include <Wire.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
float maxForce;
long resetTime; 
HX711_ADC LoadCell(8, 9); // parameters: dt pin, sck pin

void setup() {
  // set up the LCD's number of columns and rows:
  LoadCell.begin(); // start connection to HX711
  LoadCell.start(2000); // load cells gets 2000ms of time to stabilize
  LoadCell.setCalFactor(999.0); // calibration factor for load cell => strongly dependent on your individual setup
  Serial.begin(9600);
}

void loop() {
  //reset condition based on byte raspi is sending
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if(data.equals("T")){
      maxForce = 0;
    }
  }
  LoadCell.update(); // retrieves data from the load cell
  float i = LoadCell.getData(); // get output value
  float val = max(0,i*100/10);
  maxForce = int(max(val, maxForce));
  Serial.write((byte)maxForce); //send value as a byte over serial back to raspi
  
}
