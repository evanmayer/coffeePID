// using Brett Beauregard's excellent PID library 
// with example code from 
// https://playground.arduino.cc/Code/PIDLibraryRelayOutputExample
// thermocouple reading libs
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <PID_v1.h>

// Create a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   0
#define MAXCS   1
#define MAXCLK  2

// Define variables for PID
double setPoint;
double input;
double output;

volatile long onTime = 0;

// kettle tuning params
//double kP = 78.;
//double kI = 0.92;
//double kD = 16.;

// gaggia tuning params
double kP = 78.;
double kI = 1.4;
double kD = 19.;

// Initialize a PID, with initial tuning params k_p, k_i, k_d
PID coffeePID(&input, &output, &setPoint, kP, kI, kD, DIRECT);

// a window size as a basis for relay "on time"
int windowSize = 2500; // ms
unsigned long windowStartTime;

// initialize the thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void setup() {
  //////////////////////////////////
  // pin setup                    //
  //////////////////////////////////
  // init GPIO 4 as output, and mirror with 13 for signaling ON
  // this will control the relay.
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  //////////////////////////////////
  // PID setup                    //
  //////////////////////////////////
  // ############################ //
  // setpoint                     //
  // ############################ //
  setPoint = 225.; 
    
  windowStartTime = millis();
  coffeePID.SetTunings(kP, kI, kD);
  //coffeePID.SetSampleTime(1000);
  coffeePID.SetOutputLimits(0, windowSize);
  coffeePID.SetMode(AUTOMATIC);
  
  //////////////////////////////////
  // serial setup                 //
  //////////////////////////////////
  Serial.begin(9600);
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc
  // wait for MAX chip to stabilize
  delay(500);

  //////////////////////////////////
  // timer interrupt, t = 15ms    //
  //////////////////////////////////
  // from
  // https://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino/the-whole-enchilada
  // Run timer2 interrupt every 15 ms 
  //TCCR2A = 0;
  //TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;
 
  //Timer2 Overflow Interrupt Enable
  //TIMSK2 |= 1<<TOIE2;
}

//////////////////////////////////
// timer interrupt handler      //
//////////////////////////////////
//SIGNAL(TIMER2_OVF_vect) {
//  DriveOutput();
//}

void loop() {
   // print temp in deg F to serial
   //Serial.print("");
   //Serial.print("Internal Temp = ");
   //Serial.println(thermocouple.readInternal());

   double temp = thermocouple.readFarenheit();
   if (isnan(temp)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     // update input var and compute PID 
     input = temp;
     coffeePID.Compute();
     
     onTime = output;
     DriveOutput();
     
     //Serial.print("C = ");
     //Serial.print("F = ");
     //Serial.print(output/10);
     //Serial.print(',');
     Serial.print(setPoint);
     Serial.print(',');
     Serial.println(temp);
   }
   //delay(500);
}

// Controls relay pin level and judges pulse width window
void DriveOutput() {
     // turn the LED/output pin on/off
     unsigned long now = millis();
     if ((now - windowStartTime) > windowSize) {
          // amt to shift relay's window time
         windowStartTime += windowSize;
     }
     
     if ((onTime > 100) && (onTime > (now - windowStartTime))) {
         digitalWrite(4, HIGH);
         digitalWrite(13, HIGH);
         //Serial.println("HI");
     } else {
         digitalWrite(4, LOW);
         digitalWrite(13, LOW);
         //Serial.println("LOW");
     }

}
