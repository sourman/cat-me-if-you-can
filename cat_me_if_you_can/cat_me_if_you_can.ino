/*
  Analog input, analog output, serial output

 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground

 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

 #include <CurieNeuronsPro.h>
 CurieNeurons brain; /* Create a neural network object */

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to
long int micro;
long int mili;
int mic = 0;        // value read from the microphone
int outputValue = 0;        // value output to the PWM (analog out)


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  while (!Serial)    // wait for the serial port to open
      ;
  // Initialize the neurons and set a conservative Max Influence Field
  if (brain.begin()==0) Serial.print("\nNeural network is initialized!");
  else Serial.print("\nNeural network is NOT properly connected!");
  brain.forget(500); //set a conservative  Max Influence Field prior to learning
}

bool silent;

void loop() {
  byte vector[128]; // Maximum number of bytes a Neuron can process
  
  if (mili + 1000 <= millis()) {
      mili = millis();
      silent = !silent;
  }
  
  // read the analog in value:
  mic = analogRead(analogInPin);

  
  // map it to the range of the analog out:
  outputValue = map(mic, 0, 1023, 0, 255);
  // change the analog out value:
  if (silent)
    analogWrite(analogOutPin, 0);
  else
    analogWrite(analogOutPin, outputValue);

  // print the results to the serial monitor:
  Serial.print("mic = ");
  Serial.println(mic);
//  Serial.print("\t output = ");
//  Serial.println(outputValue);


  delayMicroseconds(125);
}



#if 0


#include "CurieIMU.h"

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

#include <CurieNeuronsPro.h>
CurieNeurons hNN;

int catL=0; // category to learn
int prevcat=0; // previously recognized category
int dist, cat, nid, nsr, ncount; // response from the neurons

//
// Variables used for the calculation of the feature vector
//
#define sampleNbr 20  // number of samples to assemble a vector
#define signalNbr  6  // ax,ay,az,gx,gy,gz
int raw_vector[sampleNbr*signalNbr]; // vector accumulating the raw sensor data
byte vector[sampleNbr*signalNbr]; // vector holding the pattern to learn or recognize
int mina=0xFFFF, maxa=0, ming=0xFFFF, maxg=0, da=0, dg=0; // reset, or not, at each feature extraction

void setup() 
{
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) 
  {    
    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");
    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    CurieIMU.setAccelerometerRange(8);
    CurieIMU.setGyroRange(1000);
  }
  // Initialize the neurons and set a conservative Max Influence Field
  if (hNN.begin()==0) Serial.print("\nNeural network is initialized!");
  else Serial.print("\nNeural network is NOT properly connected!");
  hNN.forget(500); //set a conservative  Max Influence Field prior to learning

  Serial.print("\n\nEntering loop...");
  Serial.print("\nMove the module vertically or horizontally...");
  Serial.print("\ntype 1 + Enter if vertical motion");
  Serial.print("\ntype 2 + Enter if horizontal motion");
  Serial.print("\ntype 0 + Enter for any other motion");
}

void loop() 
{   
    // WARNING: make sure the serial printer settings is "new line"

    // learn upon keyboard entry of 1 digit between 0-2
    // otherwise continuously report any change of motion
    if (Serial.available() == 2) 
    {
      catL = Serial.read();
      Serial.read(); // to empty serial buffer of the newline char
      catL = catL - 48;
      if (catL<3) //expected category input (1-vertical, 2-horizontal, 0-still)
      {
        Serial.print("\nLearning motion category "); Serial.print(catL);
        //
        // learn 5 consecutive sample vectors
        // (make sure to keep moving the 101 board accordingly)
        //
        for (int i=0; i<5; i++)
        {
          extractFeatureVector(); // the vector array is a global
          //Optional display of the vector in the serial monitor
          //Serial.print("\nVector = ");
          //for (int i=0; i<signalNbr*sampleNbr; i++) {Serial.print(vector[i]);Serial.print("\t");}
          ncount=hNN.learn(vector, sampleNbr*signalNbr, catL);
        }
        Serial.print("\tNeurons="); Serial.print(ncount);
      }    
    }
    else
    {
      // Recognize
      extractFeatureVector(); // the vector array is a global
      hNN.classify(vector, sampleNbr*signalNbr,&dist, &cat, &nid);
      if (cat!=prevcat)
      {
        if (cat!=0x7FFF)
        {
          Serial.print("\nMotion category #"); Serial.print(cat);
        }
        else Serial.print("\nMotion unknown");      
        prevcat=cat;
      }
    }
}  

//  The following function is very academic and assemble a pattern which
//  should be more sophisticated for real-life system taking a calibration into account,
//  integrating a sampling rate adequate for the type of motion and profiling the waveforms
//  more selectively (distances between peaks and zero crossing, etc)

void extractFeatureVector()
{
  // sensor output [ax,ay,az,gx, gy,gz] is converted into a byte array as follows:
  // [ax'1, ay'1, az'1, gx'1,gy'1, gz'1, ax'2, ay'2, az'2, gx'2, gy'2, gz'2, ...] over a number of time samples.
  // a' and g' are normalized using their respective min and max values.
  //
  // the reset of the min and max values is optional depending if you want to
  // use a running min and max from the launch of the script or not
  mina=0xFFFF, maxa=0, ming=0xFFFF, maxg=0, da=0, dg=0;
  
  for (int sampleId=0; sampleId<sampleNbr; sampleId++)
  {
    //Build the vector over sampleNbr and broadcast to the neurons
    CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
    
    // update the running min/max for the a signals
    if (ax>maxa) maxa=ax; else if (ax<mina) mina=ax;
    if (ay>maxa) maxa=ay; else if (ay<mina) mina=ay;
    if (az>maxa) maxa=az; else if (az<mina) mina=az;    
    da= maxa-mina;
    
    // update the running min/max for the g signals
    if (gx>maxg) maxg=gx; else if (gx<ming) ming=gx;
    if (gy>maxg) maxg=gy; else if (gy<ming) ming=gy;
    if (gz>maxg) maxg=gz; else if (gz<ming) ming=gz;   
    dg= maxg-ming;

    // accumulate the sensor data
    raw_vector[(sampleId*signalNbr)+1]= ay;
    raw_vector[(sampleId*signalNbr)+2]= az;
    raw_vector[(sampleId*signalNbr)+3]= gx;
    raw_vector[(sampleId*signalNbr)+4]= gy;
    raw_vector[(sampleId*signalNbr)+5]= gz;
  }
  
  // normalize vector
  for(int sampleId=0; sampleId < sampleNbr; sampleId++)
  {
    for(int i=0; i<3; i++)
    {
      vector[sampleId*signalNbr+i]  = (((raw_vector[sampleId*signalNbr+i] - mina) * 255)/da) & 0x00FF;
      vector[sampleId*signalNbr+3+i]  = (((raw_vector[sampleId*signalNbr+3+i] - ming) * 255)/dg) & 0x00FF;
    }
  }
}

#endif









#if 0







/*
PlainFFT Library: example of use
Input vectors receive computed results from FFT
No warranty, no claims, just fun
Didier Longueville invenit et fecit October 2010
*/
#include "PlainFFT.h"

PlainFFT FFT = PlainFFT(); // Create FFT object
// These values can be changed in order to evaluate the functions
const uint16_t samples = 64;
double signalFrequency = 4000;
double samplingFrequency = 8000;
uint8_t signalIntensity = 100;
// These are input and output vectors
double vReal[samples]; 
double vImag[samples];
uint8_t runOnce = 0x00;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

void setup(){
	Serial.begin(115200);
	Serial.println("Ready");
  delay(5000);
}

void loop() {
	if (runOnce == 0x00) {
		runOnce = 0x01;
		// Build raw data
		double cycles = (((samples-1) * signalFrequency) / samplingFrequency);
		for (uint8_t i = 0; i < samples; i++) {
			vReal[i] = uint8_t((signalIntensity * (sin((i * (6.2831 * cycles)) / samples) + 1.0)) / 2.0);
		}
		printVector(vReal, samples, SCL_TIME);
		FFT.windowing(vReal, samples);	// Weigh data
		printVector(vReal, samples, SCL_TIME);
		FFT.compute(vReal, vImag, samples, FFT_FORWARD); // Compute FFT
		printVector(vReal, samples, SCL_INDEX);
		printVector(vImag, samples, SCL_INDEX);
		FFT.complexToMagnitude(vReal, vImag, samples); // Compute magnitudes
		printVector(vReal, (samples >> 1), SCL_FREQUENCY);	
		double x = FFT.majorPeak(vReal, samples, samplingFrequency);
		Serial.println(x, 6);
	}
}

void printVector(double *vD, uint8_t n, uint8_t scaleType) {
	double timeInterval = (1.0 / samplingFrequency);
	for (uint16_t i = 0; i < n; i++) {
		// Print abscissa value
		switch (scaleType) {
		case SCL_INDEX:
			Serial.print(i, DEC);
			break;
		case SCL_TIME:
			Serial.print((i * timeInterval), 6);
			break;
		case SCL_FREQUENCY:
			Serial.print((i / (timeInterval * (samples-1))), 6);
			break;
		}
		Serial.print(" ");
		// Print ordinate value
		Serial.print(vD[i], 6);
		Serial.println();
	}
	Serial.println();
}

#endif
