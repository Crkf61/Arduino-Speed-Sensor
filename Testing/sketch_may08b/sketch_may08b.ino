#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();

const uint16_t samples = 128; // must be a power of 2 (8192 = 2^14)
const uint8_t RMS_noise_samples = 25;
const double samplingFrequency = 8900;


#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

int sensorPin = A0;
double vReal[samples];
double vImag[samples];
int dataRMS[RMS_noise_samples];

int redled = 2;
int blueled = 3;
int greenled = 4;

int start_button = 6;

int quiet_value = 338;
int done=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(redled, OUTPUT);
  pinMode(blueled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(start_button, INPUT);

  // Setup LCD display

  //digitalWrite(blueled, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(greenled, HIGH);

  if(start_button && !done)
  {
    digitalWrite(blueled, HIGH);
    digitalWrite(greenled, LOW);
    for(int i=0; i<samples; i++)
    {
      int sensorVal = analogRead(sensorPin);
      vReal[i] = sensorVal - 355;
      vImag[i] = 0.0;
    }
    done = 1;
    digitalWrite(blueled, LOW);
    digitalWrite(redled, HIGH);
  }

  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  digitalWrite(greenled, HIGH);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  Serial.println("here are the magnitudes");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);

  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  Serial.println(x, 6);
  digitalWrite(redled, LOW);
  while(1);
  
  // display 'cycling through first array' on LCD
  
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
