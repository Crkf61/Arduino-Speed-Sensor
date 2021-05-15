#include "fix_fft.h"

#define SAMPLES 256
#define SAMPLE_WAIT 500 // microseconds. This gives 2kHz sampling rate
#define INDS_TO_FREQS 7.8125 //31.25 //15.625  //7.8125  // 3.90625
#define AUDIO A0
#define BUFFER_SIZE 24

#define RED 2
#define BLUE 3
#define GREEN 4

char vReal[SAMPLES];
char vImag[SAMPLES];
// only need half of the values, as the other half are above nyquist limit
//uint16_t barht[SAMPLES/2]; // bar height

double max_magnitude = 0.0;
double second_magnitude = 0.0;
uint8_t max_index;
uint8_t second_index;
double average_index;
uint8_t buffer_pointer = 0;

double cycling_max_indices[BUFFER_SIZE]; // 3 seconds either side

double RMS_noise = 0.0; // used for determining when the drone has passed over

void setup() {
  //Connect 3v3 supply to VREF pin
  //so that analog reads are mapped nicely
  Serial.begin(9600);
  analogReference (EXTERNAL) ;
  analogRead (0) ; // Make sure pin is stable

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);

  Serial.println("Beginning fourier transform testing");
}

void loop() {
  // get audio data to fill up buffer
  uint16_t val;

  digitalWrite(RED, HIGH);
  for(int i = 0; i < BUFFER_SIZE; i++)
  {
    RMS_noise = 0.0;
    for(int j = 0; j < SAMPLES; j++)
    {
      unsigned long t_start = micros();
      uint16_t val = analogRead(AUDIO); //   0  - 1023
      char val2 = (char)(val/4 - 128);  // -128 - 1023
      vReal[j] = val2;
      vImag[j] = 0;
      RMS_noise = RMS_noise + val2 * val2;
      while(micros() - t_start < SAMPLE_WAIT);
    }
    RMS_noise = sqrt(RMS_noise/SAMPLES);
    //Serial.println("RMS noise: %.3f", RMS_noise);

    fix_fft(vReal, vImag, 8, 0); // log2(9) = 512, the number of samples used
    // now magnitudes of the frequencies are stored in vReal and vImag.

    // get absolute magnitudes of each frequency
    max_magnitude = 0.0;
    second_magnitude = 0.0;
    max_index = 0;
    second_index = 0;
    for(uint8_t i = 0; i < SAMPLES/2; i ++)
    {
      //barht[i] = (uint16_t)sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);
      double magnitude = sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);
      //Serial.println(magnitude);
      if(magnitude > max_magnitude)
      {
        // update maximum magnitude and index at which it occurs
        max_magnitude = magnitude;
        max_index = i;
        // potential to add something to interpolate between the two highest
        // frequencies, to get better resolution
      }
      else if(magnitude > second_magnitude)
      {
        second_magnitude = magnitude;
        second_index = i;
      }
    }
    if((max_index - second_index) * (max_index - second_index) == 1)
    {
      average_index = (double)((max_index * max_magnitude + second_index * second_magnitude)/(max_magnitude + second_magnitude));
    }
    else
    {
      average_index = (double)max_index;
    }
    cycling_max_indices[buffer_pointer] = average_index; // store 'index' of largest frequency component
    buffer_pointer = (buffer_pointer + 1) % BUFFER_SIZE;
  }
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, HIGH);

  Serial.print("last RMS noise value (max 128): ");
  Serial.println(RMS_noise);

  Serial.println("\nMax indices:");
  for(int i = 0; i < BUFFER_SIZE; i++)
  {
    Serial.println(cycling_max_indices[i]);
  }
  Serial.println("\n\nMax frequencies:");
  for(int i = 0; i < BUFFER_SIZE; i++)
  {
    Serial.println(cycling_max_indices[i]*INDS_TO_FREQS);
  }
  Serial.println("\n\nFinished debugging sequence. bye");
  while(1);

}



// FLOWCHART
/*
 Setup ADC
 Setup LCD display

 Enter sampling loop, to get a peak frequency
 Fill up 2~5 second circular buffer, RED led during process

 Continue to cycle buffer. BLUE led.
 And work out RMS noise at each time step.
 At peak noise, continue for 1~2.5 seconds. RED LED

 Finish calculations, calculate speed.
 Display speed on screen, GREEN LED

 Wait forever until reset button pressed.
 
 */
