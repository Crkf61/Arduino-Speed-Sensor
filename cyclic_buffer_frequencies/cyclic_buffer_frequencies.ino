#include "fix_fft.h"

#define SAMPLES 256
#define SAMPLE_FREQ 2000.0
#define SAMPLE_WAIT 1000000/SAMPLE_FREQ // microseconds. This gives 2kHz sampling rate
#define INDS_TO_FREQS SAMPLE_FREQ/SAMPLES //7.8125 //31.25 //15.625  //7.8125  // 3.90625
#define AUDIO A0
#define BUFFER_SIZE 40
#define PEAK_NOISE_THRESH 3.0
#define PEAK_TO_EDGE_MAX 0.99
#define PEAK_TO_EDGE_MIN 0.01

#define MAX_F_TOLERANCE 100.0 // maximum % tolerance to accept congruent frequency results in analysis

#define SPEED_OF_SOUND 343.0 // m/s

#define RED 2
#define BLUE 3
#define GREEN 4
#define YELLOW 9
#define BUTTON 5

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
double RMS_noise_floor;
double RMS_noise_list[BUFFER_SIZE];


void setup() {
  //Connect 3v3 supply to VREF pin
  //so that analog reads are mapped nicely
  Serial.begin(9600);
  analogReference (EXTERNAL) ;
  analogRead (0) ; // Make sure pin is stable

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(BUTTON, INPUT);

  Serial.println("Beginning fourier transform testing");
}

void loop() {
  // get audio data to fill up buffer
  digitalWrite(RED, HIGH);
  for(int i = 0; i < BUFFER_SIZE; i++)
  {
    RMS_noise = 0.0;
    for(int j = 0; j < SAMPLES; j++)
    {
      unsigned long t_start = micros();
      uint16_t val = analogRead(AUDIO); //   0  - 1023
      char val2 = (char)(val/4 - 128);  // -128 - 128
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
    Serial.print("Frequency: "); Serial.print(INDS_TO_FREQS*average_index);
    Serial.print(" Magnitude: "); Serial.println(max_magnitude);
    buffer_pointer = (buffer_pointer + 1) % BUFFER_SIZE;
    RMS_noise_floor = RMS_noise_floor + RMS_noise;
  }
  RMS_noise_floor = RMS_noise_floor/BUFFER_SIZE;
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, HIGH);
  /*
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
  */
  uint8_t peak_noise = 0; // used to indicate when to stop sampling
  uint8_t threshold_noise = 0; // used to trigger searching for peak
  
  while(!peak_noise)
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
      double magnitude = sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);
      if(magnitude > max_magnitude)
      {
        // update maximum magnitude and index at which it occurs
        max_magnitude = magnitude;
        max_index = i;
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
    if(digitalRead(BUTTON))
    {
      Serial.print("\nDebugging data:\nMagnitudes: ");
      Serial.println(max_magnitude); Serial.println(second_magnitude);
      Serial.print("Frequencies: ");
      Serial.println((double)max_index*INDS_TO_FREQS); Serial.println((double)second_index*INDS_TO_FREQS); Serial.println((double)average_index*INDS_TO_FREQS);
      Serial.print("RMS noise: ");
      Serial.println(RMS_noise);
    }
    cycling_max_indices[buffer_pointer] = average_index; // store 'index' of largest frequency component
    RMS_noise_list[buffer_pointer] = RMS_noise;
    
    if(RMS_noise > PEAK_NOISE_THRESH * RMS_noise_floor)
    {
      threshold_noise = 1;
      digitalWrite(YELLOW, HIGH);
    }

    if(threshold_noise)
    {
      if(test_noise_level(RMS_noise_list))
      {
        // stop sampling and move to calculating frequencies
        peak_noise = 1;
        digitalWrite(GREEN, HIGH);
        buffer_pointer = (buffer_pointer - 1) % BUFFER_SIZE; // negates following line if we're exiting the loop
      }
    }
    
    buffer_pointer = (buffer_pointer + 1) % BUFFER_SIZE;
  }

  // time to extract the frequencies and calculate the doppler shift!
  // we have a list of 24 frequencies to work with.
  // best accuracy for doppler is at start and end of list
  // however best signal to noise ratio is in the middle
  // and going too far to the edges could mean its not a constant frequency.
  // sooooo yeah gonna take the average of the 4th and 5th, then the 19th and 20th

  double start_i_1 = cycling_max_indices[(buffer_pointer + 4) % BUFFER_SIZE];
  double start_i_2 = cycling_max_indices[(buffer_pointer + 5) % BUFFER_SIZE];
  double end_i_1 = cycling_max_indices[(buffer_pointer - 3) % BUFFER_SIZE];
  double end_i_2 = cycling_max_indices[(buffer_pointer - 4) % BUFFER_SIZE];

  uint8_t successful = 1;

  double c_thresh = (MAX_F_TOLERANCE/100.0)*(MAX_F_TOLERANCE/100.0);
  double c_ind_1 = cong_indicator(start_i_1, start_i_2);
  double c_ind_2 = cong_indicator(end_i_1, end_i_2);

  if(c_ind_1 > c_thresh || c_ind_2 > c_thresh)
  {
    successful = 0;
  }

  if(!successful)
  {
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, HIGH);
    digitalWrite(YELLOW, HIGH);
    digitalWrite(BLUE, LOW);

    Serial.println("\nAdjacent Frequencies not congruent");
    printArray(cycling_max_indices);
    Serial.print("\nbuffer pointer: ");
    Serial.println(buffer_pointer);
    
    while(1);
  }

  // take averages, store in old variables. also convert to actual frequencies
  start_i_1 = INDS_TO_FREQS*(start_i_1 + start_i_2)/2.0;
  end_i_1 = INDS_TO_FREQS*(end_i_1 + end_i_2)/2.0;
  
  double drone_speed = SPEED_OF_SOUND*(start_i_1 - end_i_1)/(start_i_1 + end_i_1);

  Serial.print(drone_speed);
  Serial.println(" ms-1");

  Serial.println("\nFrequencies used:");
  Serial.println(start_i_1);
  Serial.println(end_i_1);

  Serial.println("\nCongruence indicator:");

  
  digitalWrite(BLUE, LOW);
  digitalWrite(GREEN, HIGH);
  while(1);
}

double cong_indicator(double index_1, double index_2) // check congruence is lower than (5) %
{
  double c_i = (index_1 - index_2)*(index_1 - index_2)/(index_1 * index_2);
  return c_i;
}

uint8_t test_noise_level(double rms_noise_list[BUFFER_SIZE])
{
  uint8_t returner = 1;
  double mid_val = rms_noise_list[BUFFER_SIZE/2];
  // check there is a peak at the middle index
  for(int i = 0; i < BUFFER_SIZE; i ++)
  {
    if(rms_noise_list[i] > mid_val)
    {
      returner = 0;
      if(i == BUFFER_SIZE/2) // Shouldn't happen, but would mess up program
      {
        digitalWrite(RED, HIGH);
        digitalWrite(GREEN, HIGH);
        digitalWrite(BLUE, HIGH);
        digitalWrite(YELLOW, HIGH);
        while(1);
      }
    }
  }
  // if peak present, check noise at start and finish is 1/2 magnitude of middle
  if(returner)
  {
    double start_val = rms_noise_list[(buffer_pointer + 1) % BUFFER_SIZE];
    double end_val = rms_noise_list[buffer_pointer];
    if(start_val > mid_val*PEAK_TO_EDGE_MAX || end_val > mid_val*PEAK_TO_EDGE_MAX)
    {
      returner = 0;
    }
    if(start_val < mid_val*PEAK_TO_EDGE_MIN || end_val < mid_val*PEAK_TO_EDGE_MIN)
    {
      returner = 0;
    }
  }
  return returner;
}

void printArray(double arr1[BUFFER_SIZE])
{
  for(int i=0; i < BUFFER_SIZE; i++)
  {
    Serial.println(arr1[i]);
  }
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
