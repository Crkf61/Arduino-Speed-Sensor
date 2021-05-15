// this is a test to check git stuff is working!
// test number 2
// testing 3
//

#include "fix_fft.h"

#define SAMPLES             128                    // must be power of 2
#define BUFFER_SIZE         20
#define SAMPLE_FREQ         4000.0
#define SPEED_OF_SOUND      343.0                 // m/s
#define SAMPLE_WAIT         1000000/SAMPLE_FREQ   // microseconds. This gives 2kHz sampling rate
#define INDS_TO_FREQS       SAMPLE_FREQ/SAMPLES

#define PEAK_NOISE_THRESH         3.0
#define PEAK_TO_EDGE_MAX          0.99
#define PEAK_TO_EDGE_MIN          0.01
#define FREQUENCY_SEPARATION_TOL  4 // value of 4 allows max and second frequencies to be 2 places away.
#define MAX_F_TOLERANCE           10.0 // maximum % tolerance to accept congruent frequency results in analysis
#define C_THRESH                  (MAX_F_TOLERANCE/100.0)*(MAX_F_TOLERANCE/100.0)

#define AUDIO   A0
#define RED     2
#define BLUE    3
#define GREEN   4
#define YELLOW  9
//#define BUTTON  5

char vReal[SAMPLES];
char vImag[SAMPLES];
float cycling_max_indices[BUFFER_SIZE]; // 3 seconds either side
float sum_mag_list[BUFFER_SIZE];
uint8_t buffer_pointer = 0;
float magnitude_noise_floor = 0.0;
float average_index;
float sum_magnitude;
float final_freq_1;
float final_freq_2;
float drone_speed;

void setup() {
  //Connect 3v3 supply to VREF pin
  //so that analog reads are mapped nicely
  Serial.begin(9600);
  analogReference(EXTERNAL);
  analogRead(AUDIO); // Make sure pin is stable

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  //pinMode(BUTTON, INPUT);

  Serial.println("Beginning fourier transform testing");
}

void loop() {
  // fill up buffer, find 'noise floor'
  digitalWrite(RED, HIGH);
  for(uint8_t a = 0; a < BUFFER_SIZE; a++)
  {
    fourier_transform_loop();
    Serial.println(a);
    if(sum_magnitude > magnitude_noise_floor)
    {
      magnitude_noise_floor = sum_magnitude;
    }
  }
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, HIGH);

  // enter sampling loop until drone passes
  bool threshold_noise = 0;
  bool peak_noise = 0;
  
  while(!peak_noise)
  {
    fourier_transform_loop();
    
    if(sum_magnitude > PEAK_NOISE_THRESH * magnitude_noise_floor)
    {
      threshold_noise = 1;
      digitalWrite(YELLOW, HIGH);
    }

    if(threshold_noise)
    {
      if(test_noise_level())
      {
        // stop sampling and move to calculating frequencies
        peak_noise = 1;
        digitalWrite(GREEN, HIGH);
        buffer_pointer = (buffer_pointer - 1) % BUFFER_SIZE; // negates update performed by fourier_transform_loop()
      }
    }
  }

  // time to extract the frequencies and calculate the doppler shift!
  if(evaluate_success())
  {
    digitalWrite(RED, LOW);
    digitalWrite(YELLOW, LOW);
    digitalWrite(BLUE, LOW);
    digitalWrite(GREEN, HIGH);
    drone_speed = SPEED_OF_SOUND*(final_freq_1 - final_freq_2)/(final_freq_1 + final_freq_2); // doppler shift
    printResults();
    while(1);
  }
}

void fourier_transform_loop(void) // Purpose: store highest freq and its magnitude, for short audio sample
{
  //Serial.println("Fourier transform loop");
  fill_arrays();
  //Serial.println("fill_arrays()");
  fix_fft(vReal, vImag, 8, 0);                          // log2(8) = 256, the number of samples used. Stores values in vReal and vImag
  //Serial.println("fix_fft(...)");
  find_mags_and_inds();                                  // find largest magnitude frequency. updates 'average_index' and 'sum_magnitude'
  //Serial.println("find mags and inds");
  cycling_max_indices[buffer_pointer] = average_index;  // store 'index' of largest freq
  //Serial.println("cycling max indices");
  sum_mag_list[buffer_pointer] = sum_magnitude;         // store magnitude of largest freq
  //Serial.println("sum mag list");
  buffer_pointer = (buffer_pointer + 1) % BUFFER_SIZE;
  //Serial.println("advance pointer");
  Serial.print("Freq: "); Serial.print(average_index*INDS_TO_FREQS); Serial.print(" Hz, Magnitude: "); Serial.println(sum_magnitude);
}

void fill_arrays(void)
{
  for(uint8_t j = 0; j < SAMPLES; j++)
  {
    unsigned long t_start = micros();
    vReal[j] = sample_audio();
    vImag[j] = 0;
    while(micros() - t_start < SAMPLE_WAIT);
  }
}

char sample_audio(void)
{
  uint16_t val = analogRead(AUDIO);
  char val2 = (char)(val/4 - 128);
  return val2;
}

void find_mags_and_inds(void)
{
  float max_magnitude = 0.0;
  float second_magnitude = 0.0;
  uint8_t max_index = 0;
  uint8_t second_index = 0;

  for (uint8_t i = 0; i < SAMPLES/2; i++)
  {
    float magnitude = sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);
    if(magnitude > max_magnitude)
    {
      max_magnitude = magnitude;
      max_index = i;
    }
    else if(magnitude > second_magnitude)
    {
      second_magnitude = magnitude;
      second_index = i;
    }
  }
  update_sum_mag_and_av_ind(max_magnitude, second_magnitude, max_index, second_index);
}

void update_sum_mag_and_av_ind(float maxm, float secm, uint8_t maxi, uint8_t seci)
{
  if((maxi - seci) * (maxi - seci) <= FREQUENCY_SEPARATION_TOL)
  {
    average_index = (float)((maxi * maxm + seci * secm)/(maxm + secm));
    sum_magnitude = (float)(maxm + secm);
  }
  else
  {
    average_index = (float)maxi;
    sum_magnitude = (float)maxm;
  }
}

bool evaluate_success(void)
{
  bool success = 1;
  
  float start_i_1 = cycling_max_indices[(buffer_pointer + 1) % BUFFER_SIZE];
  float start_i_2 = cycling_max_indices[(buffer_pointer + 2) % BUFFER_SIZE];
  float end_i_1 = cycling_max_indices[buffer_pointer];
  float end_i_2 = cycling_max_indices[(buffer_pointer - 1) % BUFFER_SIZE];

  float c_ind_1 = cong_indicator(start_i_1, start_i_2);
  float c_ind_2 = cong_indicator(end_i_1, end_i_2);

  if(c_ind_1 > C_THRESH || c_ind_2 > C_THRESH)
  {
    success = 0;
    Serial.println("\nAdjacent Frequencies not congruent");
    printArray(cycling_max_indices);
    Serial.print("\nbuffer pointer: ");
    Serial.println(buffer_pointer);
    
    for(uint8_t i = 0; i < 6; i++)
    {
      uint8_t j = (i+1) % 2;
      digitalWrite(RED, j);
      digitalWrite(GREEN, j);
      digitalWrite(YELLOW, j);
      digitalWrite(BLUE, j);
      delay(1000);
    }
  }
  else
  {
    final_freq_1 = INDS_TO_FREQS*(start_i_1 + start_i_2)/2.0;
    final_freq_2 = INDS_TO_FREQS*(end_i_1 + end_i_2)/2.0;
  }
  return success;
}

float cong_indicator(float index_1, float index_2) // check congruence is lower than (5) %
{
  float c_i;
  if(index_1*index_2 == 0) // don't want division by 0!
  {
    c_i = 1000.0;
  }
  else
  {
    c_i = (index_1 - index_2)*(index_1 - index_2)/(index_1 * index_2);
  }
  return c_i;
}

uint8_t test_noise_level()
{
  uint8_t returner = 1;
  double mid_val = sum_mag_list[BUFFER_SIZE/2];
  // check there is a peak at the middle index
  for(int i = 0; i < BUFFER_SIZE; i ++)
  {
    if(sum_mag_list[i] > mid_val)
    {
      returner = 0;
      digitalWrite(RED, LOW);
    }
  }
  // if peak present, check noise at start and finish is 1/2 magnitude of middle
  if(returner)
  {
    digitalWrite(RED, HIGH);
    double start_val = sum_mag_list[buffer_pointer];
    double end_val = sum_mag_list[(buffer_pointer - 1) % BUFFER_SIZE];
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

void printResults(void)
{
  Serial.println("\nDrone Speed: ");
  Serial.print(drone_speed);
  Serial.println(" ms-1");

  Serial.println("\nFrequencies used (Hz):");
  Serial.println(final_freq_1);
  Serial.println(final_freq_2);
}

void printArray(float arr1[BUFFER_SIZE])
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
