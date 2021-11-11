#include <fix_fft.h>

#include "fourier.hpp"

char vReal[SAMPLES];
char vImag[SAMPLES];

float samples_on_approach[SAMPLES_IN_CALCULATION];
float samples_on_retreat[SAMPLES_IN_CALCULATION];

float quantised_value;
float sum_magnitude;

void fourier_transform_loop() {
  // threshold magnitude for recording
  // TODO: hardcoded for now would like to update dynamically
  //   may not be possible on an arduino without good data analysis
  float min_magnitude = 10.0;

  static int circle_ptr = 0;

  static State state = approach;
  static int loud_volume_counter = 0;

  fill_arrays();
  fix_fft(vReal, vImag, LOG_SAMPLES, 0);
  find_mags_and_inds();

  // Update arrays with latest sample values
  if (state == approach) {
    samples_on_approach[circle_ptr] = quantised_value;
  }
  samples_on_retreat[circle_ptr] = quantised_value;
  circle_ptr = (circle_ptr + 1) % SAMPLES_IN_CALCULATION;

  // Increment counter if drone is heard
  if (sum_magnitude > min_magnitude) {
    loud_volume_counter++;
  } else {
    loud_volume_counter = 0;
  }

  if (state == passed) {
    // Output the doppler shift calculation
    Serial.println("DRONE PASSED! HERE ARE THE RESULTS");
    for (int i = 0; i < SAMPLES_IN_CALCULATION; i++) {
      Serial.print(samples_on_approach[i] * INDS_TO_FREQS);
      Serial.print(", ");
      Serial.println(samples_on_retreat[i] * INDS_TO_FREQS);
    }
    Serial.println(calculate_doppler_shift());
  }

  // Output to serial for debugging and data logging.
  //Serial.println("routine checks");
  Serial.print(state == approach? "approach" : (state == retreat? " retreat" : "  passed"));  
  Serial.print(", ");
  Serial.print(quantised_value * INDS_TO_FREQS);
  Serial.print(", ");
  Serial.println(sum_magnitude);
  //Serial.print(", ");
  //Serial.println(circle_ptr);

  // Update state machine
  state = stateMachine(state, loud_volume_counter);
}

float calculate_doppler_shift() {
  float approach = 0.0, retreat = 0.0;
  int samples_used = SAMPLES_IN_CALCULATION;
  for (int i = 0; i < SAMPLES_IN_CALCULATION; i++) {
    float app_freq = samples_on_approach[i] * INDS_TO_FREQS;
    float ret_freq = samples_on_retreat[i] * INDS_TO_FREQS;
    if (app_freq > 1100 || ret_freq > 1100 || app_freq < 100 || ret_freq < 100) {
      samples_used --;
    }
    else {
      approach += app_freq;
      retreat += ret_freq;
    }
  }
  approach /= samples_used;
  retreat /= samples_used;

  return SPEED_OF_SOUND * (approach - retreat) / (approach + retreat);
}

inline State stateMachine(State current, int counter) {
  /*
   * States are:
   *  approach,
   *  retreat,
   *  passed
   */
  State transitionTable[numberOfStates][2] =
  {
    {approach, retreat },
    {passed,   retreat },
    {approach, approach}
  };
  return transitionTable[current][counter >= SAMPLES_IN_CALCULATION];
}

char sample_audio() {
  uint16_t val = analogRead(AUDIO);
  char val2 = (char)(val/4 - 128);
  return val2;
}

void fill_arrays() {
  for(int i = 0; i < SAMPLES; i++) {
    vReal[i] = sample_audio();
    vImag[i] = 0;
    
    // delay takes a value in seconds
    delay(SAMPLE_WAIT / 1000);
  }
}

void find_mags_and_inds() {
  float max_magnitude = 0.0;
  float second_magnitude = 0.0;
  uint8_t max_quantisation = 0;
  uint8_t second_quantisation = 0;

  float magnitude;

  // TODO: why does this only go to SAMPLES/2
  //  is it related to the existence of the second_<something> variables?
  for (int i = 0; i < SAMPLES/2; i++) {
    magnitude = sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);

    if(magnitude > max_magnitude) {
      max_magnitude = magnitude;
      max_quantisation = i;
    } else if(magnitude > second_magnitude) {
      second_magnitude = magnitude;
      second_quantisation = i;
    }
  }
  update_sum_mag_and_av_ind(max_magnitude, second_magnitude, max_quantisation, second_quantisation);
}

// TODO: is this still needed if more mem is available for SAMPLES?
void update_sum_mag_and_av_ind(float maxm, float secm, uint8_t maxi, uint8_t seci) {

  if((maxi - seci) * (maxi - seci) <= FREQUENCY_SEPARATION_TOL) {
    quantised_value = (float)((maxi * maxm + seci * secm)/(maxm + secm));
    sum_magnitude = (float)(maxm + secm);
  } else {
    quantised_value = (float)maxi;
    sum_magnitude = (float)maxm;
  }
}
