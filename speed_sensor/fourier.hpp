#ifndef FOURIER_HPP
#define FOURIER_HPP

// arduino hardware constants
#define AUDIO   A0
#define RED     2
//#define BLUE    3
#define GREEN   4
//#define YELLOW  9

#define LOG_SAMPLES    8
#define SAMPLES        ((int)pow(2,LOG_SAMPLES)) // must be power of 2
#define BUFFER_SIZE    48
#define SAMPLE_FREQ    4000.0
#define SAMPLE_WAIT    (1000000/SAMPLE_FREQ) // microseconds.
#define INDS_TO_FREQS  (2*SAMPLE_FREQ/SAMPLES)
#define SPEED_OF_SOUND 343

#define FREQUENCY_SEPARATION_TOL 4 // allows max and second frequencies to be 2 places apart

// this many samples at the start and end of hearing the drone
// these samples are averaged and used to calculate doppler shift
#define SAMPLES_IN_CALCULATION 16

// for state machine in simulating the passing of the drone
enum State {
  approach = 0,
  retreat,
  passed,
  numberOfStates // evaluates to 3 since there are 3 states
};

// Store highest freq and its magnitude, for short audio sample
void fourier_transform_loop();

float calculate_doppler_shift();
State stateMachine(State current, int counter);
char sample_audio();

// take samples to be used in fourier transform calcuation
void fill_arrays();

// find largest magnitude frequency. updates 'average_index' and 'sum_magnitude'
void find_mags_and_inds();

void update_sum_mag_and_av_ind(float maxm, float secm, uint8_t maxi, uint8_t seci);

#endif //FOURIER_HPP
