# Arduino Drone Speed Sensor

This is the arduino project code for a drone speed sensor on the arduino.
The system uses a frequency and volume sensor to listen for a passing drone.
When a loud enough volume is reached, the system saves a prefix of the collected loud samples.
After the drone has passed and the samples from the microphone are below the volume threshold, a suffix of the samples are saved.
The prefix and suffix are averaged to get frequencies for the approach and retreat of the drone as it passed.
These average frequencies can be used to estimate the speed of the drone using the doppler effect.

## To-do

- Add hardware description.
- Dynamically calculate the threshold volume.
- Modify system for determining when the drone has passed
  - Currently, this is triggered when a sample is collected that is below the threshold volume.
  - This would work better if it only triggered after a number of samples are collected.
  - Perhaps another state in the state machine, in which samples are not saved but collection may resume.
