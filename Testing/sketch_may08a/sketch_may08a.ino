int sensorPin = A0;
int readings[16000];
//int readings_2[16000];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int t_start = millis();
  int counter=0;
  int soundRMS[25];
  for(int i=0; i<16000; i++)
  {
    int sensorVal = analogRead(sensorPin);
    readings[i] = sensorVal;
    soundRMS[(i%25)] = sensorVal;
    counter++;
    if(counter > 20000)
    {
      Serial.println("uh oh");
    }
  }
//  for(int i=0; i<16000; i++)
//  {
//    readings_2[i] = analogRead(sensorPin);
//  }
  int t_end = millis();
  float t_elapsed = (t_end - t_start)/1000.0; // in seconds
  float rate = 16000/t_elapsed;  
  Serial.println(t_elapsed);
  Serial.println(rate);
  delay(1000);
}
