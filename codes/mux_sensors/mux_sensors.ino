uint8_t sense_pin = A0;
uint8_t sel[4] = {2, 3, 4, 5};
/*
  0 - S0 (LSB)
  1 - S1
  2 - S2
  3 - S3 (MSB)
*/
int sensor_data[16] = {0};
double present_us = 0;
void setup() {
  // put your setup code here, to run once:
  init_sensors();
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  present_us = micros();
  read_sensors();
  Serial.println(micros() - present_us);
}

void init_sensors()
{
  pinMode(sense_pin, INPUT);
  for (int i = 0; i < 4; i++)
  {
    pinMode(sel[i], OUTPUT);
  }
}

void read_sensors()
{
  for (int i = 0; i < 16; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      digitalWrite(sel[j], ((i >> j) & 0x01));
    }
    sensor_data[i] = analogRead(sense_pin);
    Serial.print(String(sensor_data[i]) + " ");
  }
  Serial.print(" ");
}

