uint16_t sensor[6] = {0};
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
read_sensors();
}

void read_sensors()
{
  for(int i = 0; i < 6; i++)
  {
    sensor[i] = analogRead(A0+i);

    delay(1);
    Serial.print((String)sensor[i] + " ");
    delay(1);
  }
  Serial.println(" ");
}

