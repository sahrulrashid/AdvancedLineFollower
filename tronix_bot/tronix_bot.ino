#define l1 5
#define l2 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
#define r1 6
#define r2 9

uint16_t sensor[6] = {0}, min_reading = 1023, max_reading = 0, threshold = 750, data = 0;
int next_dir = 0;
uint16_t max_speed = 150;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  pinMode(r2, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(7, OUTPUT);

 calibrate_sensors();
}

void loop() {
  // put your main code here, to run repeatedly:
  read_sensors();
  int temp = data & 0b00011110;
  if (data & 0b00100000 && temp != 0b00011110)
  {
    next_dir = 0;
    digitalWrite(13,HIGH);
    digitalWrite(7,LOW);
  }
  if (data & 0b00000001 && temp != 0b00011110)
  {
    next_dir = 1;
    digitalWrite(7,HIGH);
    digitalWrite(13, LOW);
  }

  data &= 0b00011110;

  if (data == 0b01100)
  {
    //go straight
    motors(max_speed, max_speed);
  }
  else if (data == 0b1000)
  {
    motors(0.75 * max_speed, max_speed);
  }
  else if (data == 0b11000)
  {
    motors(0.5 * max_speed, max_speed);
  }
  else if (data == 0b11000)
  {
    motors(0.5 * max_speed, max_speed);
  }
  else if (data == 0b10000)
  {
    motors(0.25 * max_speed, max_speed);
  }
  else if (data == 0b100)
  {
    motors(max_speed, 0.75 * max_speed);
  }
  else if(data == 0b110)
  {
    motors(max_speed, 0.5 * max_speed);
  }
  else if(data == 0b10)
  {
    motors(max_speed, 0.25 * max_speed);
  }
  else if(data == 0b00001110)
  {
    motors(-max_speed, max_speed);
  }
  else if(data == 0b00011100)
  {
    motors(max_speed, -max_speed);
  }
  else if(data == 0b00001110 || data == 0b00111100)
  {
    if(next_dir == 0)
    {
      motors(max_speed, max_speed);
      delay(100);
      Serial.println("left Turn");
      motors(-120,120);
      while(1)
      {
        read_sensors();
        if(sensor[1] == 1)
        {
          break;
        }
      }
    }
    else
    {
      motors(max_speed, max_speed);
      delay(100);
      Serial.println("right Turn");
      motors(120,-120);
      while(1)
      {
        read_sensors();
        if(sensor[4] == 1)
        {
          break;
        }
      }
    }
  }
}

void read_sensors()
{
  data = 0;
  for (int i = 0; i < 6; i++)
  {
    delay(1);
    data = data << 1;
    sensor[i] = analogRead(A0 + i);
    if (sensor[i] < threshold)
    {
      sensor[i] = 1;
    }
    else
    {
      sensor[i] = 0;
    }
    delay(1);
    data |= sensor[i];
  }
  Serial.println(data, BIN);
}

void calibrate_sensors()
{
   motors(-max_speed, max_speed);
  for (int i = 0; i < 100; i++)
  {
    for (int i = 0; i < 6; i++)
    {
      sensor[i] = analogRead(A0 + i);
      delay(1);
      if (min_reading > sensor[i])
      {
        min_reading = sensor[i];
      }
      if (max_reading < sensor[i])
      {
        max_reading = sensor[i];
      }
    }
  }
  motors(max_speed, -max_speed);
  for (int i = 0; i < 100; i++)
  {
    for (int i = 0; i < 6; i++)
    {
      sensor[i] = analogRead(A0 + i);
      delay(1);
      if (min_reading > sensor[i])
      {
        min_reading = sensor[i];
      }
      if (max_reading < sensor[i])
      {
        max_reading = sensor[i];
      }
    }
  }
  threshold = min_reading + (max_reading - min_reading) / 2;
  threshold += 125;
  motors(0, 0);
  Serial.println(threshold);
  delay(2000);
}

void motors(int left, int right)
{
  if (left >= 0 && right >= 0)
  {
    analogWrite(l1, left);
    analogWrite(l2, 0);
    analogWrite(r1, right);
    analogWrite(r2, 0);
  }
  if (left < 0 && right >= 0)
  {
    analogWrite(l1, 0);
    analogWrite(l2, -left);
    analogWrite(r1, right);
    analogWrite(r2, 0);
  }
  if (left >= 0 && right < 0)
  {
    analogWrite(l1, left);
    analogWrite(l2, 0);
    analogWrite(r1, 0);
    analogWrite(r2, -right);
  }
  if (left < 0 && right < 0)
  {
    analogWrite(l1, 0);
    analogWrite(l2, -left);
    analogWrite(r1, 0);
    analogWrite(r2, -right);
  }
}

