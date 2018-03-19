#define l1 9
#define l2 10

#define r1 6
#define r2 5

#define max_speed 200
#define turn_delay 650

int total_jun = 4;
int present_junction = 0, dest_jun = 1;
int base_speed =  120;
int align_speed = 100;
int turn_speed = 200;

boolean calib = 0;
uint16_t sensor[8] = {0}, min_reading = 1023, max_reading = 0, threshold = 750, data = 0;
int sensor_reading = 0;
int w[8] = {0, 10, 20, 30, 40, 50, 60, 70};
int dir = 1;
boolean junction = 0;
int sum = 0;
uint8_t sensor_enable = 4;

//PID Variables
float Kp = 4, Kd = 8, Ki = 0;
//float Kp = 6, Kd = 18, Ki = 0;
float P = 0, I = 0, D = 0;
float PID = 0;
float error = 0;
float last_error = 0;
float set_position = 35;

uint8_t sensor_pin = A0;
uint8_t sel_pins[3] = {7, 8, 11};

uint8_t button_pins[2] = {2, 3};
uint8_t trig_pin = A2;
uint8_t echo_pin = A1;

int left_speed = 0, right_speed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  init_mux();
  calib = 1;
  calibrate_sensors();
}

void loop() {
 // cal_PID();
 // motors(base_speed - PID, base_speed + PID);
 read_sensors();
   cal_PID();
  motors(base_speed + PID, base_speed - PID);
 }

void init_mux()
{
  pinMode(sensor_pin, INPUT);
  for(int i = 0; i < 3; i++)
  {
    pinMode(sel_pins[i], OUTPUT);
  }
}
void brake()
{
  analogWrite(l1, 255);
  analogWrite(l2, 255);
  analogWrite(r1, 255);
  analogWrite(r2, 255);
}
void cal_error()
{
  error = set_position - sensor_reading;
}

void cal_PID()
{
  cal_error();
  P = error;
  D = error - last_error;
  PID = (Kp * P) + (Kd * D);
  last_error = error;
  if (abs(PID) > max_speed)
  {
    if (PID > 0)
      PID = max_speed;
    else if (PID < 0)
    {
      PID = -1 * max_speed;
    }
  }
}


void read_sensors()
{
  int temp = 0;
  sum = 0;
  for (int i = 0; i < 8; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      digitalWrite(sel_pins[j], ((i >> j) & 0x01));
    }
    delayMicroseconds(1000);
    sensor[i] = analogRead(sensor_pin);
    delayMicroseconds(500);
    sensor[i] = analogRead(sensor_pin);
    delayMicroseconds(500);
    if (calib != 1)
    {
      if (sensor[i] < threshold)
      {
        sensor[i] = 1;
      }
      else
      {
        sensor[i] = 0;
      }
      sum += sensor[i];
      temp += (w[i] * sensor[i]);
    }
    Serial.print(sensor[i]);
    Serial.print(" ");
  }
  if (sum != 0)
  {
    temp = temp / sum;
    sensor_reading = temp;
  }
  if (sum == 8)
  {
    junction = 1;
  }
  else
  {
    junction = 0;
  }
  Serial.print(sensor_reading);
  Serial.print(" ");
  Serial.print(junction);
  Serial.print(" ");
  Serial.println("");
}

void calibrate_sensors()
{
  motors(-base_speed, base_speed);
  for (int i = 0; i < 15; i++)
  {
    read_sensors();
    for (int i = 0; i < 8; i++)
    {

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
   motors(base_speed, -base_speed);
  for (int i = 0; i < 15; i++)
  {
    read_sensors();
    for (int i = 0; i < 8; i++)
    {
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
  threshold += 150;
  motors(0, 0);
  Serial.println(threshold);
  Serial.println(threshold);
  delay(2000);
  calib = 0;
}

void motors(int left, int right)
{
  if (abs(left) > max_speed)
  {
    if (left > 0)
      left = max_speed;
    else if (PID < 0)
    {
      left = -1 * max_speed;
    }
  }
  if (abs(right) > max_speed)
  {
    if (right > 0)
      right = max_speed;
    else if (PID < 0)
    {
      right = -1 * max_speed;
    }
  }
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

