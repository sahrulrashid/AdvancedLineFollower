#include<Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

#define l1 9
#define l2 10

#define r1 6
#define r2 5

#define turn_delay 650

int max_speed = 200;
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
boolean line_mode = 0;

//PID Variables
float Kp = 4, Kd = 8, Ki = 0;
//float Kp = 6, Kd = 18, Ki = 0;
float P = 0, I = 0, D = 0;
float PID = 0;
float error = 0;
float last_error = 0;
float set_position = 35;
boolean pid_flag = 0;

uint8_t sensor_pin = A0;
uint8_t sel_pins[3] = {7, 8, 11};

uint8_t button_pins[2] = {2, 3};
uint8_t trig_pin = A2;
uint8_t echo_pin = A1;

int left_speed = 0, right_speed = 0;

//UI variables:
boolean button_status[2] = {1, 1} , prev_button_status[2] = {1, 1};
boolean button[2] = {0, 0};
int mode = 0, max_modes = 10, prev_mode = 0, select = 0, prev_select = 0;
unsigned long last_ms = 0;
int update_ms = 250;
float battery = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  init_mux();
  init_lcd();
  UI_mode(mode);
  while (1)
  {
    if (pid_flag == 1)
    {
      break;
    }
    select = 0;
    button_stat();
    if (select == 1)
    {
      select_UI(mode);
    }
    if (mode != prev_mode)
    {
      UI_mode(mode);
    }
    prev_mode = mode;
    for (int i = 0; i < 2; i++)
    {
      Serial.print(button[i]);
      Serial.print(" ");
    }
    Serial.print(mode);
    Serial.print(" ");
    Serial.println(" ");
  }
  last_ms = millis();
}

void loop() {
  // cal_PID();
  // motors(base_speed - PID, base_speed + PID);
  if (millis() - last_ms > update_ms)
  {
    last_ms = millis();
    disp_sensor();
  }
  read_sensors();
  cal_PID();
  motors(base_speed + PID, base_speed - PID);
}

void select_UI(int index)
{
  switch (index)
  {
    case 1 :
      {
        lcd.clear();
        lcd.setCursor(4, 0);
        lcd.print("IE-NITK");
        lcd.setCursor(3, 1);
        lcd.print("CALIBRATING...");
        calibrate_sensors();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("THRESHOLD");
        lcd.setCursor(0, 1);
        lcd.print(threshold);
        while ( mode == 1)
        {
          button_stat();
          select = 0;
        }
        mode = 1;
        prev_mode = 0;
      }
      break;
    case 2 :
      {
        last_ms = millis();
        while (1)
        {
          if (millis() - last_ms > update_ms)
          {
            last_ms = millis();
            disp_sensor();
          }
          button_stat();
          if (mode != 2)
          {
            break;
          }
        }
        mode = 2;
        prev_mode = 1;
      }
      break;
    case 3 :
      {
        last_ms = millis();
        while (1)
        {
          button_stat();
          if (prev_select != select)
          {
            base_speed += 5;
            if (base_speed > 255)
            {
              base_speed = 0;
            }
          }
          prev_select = select;
          if (millis() - last_ms > update_ms)
          {
            last_ms = millis();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("SET BASE SPEED");
            lcd.setCursor(1, 1);
            lcd.print(base_speed);
          }
          button_stat();
          if (mode != 3)
          {
            break;
          }
        }
        mode = 3;
        prev_mode = 2;
      }
      break;
    case 4 :
      {
        last_ms = millis();
        while (1)
        {
          button_stat();
          if (prev_select != select)
          {
            max_speed += 5;
            if (max_speed > 255)
            {
              max_speed = 0;
            }
          }
          prev_select = select;
          if (millis() - last_ms > update_ms)
          {
            last_ms = millis();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("SET PID SPEED");
            lcd.setCursor(1, 1);
            lcd.print(max_speed);
          }
          button_stat();
          if (mode != 4)
          {
            break;
          }
        }
        mode = 4;
        prev_mode = 2;
      }
      break;
    case 5 :
      {
        last_ms = millis();
        while (1)
        {
          button_stat();
          if (prev_select != select)
          {
            Kp += 0.5;
            if (Kp > 10)
            {
              Kp = 0;
            }
          }
          prev_select = select;
          if (millis() - last_ms > update_ms)
          {
            last_ms = millis();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("SET Kp");
            lcd.setCursor(1, 1);
            lcd.print(Kp);
          }
          button_stat();
          if (mode != 5)
          {
            break;
          }
        }
        mode = 5;
        prev_mode = 4;
      }
      break;
    case 6 :
      { 
        last_ms = millis();
        while (1)
        {
          button_stat();
          if (prev_select != select)
          {
            Kd += 1;
            if (Kd > 20)
            {
              Kd = 0;
            }
          }
          prev_select = select;
          if (millis() - last_ms > update_ms)
          {
            last_ms = millis();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("SET Kd");
            lcd.setCursor(1, 1);
            lcd.print(Kd);
          }
          if (mode != 6)
          {
            break;
          }
        }
        mode = 6;
        prev_mode = 5;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SET Kd");
        lcd.setCursor(1, 1);
        lcd.print(Kd);
      }
      break;
    case 7 :
      {
        last_ms = millis();
        while (1)
        {
          button_stat();
          if (prev_select != select)
          {
            line_mode = !line_mode;
          }
          prev_select = select;
          if (millis() - last_ms > update_ms)
          {
            last_ms = millis();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("LINE MODE:");
            lcd.setCursor(1, 1);
            if (line_mode == 0)
            {
              lcd.print("BLACK");
            }
            else
            {
              lcd.print("WHITE");
            }
          }
          if (mode != 7)
          {
            break;
          }
        }
        mode = 7;
        prev_mode = 6;
      }
      break;
    case 8 :
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("BATTERY STAT");
        lcd.setCursor(1, 1);
        lcd.print(battery);
      }
      break;
    case 9 :
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("STARTING IN...");
        lcd.setCursor(0, 1);
        for (int i = 0; i < 8; i++)
        {
          lcd.print(i);
          lcd.print(" ");
          delay(500);
        }
        pid_flag = 1;
      }
      break;
  }
}

void UI_mode(int index)
{
  if (index >= max_modes)
  {
    return;
  }
  switch (index)
  {
    case 0 :
      {
        lcd.clear();
        lcd.setCursor(4, 0);
        lcd.print("IE-NITK");
        lcd.setCursor(1, 1);
        lcd.print("LINE FOLLOWER");
      }
      break;
    case 1 :
      {
        lcd.clear();
        lcd.setCursor(4, 0);
        lcd.print("IE-NITK");
        lcd.setCursor(3, 1);
        lcd.print("CALIBRATE");
      }
      break;
    case 2 :
      {
        lcd.clear();
        lcd.setCursor(4, 0);
        lcd.print("IE-NITK");
        lcd.setCursor(1, 1);
        lcd.print("SENSOR READINGS");
      }
      break;
    case 3 :
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SET BASE SPEED");
        lcd.setCursor(1, 1);
        lcd.print(base_speed);
      }
      break;
    case 4 :
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SET PID SPEED");
        lcd.setCursor(1, 1);
        lcd.print(max_speed);
      }
      break;
    case 5 :
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SET Kp");
        lcd.setCursor(1, 1);
        lcd.print(Kp);
      }
      break;
    case 6 :
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SET Kd");
        lcd.setCursor(1, 1);
        lcd.print(Kd);
      }
      break;
    case 7 :
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("LINE MODE:");
        lcd.setCursor(1, 1);
        if (line_mode == 0)
        {
          lcd.print("BLACK");
        }
        else
        {
          lcd.print("WHITE");
        }
      }
      break;
    case 8 :
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("BATTERY STAT");
        lcd.setCursor(1, 1);
        lcd.print(battery);
      }
      break;
    case 9 :
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("IE-NITK");
        lcd.setCursor(4, 1);
        lcd.print("START");
      }
      break;
  }
}

void disp_sensor()
{
  read_sensors();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("S: ");
  lcd.print(sensor_reading);
  lcd.setCursor(15, 0);
  if (junction == 1)
  {
    lcd.print("J");
  }
  else
  {
    lcd.print(" ");
  }

  lcd.setCursor(0, 1);
  for (int i = 0; i < 8 ; i++)
  {
    lcd.print(sensor[i]);
    lcd.print(" ");
  }
}

void button_stat()
{
  for (int i = 0; i < 2; i++)
  {
    button_status[i] = digitalRead(button_pins[i]);
    if (prev_button_status[i] != button_status[i] && button_status[i] == LOW)
    {
      delay(5);
      button_status[i] = digitalRead(button_pins[i]);
      if (prev_button_status[i] != button_status[i] && button_status[i] == LOW)
      {
        button[i] = !button[i];
        if ( i == 0 )
        {
          mode++;
        }
        if (mode == max_modes)
        {
          mode = 0;
        }
        if ( i == 1 && mode != 0 && mode != 8)
        {
          select++;
        }
      }

    }
    prev_button_status[i] = button_status[i];
  }
}

void init_lcd()
{
  lcd.init(); 
  lcd.backlight(); 
  lcd.clear();
  for (int i = 0; i < 2; i++)
  {
    pinMode(button_pins[i], INPUT_PULLUP);
  }
}

void init_mux()
{
  pinMode(sensor_pin, INPUT);
  for (int i = 0; i < 3; i++)
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
      if (line_mode == 1)
      {
        sensor[i] = !sensor[i];
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
  calib = 1;
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

