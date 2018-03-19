void setup() {
  // put your setup code here, to run once:
pinMode(2,OUTPUT);
pinMode(A0,INPUT);
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(2,LOW);
delayMicroseconds(500);
int a = analogRead(A0);
Serial.print(a);
Serial.print('\t');

digitalWrite(2,HIGH);
int b = analogRead(A0);
Serial.print(b);
Serial.print('\t');
int c = b-a;
Serial.println(c);
}
