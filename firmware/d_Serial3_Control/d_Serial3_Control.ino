

void setup() {
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  Serial.begin(9600);
  // Serial3.begin(9600);
}
char data;
void loop() {
  // when you typed any character in terminal
  if (Serial.available()) {
    data = (char)Serial.read();


  }
  if (data == 'F')
  {
    forward();
  }
  if (data == 'B')
  {
    backward();
  }
  if (data == 'L')
  {
    rightward();
  }
  if (data == 'R')
  {
    leftward();
    
  }
  if (data == 'S')
  {
    
  digitalWrite(4, 0);
  digitalWrite(5, 0);
  digitalWrite(6, 0);
  digitalWrite(7, 0);

  digitalWrite(8, 0);
  digitalWrite(9, 0);
  digitalWrite(10, 0);
  digitalWrite(11, 0);  
  }
}

void forward()
{
  digitalWrite(4, 1);
  digitalWrite(5, 0);
  digitalWrite(6, 0);
  digitalWrite(7, 1);

  digitalWrite(8, 1);
  digitalWrite(9, 0);
  digitalWrite(10, 1);
  digitalWrite(11, 0);
}

void backward()
{
  digitalWrite(4, 0);
  digitalWrite(5, 1);
  digitalWrite(6, 1);
  digitalWrite(7, 0);

  digitalWrite(8, 0);
  digitalWrite(9, 1);
  digitalWrite(10, 0);
  digitalWrite(11, 1);
}
void leftward()
{
  digitalWrite(4, 1);
  digitalWrite(5, 0);
  digitalWrite(6, 1);
  digitalWrite(7, 0);

  digitalWrite(8, 0);
  digitalWrite(9, 1);
  digitalWrite(10, 1);
  digitalWrite(11, 0);
}

void rightward()
{
  digitalWrite(4, 0);
  digitalWrite(5, 1);
  digitalWrite(6, 0);
  digitalWrite(7, 1);

  digitalWrite(8, 1);
  digitalWrite(9, 0);
  digitalWrite(10, 0);
  digitalWrite(11, 1);
}
