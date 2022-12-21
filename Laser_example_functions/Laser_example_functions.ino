int incomingByte = 0; // for incoming serial data

void setup() {
  //put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
}

void loop() {
  char func_Number = 0;

  if (Serial.available())
  {
    func_Number = Serial.read();
  }

  switch (func_Number)
  {
    case '1':
      read_distance();
      break;
    case '2':
      laser_on();
      break;
    case '3':
      laser_off();
      break;
    case '4':
      digitalWrite(7, HIGH); //laser power on
      break;
    case '5':
      digitalWrite(7, LOW); //laser power off
      break;

  }

  func_Number = 0;

}
//Functions-----------------------------

void read_distance()
{
  //Send command
  Serial.write(0xAA);
  Serial.write(0x01);
  Serial.write(0x44);
  Serial.write(0x45);
  Serial.write(0xA8);

  //Parse result
  String Distance = Serial.readString();
  String Distance1 = Distance.substring(4, 6);
  String Distance2 = ".";
  String Distance3 = Distance.substring(6, 9);
  String Distance_concat =  Distance1 + Distance2 + Distance3;
  double Distance_concat_dob = Distance_concat.toDouble();
  String Distance4 = "m";
  String Distance_concat_string = Distance_concat_dob + Distance4;
  Serial.println(Distance_concat_string);
}

void laser_on()
{
  Serial.write(0xAA);
  Serial.write(0x01);
  Serial.write(0x42);
  Serial.write(0x43);
  Serial.write(0xA8);
}

void laser_off()
{
  Serial.write(0xAA);
  Serial.write(0x01);
  Serial.write(0x43);
  Serial.write(0x44);
  Serial.write(0xA8);
}

void busser_on_off()
{
  Serial.write(0xAA);
  Serial.write(0x01);
  Serial.write(0x47);
  Serial.write(0x0); //change this to 0x01 to turn busser on
  Serial.write(0x49);
  Serial.write(0xA8);
}
