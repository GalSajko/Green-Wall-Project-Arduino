#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPU6050.h>
#include <utility/imumaths.h>
#include <math.h>

Adafruit_PWMServoDriver myServo = Adafruit_PWMServoDriver();
Adafruit_MPU6050 mpu;

/* Pins declaration. */
int DIRECTION_CONTROL_PINS[2] = {2, 4};
int PUMP_PWM_PINS[3] = {3, 5, 6};
int NUMBER_OF_PUMPS = 3;
int VOLTAGE_DIVIDER = A6;
int TUBE_HOLDER_SERVO = 11;

/* Servo break MIN/MAX position */
int MIN_MAX_VALUES[5][2] = {
	{ 400, 2100 },
	{ 400, 2100 },
	{ 400, 2100 },
	{ 400, 2100 },
  { 400, 2100 },
};

int EXPANDED_TUBE_HOLDER_VALUE = 2875;
int CONTRACTED_TUBE_HOLDER_VALUE = 1015;

/* Commands to communicate with PC. */
String INIT_MESSAGE = "init";
String INIT_RESPONSE = "OK";

char PUMP_OFF_COMMAND = '0';
char PUMP_ON_COMMAND = '1';

char BREAK_OFF_COMMAND = '4';
char BREAK_ON_COMMAND = '5';
int ALL_BREAKS = 5;

char EXPAND_TUBE_HOLDER = '6';
char CONTRACT_TUBE_HOLDER = '7';

float G = 9.81;
float FLAT_ACC_OFFSETS[3] = {9.92 - G, 0.45, 1.59};

/* Analog voltage for controlling the water pumps. */
int PUMP_ON_VOLTAGE = 250;
int PUMP_OFF_VOLTAGE = 0;

char pump_command = PUMP_OFF_COMMAND;
int pump_id = 0;

char break_command;
int break_id = 0;

struct PcCommands
{
  char command;
  int pump_id;
  int break_id;
};

String add_plus_signs(String a, String b, String c)
{
  if (a[0] != '-')
  {
    a = '+' + a;
  }
  if (b[0] != '-')
  {
    b = '+' + b;
  }
  if (c[0] != '-')
  {
    c = '+' + c;
  }

  String abc = a + b + c;

  return abc;
}

String prepare_gravity_string(float value)
{
  String gravity_string = String(abs(value), 2);

  bool add_zero = abs(value) < 10.0;
  bool add_plus = value >= 0.0;

  if (add_zero)
  {
    gravity_string = "0" + gravity_string;
  }
  if (add_plus)
  {
    gravity_string = "+" + gravity_string;
  }
  else
  {
    gravity_string = "-" + gravity_string;
  }

  return gravity_string;
}

String get_gravity_vector_message(sensors_event_t event)
{
  float x_val = -(event.acceleration.x - FLAT_ACC_OFFSETS[0]);
  String y = prepare_gravity_string(x_val);

  float y_val = event.acceleration.y - FLAT_ACC_OFFSETS[1];
  String x = prepare_gravity_string(y_val);

  float z_val = -(event.acceleration.z - FLAT_ACC_OFFSETS[2]);
  String z = prepare_gravity_string(z_val);

  return x + y + z;
}

/* Create message from voltage devider */
String battery_voltage()
{
  float voltage_value = analogRead(VOLTAGE_DIVIDER);
  float voltage = voltage_value * 4.0 * 4.75 / 1023; // 4x voltage drop (R1 = 100kOhm, R2 = 300kOhm), 4.75 Arduino NANO max. ADC
  String voltage_output;

  if (voltage < 10) {
    voltage_output = "0" + String(voltage, 1);
  }
  else {
    voltage_output = String(voltage, 1); // 4 digit value (. included)
  }

  return voltage_output;
}

/* Parse message from PC into commands. */
struct PcCommands parse_data(String data)
{
  struct PcCommands commands;
  if (data[0] == PUMP_ON_COMMAND || data[0] == PUMP_OFF_COMMAND)
  {
    commands.command = data[0];
    int pump_id = data[1] - '0';
    if (pump_id < 3 && pump_id >= 0)
    {
      commands.pump_id = pump_id;
    }
  }
  else if (data[0] == BREAK_ON_COMMAND || data[0] == BREAK_OFF_COMMAND)
  {
    commands.command = data[0];
    int break_id = data[1] - '0';
    if (break_id < 6 && break_id >= 0)
    {
      commands.break_id = break_id;
    }
  }
  else if (data[0] == EXPAND_TUBE_HOLDER || data[0] == CONTRACT_TUBE_HOLDER)
  {
    commands.command = data[0];
  }
  return commands;
}

/* Controll water pumps. */
void pump_control(char command, int pump_id)
{
  int value;
  if (command == PUMP_ON_COMMAND)
  {
    value = PUMP_ON_VOLTAGE;
  }
  else if (command == PUMP_OFF_COMMAND)
  {
    value = PUMP_OFF_VOLTAGE;
  }

  analogWrite(PUMP_PWM_PINS[pump_id], value);
}

/* Brake control */
void break_control(char command, int break_id)
{
  if (command == BREAK_ON_COMMAND)
  {
    if (break_id == ALL_BREAKS) {
      for (int break_counter = 0; break_counter < 5; break_counter++)
      {
        myServo.writeMicroseconds(break_counter, MIN_MAX_VALUES[break_counter][1]);
      }
    }
    else if (break_id < 5)
    {
      myServo.writeMicroseconds(break_id, MIN_MAX_VALUES[break_id][1]);
    }
  }
  else if (command = BREAK_OFF_COMMAND)
  {
    if (break_id == ALL_BREAKS) {
      for (int break_counter = 0; break_counter < 5; break_counter++)
      {
        myServo.writeMicroseconds(break_counter, MIN_MAX_VALUES[break_counter][0]);
      }
    }
    else if (break_id < 5)
    {
      myServo.writeMicroseconds(break_id, MIN_MAX_VALUES[break_id][0]);
    }
  }
}

void tube_holder_control(char command)
{
  if (command == EXPAND_TUBE_HOLDER)
  {
    myServo.writeMicroseconds(TUBE_HOLDER_SERVO, EXPANDED_TUBE_HOLDER_VALUE);
  }
  else if (command == CONTRACT_TUBE_HOLDER)
  {
    myServo.writeMicroseconds(TUBE_HOLDER_SERVO, CONTRACTED_TUBE_HOLDER_VALUE);
  }
}

void read_average_offsets()
{
  float x_sum = 0;
  float y_sum = 0;
  float z_sum = 0;

  int no_of_measurements = 10000;

  Serial.println("Measuring initial offsets ...");
  for (int i = 0; i < no_of_measurements; i++)
  {
    sensors_event_t acc_event, gyro_event, temp_event;
    mpu.getEvent(&acc_event, &gyro_event, &temp_event);

    x_sum += acc_event.acceleration.x;
    y_sum += acc_event.acceleration.y;
    z_sum += acc_event.acceleration.z;
  }
  Serial.print("X: ");
  Serial.println(x_sum / no_of_measurements);
  Serial.print("Y: ");
  Serial.println(y_sum / no_of_measurements);
  Serial.print("Z: ");
  Serial.println(z_sum / no_of_measurements);
}

void setup() 
{
  Serial.begin(115200);
  myServo.begin();
  myServo.setPWMFreq(60);

  if (!mpu.begin())
  {
    Serial.println("No MPU6050 detected.");
    while(1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);
  
  digitalWrite(DIRECTION_CONTROL_PINS[0], HIGH);
  digitalWrite(DIRECTION_CONTROL_PINS[1], LOW);

  // read_average_offsets();
}

void loop() 
{
  sensors_event_t acc_event, gyro_event, temp_event;
  mpu.getEvent(&acc_event, &gyro_event, &temp_event);

  if (Serial.available() > 0)
  {
    String data = Serial.readStringUntil('\n');
    
    if (data == INIT_MESSAGE)
    {
      Serial.println(INIT_RESPONSE + '\n');
      return;
    }
    else
    {
      struct PcCommands commands = parse_data(data);
      if (commands.command == PUMP_ON_COMMAND || commands.command == PUMP_OFF_COMMAND)
      {
        pump_command = commands.command;
        pump_id = commands.pump_id;
      }
      else if (commands.command == BREAK_ON_COMMAND || commands.command == BREAK_OFF_COMMAND)
      {
        break_command = commands.command;
        break_id = commands.break_id;
        break_control(break_command, break_id);
      }
      else if (commands.command == EXPAND_TUBE_HOLDER || commands.command == CONTRACT_TUBE_HOLDER)
      {
        tube_holder_control(commands.command);
      }
    }
  }
  pump_control(pump_command, pump_id);
  Serial.print(get_gravity_vector_message(acc_event) + battery_voltage() + '\n');
}

/* Servo enabling/disabling holding torque 

  myServo.setPWM(0, 0, 150);                                   // holds the position
  myServo.setPWM(0, 4096, 0); OR myServo.setPWM(0, 0, 4096);  // turns off the motor/makes it movable 

*/