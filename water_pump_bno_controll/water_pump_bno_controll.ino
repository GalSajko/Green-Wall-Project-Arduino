#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

/*BNO calibration data.*/
adafruit_bno055_offsets_t CALIB_DATA = 
{
  -1, 19, -21, 0, 0, 0, 2, -1, 2, 1000, 480
};

/*Pins declaration.*/
int DIRECTION_CONTROL_PINS[2] = {2, 4};
int PWM_PINS[3] = {3, 5, 6};
int NUMBER_OF_PUMPS = 3;

/*Commands to communicate with PC.*/
String INIT_MESSAGE = "init";
String INIT_RESPONSE = "OK";
char PUMP_OFF_COMMAND = '0';
char PUMP_ON_COMMAND = '1';
char INIT_BNO = '2';
char READ_BNO_RPY = '3';

/*Analog voltage for controlling the water pumps.*/
int PUMP_ON_VOLTAGE = 250;
int PUMP_OFF_VOLTAGE = 0;

char pump_command = PUMP_OFF_COMMAND;
int pump_id = 0;


struct Eulers 
{
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
} init_rpy;

struct PcCommands
{
  char command;
  int pump_id;
};

/*Substract starting rpy values from eulers.*/
void substract_init_rpy_values(Eulers *eulers)
{
  eulers->pitch -= init_rpy.pitch;
  eulers->roll -= init_rpy.roll;
  eulers->yaw -= init_rpy.yaw;
}

/*Convert quaternion into eulers.*/
Eulers get_euler_angles_from_quaternion(imu::Quaternion quaternion, bool do_substract = true)
{
    Eulers eulers;
    float q_1 = quaternion.x();
    float q_2 = quaternion.y();
    float q_3 = quaternion.z();
    float q0 = quaternion.w();

    eulers.roll = atan2(2 * (q0 * q_1 + q_2 * q_3), 1 - 2 * (q_1 * q_1 + q_2 * q_2));
    eulers.pitch = asin(2 * (q0 * q_2 - q_1 * q_3));
    eulers.yaw = atan2(2 * (q0 * q_3 + q_1 * q_2), -1 + 2 * (q0 * q0 + q_1 * q_1));

    if (do_substract)
    {
      substract_init_rpy_values(&eulers);
    }
  
    return eulers;
}

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

/*Create message from eulers, to send on PC.*/
String get_euler_message(Eulers eulers)
{
  String roll = String(eulers.yaw);
  String pitch = String(eulers.roll * (-1));
  String yaw = String(eulers.pitch);

  String rpy = add_plus_signs(roll, pitch, yaw);

  return rpy;
}

String get_gravity_vector_message(sensors_event_t event)
{
  String x = String(event.acceleration.x);
  String y = String(event.acceleration.y);
  // Minus because acc reading declaration.
  String z = String(-event.acceleration.z);

  String gravity = add_plus_signs(x, y, z);

  return gravity;
}

/*Parse message from PC into commands.*/
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
  else if (data[0] == INIT_BNO || data[0] == READ_BNO_RPY)
  {
    commands.command = data[0];
  }

  return commands;
}

/*Controll water pumps.*/
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

  analogWrite(PWM_PINS[pump_id], value);
}

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() 
{
  Serial.begin(115200);

  if (!bno.begin(OPERATION_MODE_CONFIG))
  {
    Serial.print("No BNO055 detected.");
    while(1);
  }

  // Load BNO calibration.
  bno.setSensorOffsets(CALIB_DATA);
  // Put BNO in IMU mode (relative heading).
  bno.setMode(OPERATION_MODE_IMUPLUS);

  delay(1000);

  // Remap BNO axis to match spider's orientation on the wall.
  bno.setAxisRemap(bno.REMAP_CONFIG_P1);
  bno.setAxisSign(bno.REMAP_SIGN_P2);

  bno.setExtCrystalUse(true);
  
  digitalWrite(DIRECTION_CONTROL_PINS[0], HIGH);
  digitalWrite(DIRECTION_CONTROL_PINS[1], LOW);
}

void loop() 
{
  Eulers eulers = get_euler_angles_from_quaternion(bno.getQuat());
  sensors_event_t event;
  bno.getEvent(&event, bno.VECTOR_GRAVITY);

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
      else if (commands.command == INIT_BNO)
      {
         init_rpy = get_euler_angles_from_quaternion(bno.getQuat(), false);
      }
    }
  }
  pump_control(pump_command, pump_id);
  Serial.print(get_euler_message(eulers) + get_gravity_vector_message(event) + '\n');
}
