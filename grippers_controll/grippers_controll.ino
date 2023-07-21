#include <Servo.h>

const int NUMBER_OF_LEGS = 5;

int GRIPPERS_CONTROL_PINS[NUMBER_OF_LEGS] = {3, 5, 6, 9, 10};
int GRIPPERS_FEEDBACK_PINS[NUMBER_OF_LEGS] = {A1, A3, A5, A6, A7};
int SWITCH_PINS[5] = {A2, A4, 4, 7, 11};

// Commands for controlling a gripper.
char OPEN_COMMAND = 'o';
char CLOSE_COMMAND = 'c';
String INIT_MESSAGE = "init";

String INIT_RESPONSE = "OK";
char GRIPPER_OPENED_RESPONSE = '1';
char GRIPPER_CLOSED_RESPONSE = '0';
char GRIPPER_MOVING_RESPONSE = '2';

// Values when servo is in closed or open position.
int GRIPPERS_OPEN_THRESHOLD[NUMBER_OF_LEGS] = {670, 670, 670, 670, 670};
int GRIPPERS_CLOSE_THRESHOLD[NUMBER_OF_LEGS] = {490, 490, 490, 490, 490};

float OPEN_STROKE_MM = 0;
float CLOSED_STROKE_MM = 14;
float MAX_STROKE_MM = 30;

int switch_value;
int servo_value;
Servo grippers[NUMBER_OF_LEGS];

struct CommandGrippersIds 
{
  char command;
  int gripper_id;
};

struct CommandGrippersIds parse_data(String data)
{
  struct CommandGrippersIds cmd_grip;
  if (data[0] == OPEN_COMMAND || data[0] == CLOSE_COMMAND)
  {
    cmd_grip.command = data[0];
  }
  cmd_grip.gripper_id = data[1] - '0';
  
  return cmd_grip;
}

void set_stroke_mm(int gripper_id, float stroke_desired)
{
  // 0 mm -> 1000 usec
  // 30 mm -> 2000 usec
  int k = 1000 / MAX_STROKE_MM;
  int n = 1000;
  int usec = k * stroke_desired + n;
  Servo gripper = grippers[gripper_id];
  gripper.writeMicroseconds(usec);
}

// Create string message from grippers states:
// 0 - closed, 1 - open, 2 - in between.
// Example: "11011" - all grippers except third are opened.
String get_grippers_states_message(int current_states[])
{
  char message[NUMBER_OF_LEGS];
  for (int i = 0; i < NUMBER_OF_LEGS; i++)
  {
    if (current_states[i] < GRIPPERS_CLOSE_THRESHOLD[i])
    {
      message[i] = GRIPPER_CLOSED_RESPONSE;
    }
    else if (current_states[i] > GRIPPERS_OPEN_THRESHOLD[i])
    {
      message[i] = GRIPPER_OPENED_RESPONSE;
    }
    else if (current_states[i] < GRIPPERS_OPEN_THRESHOLD[i] && current_states[i] > GRIPPERS_CLOSE_THRESHOLD[i])
    {
      message[i] = GRIPPER_MOVING_RESPONSE;
    }
  }
  return message;
}
String get_switches_states_message(int current_states[])
{
  String message;
  for (int i = 0; i < NUMBER_OF_LEGS; i++)
  {
    message += String(current_states[i]);
  }
  return message;
}

void setup() 
{
  Serial.begin(115200);
  for (int i = 0; i < NUMBER_OF_LEGS; i++)
  {
    grippers[i].attach(GRIPPERS_CONTROL_PINS[i]);
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);
  }
}

void loop() 
{
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
      struct CommandGrippersIds cmd_grip = parse_data(data);

      if (cmd_grip.command == OPEN_COMMAND)
      {
        set_stroke_mm(cmd_grip.gripper_id, OPEN_STROKE_MM);
      }
      else if (cmd_grip.command == CLOSE_COMMAND)
      {
        set_stroke_mm(cmd_grip.gripper_id, CLOSED_STROKE_MM);
      }
    }
  }

  int grippers_states[NUMBER_OF_LEGS];
  int switches_states[NUMBER_OF_LEGS];
  for (int i = 0; i < NUMBER_OF_LEGS; i++)
  {
    grippers_states[i] = analogRead(GRIPPERS_FEEDBACK_PINS[i]);
    switches_states[i] = digitalRead(SWITCH_PINS[i]);
  }

  String grippers_message = get_grippers_states_message(grippers_states);
  String switches_message = get_switches_states_message(switches_states);
  Serial.println(grippers_message + switches_message + '\n');
  delay(100);
}
