/* Example sketch to control a stepper motor with TB6560 stepper motor driver and Arduino without a library. More info: https://www.makerguides.com */

// Define stepper motor connections and steps per revolution:
#define dirPin 2
#define stepPin 4
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#define stepsPerRevolution 15000

ros::NodeHandle nh;

std_msgs::Bool cmd_msg;
std_msgs::Bool elevator_msg;

ros::Publisher elevator("/elevator/elevator_up", &elevator_msg);

bool elevator_up = false;

//Topic Callback
void motor_cb( const std_msgs::Bool& cmd_msg ){
  if ( cmd_msg.data and not elevator_up ){
    up();
  }else if (not cmd_msg.data and elevator_up){
    down();
  }
}


ros::Subscriber<std_msgs::Bool> sub("/brain/elevator", &motor_cb);


void setup() {
  // Declare pins as output:
  Serial.begin(57600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(elevator);
}

void loop() {
  //Serial.println(1000);
  elevator_msg.data = elevator_up;
  elevator.publish( &elevator_msg );
  nh.spinOnce();
  delay(1);
}

void up(){

    // Set the spinning direction counterclockwise:
  digitalWrite(dirPin, LOW);
  
  // Spin the stepper motor 1 revolution quickly:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(300);
    }
  delay(1000);
  elevator_up = true;
}

void down(){

  digitalWrite(dirPin, HIGH);

  // Spin the stepper motor 1 revolution slowly:
  for (int i = 0; i < stepsPerRevolution; i++) {
      // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(300);
  }
  delay(1000);
  elevator_up = false;
}
