#include  <Servo.h>

Servo plate_servo;
Servo joint_servo_1;
Servo joint_servo_2;
Servo joint_servo_3;

void setup() {
  plate_servo.attach(8);
  joint_servo_1.attach(9);
  joint_servo_2.attach(10);
  joint_servo_3.attach(11);
}

void loop() {
  plate_servo.write(90);
  joint_servo_1.write(85);
  joint_servo_2.write(90);
  joint_servo_3.write(85);
  delay(15);
}