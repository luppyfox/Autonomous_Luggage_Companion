#include <Servo.h>

int incomingByte = 0;
int data_arr[] = { 90, 135, 10, 135 };
int olddata_arr[] = { 90, 135, 10, 135};
int arr_len = sizeof(data_arr) / sizeof(int);
int move_step = 1;

Servo rotate_plate, jonit_1, jonit_2, jonit_3;

void setup() {
  Serial.begin(9600);
  // moveStartTime = millis(); // start moving

  rotate_plate.attach(8);
  jonit_1.attach(9);
  jonit_2.attach(10);
  jonit_3.attach(11);
}

void loop() {
  int i = 0;
  while (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (i >= arr_len) {
      break;
    }
    if (incomingByte != -1) {
      data_arr[i] = incomingByte;
      i = i + 1;
    }
  }
  if (data_arr[0] > olddata_arr[0]) {
    olddata_arr[0] = olddata_arr[0] + move_step;
  } else if (data_arr[0] < olddata_arr[0]) {
    olddata_arr[0] = olddata_arr[0] - move_step;
  }
  if (data_arr[1] > olddata_arr[1]) {
    olddata_arr[1] = olddata_arr[1] + move_step;
  } else if (data_arr[1] < olddata_arr[1]) {
    olddata_arr[1] = olddata_arr[1] - move_step;
  }
  if (data_arr[2] > olddata_arr[2]) {
    olddata_arr[2] = olddata_arr[2] + move_step;
  } else if (data_arr[2] < olddata_arr[2]) {
    olddata_arr[2] = olddata_arr[2] - move_step;
  }
  if (data_arr[3] > olddata_arr[3]) {
    olddata_arr[3] = olddata_arr[3] + move_step;
  } else if (data_arr[3] < olddata_arr[3]) {
    olddata_arr[3] = olddata_arr[3] - move_step;
  }

  rotate_plate.write(olddata_arr[0]);
  jonit_1.write(olddata_arr[1]);
  jonit_2.write(olddata_arr[2]);
  jonit_3.write(olddata_arr[3]);
  delay(20);
}
