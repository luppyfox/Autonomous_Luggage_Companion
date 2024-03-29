#include <Servo.h>

// int myArray[4]; //this value is the upgratable data
// byte* 

bool newData=false;

int incomingByte = 0;
int data_arr[4] = {90, 180, 45, 180};
int olddata_arr[4] = {90, 180, 45, 180};

byte*
ddata = reinterpret_cast<byte*>(&data_arr);

size_t arr_len = sizeof(data_arr);
int move_step = 1;

Servo rotate_plate, jonit_1, jonit_2, jonit_3;

void setup() {
  Serial.begin(9600);
  rotate_plate.attach(8);
  jonit_1.attach(9);
  jonit_2.attach(10);
  jonit_3.attach(11);
}

void loop() {
  checkForNewData();
  if (data_arr[0] > olddata_arr[0]) {
    // int step_u = abs(data_arr[0] - olddata_arr[0]) / move_step;
    olddata_arr[0] = olddata_arr[0] + move_step;
  } else if (data_arr[0] < olddata_arr[0]) {
    // int step_u = abs(data_arr[0] - olddata_arr[0]) / move_step;
    olddata_arr[0] = olddata_arr[0] - move_step;
  }
  if (data_arr[1] > olddata_arr[1]) {
    // int step_u = abs(data_arr[0] - olddata_arr[0]) / move_step;
    olddata_arr[1] = olddata_arr[1] + move_step + move_step;
  } else if (data_arr[1] < olddata_arr[1]) {
    // int step_u = abs(data_arr[0] - olddata_arr[0]) / move_step;
    olddata_arr[1] = olddata_arr[1] - move_step - move_step;
  }
  if (data_arr[2] > olddata_arr[2]) {
    // int step_u = abs(data_arr[0] - olddata_arr[0]) / move_step;
    olddata_arr[2] = olddata_arr[2] + move_step;
  } else if (data_arr[2] < olddata_arr[2]) {
    // int step_u = abs(data_arr[0] - olddata_arr[0]) / move_step;
    olddata_arr[2] = olddata_arr[2] - move_step;
  }
  if (data_arr[3] > olddata_arr[3]) {
    // int step_u = abs(data_arr[0] - olddata_arr[0]) / move_step;
    olddata_arr[3] = olddata_arr[3] + move_step;
  } else if (data_arr[3] < olddata_arr[3]) {
    // int step_u = abs(data_arr[0] - olddata_arr[0]) / move_step;
    olddata_arr[3] = olddata_arr[3] - move_step;
  }

  // Serial.print(olddata_arr[0]);
  // Serial.print(" ");
  // Serial.print(olddata_arr[1]);
  // Serial.print(" ");
  // Serial.print(olddata_arr[2]);
  // Serial.print(" ");
  // Serial.println(olddata_arr[3]);

  rotate_plate.write(olddata_arr[0]);
  jonit_1.write(olddata_arr[1]);
  jonit_2.write(olddata_arr[2]);
  jonit_3.write(olddata_arr[3]);
  delay(20);
  newData = false;
}


void checkForNewData () {
    if (Serial.available() >= arr_len && newData == false) {
        byte inByte;
        for (byte n = 0; n < arr_len; n++) {
            ddata [n] = Serial.read();
        }
        while (Serial.available() > 0) { // now make sure there is no other data in the buffer
             byte dumpByte =  Serial.read();
             Serial.println(dumpByte);
        }
        newData = true;
    }
}