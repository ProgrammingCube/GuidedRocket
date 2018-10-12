#include "MPU9250.h"
#include <PID_v1.h>
#include <ArduinoSort.h>
#include <Servo.h>


MPU9250 IMU(Wire,0x68);
int status;

Servo canard[4];

uint8_t canard_pin[] = {2, 3, 4, 5};

float offset_x, offset_y;
float angle_x, angle_y;



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  delay(3000);

  for (uint8_t i = 0; i < 4; i++)
  {
    canard[i].attach(canard_pin[i]);
    canard[i].write(90);
  }

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  
  offset_x = getDriftX();
  offset_y = getDriftY();

  
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU.readSensor();

  angle_x = atan(IMU.getAccelY_mss()/sqrt((IMU.getAccelX_mss()*IMU.getAccelX_mss()) + (IMU.getAccelZ_mss()*IMU.getAccelZ_mss())));
  angle_y = atan(IMU.getAccelX_mss()/sqrt((IMU.getAccelY_mss()*IMU.getAccelY_mss()) + (IMU.getAccelZ_mss()*IMU.getAccelZ_mss())));
  float x_angle = ((angle_x - offset_x) * (180 / PI) + 90);
  float y_angle = ((angle_y - offset_y) * (180 / PI) + 90);
  //Serial.print("X Angle: ");
  //Serial.println(angle_x - offset_x);
  canard[1].write(180 - x_angle);
  canard[3].write(x_angle);
  canard[0].write(y_angle);
  canard[2].write(180 - y_angle);
}

double getDriftX()
{
  IMU.readSensor();
  float _array[200];
  //double y_array[200];
  for (uint8_t i = 0; i < 200; i++)
    _array[i] = atan(IMU.getAccelY_mss()/sqrt((IMU.getAccelX_mss()*IMU.getAccelX_mss()) + (IMU.getAccelZ_mss()*IMU.getAccelZ_mss())));
  sortArray(_array, 200);
  return _array[100];
}

double getDriftY()
{
  IMU.readSensor();
  float _array[200];
  for (uint8_t i = 0; i < 200; i++)
    _array[i] = atan(IMU.getAccelX_mss()/sqrt((IMU.getAccelY_mss()*IMU.getAccelY_mss()) + (IMU.getAccelZ_mss()*IMU.getAccelZ_mss())));
  sortArray(_array, 200);
  return _array[100];
}
