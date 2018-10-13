#include "MPU9250.h"
#include <ArduinoSort.h>
#include <Servo.h>


MPU9250 IMU(Wire,0x68);
int status;

Servo canard[4];

int canard_pin[] = {A1, A2, A3, A4};

float offset_x, offset_y;
float angle_x, angle_y;



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);

  for (uint8_t i = 0; i < 4; i++)
  {
    canard[i].attach(canard_pin[i]);
    canard[i].write(90);
  }

  status = IMU.begin();
  if (status < 0)
   blink_led();
  
  offset_x = getDriftX();
  offset_y = getDriftY();

  
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU.readSensor();

  angle_x = atan(IMU.getAccelZ_mss()/sqrt((IMU.getAccelX_mss()*IMU.getAccelX_mss()) + (IMU.getAccelY_mss()*IMU.getAccelY_mss())));
  angle_y = atan(IMU.getAccelX_mss()/sqrt((IMU.getAccelZ_mss()*IMU.getAccelZ_mss()) + (IMU.getAccelY_mss()*IMU.getAccelY_mss())));
  float x_angle = ((angle_x - offset_x) * (180 / PI) + 90);
  float y_angle = ((angle_y - offset_y) * (180 / PI) + 90);
  
  canard[0].write(map(180 - y_angle, 0, 180, 55, 125));
  canard[1].write(map(x_angle, 0, 180, 55, 125));
  canard[2].write(map(y_angle, 0, 180, 55, 125));
  canard[3].write(map(180 - x_angle, 0, 180, 55, 125));

  Serial.print("Servo 1 : ");
  Serial.print(canard[0].read());
  Serial.print("\tServo 2 : ");
  Serial.print(canard[1].read());
  Serial.print("\tServo 3 : ");
  Serial.print(canard[2].read());
  Serial.print("\tServo 4 : ");
  Serial.println(canard[3].read());
}

double getDriftX()
{
  IMU.readSensor();
  float _array[200];
  for (uint8_t i = 0; i < 200; i++)
    _array[i] = atan(IMU.getAccelZ_mss()/sqrt((IMU.getAccelX_mss()*IMU.getAccelX_mss()) + (IMU.getAccelY_mss()*IMU.getAccelY_mss())));
  sortArray(_array, 200);
  return _array[100];
}

double getDriftY()
{
  IMU.readSensor();
  float _array[200];
  for (uint8_t i = 0; i < 200; i++)
    _array[i] = atan(IMU.getAccelX_mss()/sqrt((IMU.getAccelZ_mss()*IMU.getAccelZ_mss()) + (IMU.getAccelY_mss()*IMU.getAccelY_mss())));
  sortArray(_array, 200);
  return _array[100];
}

static void blink_led()
{
  while(true)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(2000);
  }
}
