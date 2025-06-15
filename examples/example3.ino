#include <LD06.h>
#include <ESP32Servo.h>

HardwareSerial mySerial(2);
LD06 lidar(mySerial,3);

Servo servo;
void setup() {
    Serial.begin(230400);
    lidar.setup();
    servo.attach(25);
}

void loop() {
    auto points = lidar.getRawPoints(2000);
    int angleOFclosest = 0;
    float minDistance = 100000;
    for(int i=0; i < points.size(); i++){
        int dist = points[i].distance / 10;
        if(dist < minDistance - 5){
            minDistance = dist;
            if(points[i].angle <= 180){
                angleOFclosest = 180 - (int)points[i].angle;
                servo.write(angleOFclosest);
            }
        }
    }
    delay(100);
}
