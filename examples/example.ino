#include <LD06.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//////////////////////////
HardwareSerial mySerial(2);
LD06 lidar(mySerial,3);
//////////////////////////

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup(){
  ////////////////
  lidar.setup();
  ////////////////

  Serial.begin(230400);
  Wire.begin(21,22); // SDA, SCL
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();


}

void loop(){
  //////////////////////////////////////////////
  auto points = lidar.getPoints(2000,63);
  //////////////////////////////////////////////

  display.clearDisplay();

  for(int i = 0; i < points.size(); i++){
    display.drawPixel(points[i].x, points[i].y, SSD1306_WHITE);
  }

  display.display();
}
