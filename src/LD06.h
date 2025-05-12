#ifndef LD06_H
#define LD06_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <array>
#include <vector>

#define POINTS_PER_FRAME 12
#define FRAME_HEADER 0x54
#define FRAME_SIZE 47


typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t data_length;
  uint16_t speed;
  uint16_t start_angle;
  struct{
    uint16_t distance;
    uint8_t confidence;
  } point[POINTS_PER_FRAME];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t checksum;
}LidarFrame;



class LD06{
  public:
    //Inainte trb creat HardwareSerial serialPort(2) si pus in functie ca argument
    LD06(HardwareSerial& serialPort, int RX);
    LD06(HardwareSerial& serialPort, int RX, int PWM);
    void setup();
    bool duty(int value);
    std::vector<std::vector<uint8_t>> getMap(int RANGE, int MAP_SIZE);
    std::vector<std::vector<uint8_t>> getMap(int RANGE, int MAP_SIZE, float START_ANGLE, float END_ANGLE);
    //int getDist(int START_ANGLE, int END_ANGLE, std::vector<std::vector<uint8_t>> map, int RANGE);



  private:
    //Variables
    HardwareSerial* serialPort;
    int RX;
    int PWM;
    //Methods
    uint16_t L2Bendian(uint8_t lsb, uint8_t msb);
    int toIndex(float coordonate, float RANGE, int MAP_SIZE);
};

#endif //LD06_H
