#ifndef LD06_H
#define LD06_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <vector>

#define POINTS_PER_FRAME 12                                                                       // Each lidar frame has 12 frames
#define FRAME_HEADER 0x54                                                                         // Each frame's first byte is 0x54
#define FRAME_SIZE 47                                                                             // Each frame has 47 bytes long

struct Point {																					  // Point structure for storing collected information
  int x;																						  // Using int because there will be big values
  int y;
  int distance;
}

struct __attribute__((packed)) LidarFrame{                                                        // Frame structure according to datasheet
  uint8_t header;                                                                                 // Using uint8_t because we are memory and cpu bounded
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
};



class LD06{                                                                                        // LD06 will be the class of the lidar
  public:

    LD06(HardwareSerial& serialPort, int RX);                                                      // Constuctor
    LD06(HardwareSerial& serialPort, int RX, int PWM);                                             // Overloaded Constructor
    void setup();                                                                                  // Setup function to be called in void setup()
    bool duty(int value);                                                                          // Setting motor speed
    std::vector<std::vector<uint8_t>> getMap(int RANGE, int MAP_SIZE);                             // Mapping the points read
    std::vector<std::vector<uint8_t>> getMap(int RANGE, int MAP_SIZE, float START_ANGLE, float END_ANGLE); // -//- in a certain angle
  	std::vector<Point> getPoints(int RANGE, int MAP_SIZE);										   // Obtaining a vector of Points (faster and smaller than an 2d Vector)
    // To be continued ...

  private:

    //Variables
    HardwareSerial* serialPort;                                                                    // Serial port to communicate
    int RX;                                                                                        // Serial Receive Pin
    int PWM;                                                                                       // Duty control Pin
    static const uint8_t CrcTable[256];
    //Methods
    uint16_t L2Bendian(uint8_t lsb, uint8_t msb);                                                  // Converting from Little to Big Endian ( 00000001111111 -> 111111110000000)
    int toIndex(float coordonate, float RANGE, int MAP_SIZE);                                      // Converting from plan coordonate to grid coordonate
    uint8_t CalCRC(uint8_t* FRAME_START, uint8_t LENGTH);
};

#endif //LD06_H
