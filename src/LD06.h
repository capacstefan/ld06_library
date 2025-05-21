#ifndef LD06_H
#define LD06_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <vector>
#include <unordered_set>

#define POINTS_PER_FRAME 12                                                                       // Each lidar frame has 12 frames
#define FRAME_HEADER 0x54                                                                         // Each frame's first byte is 0x54
#define FRAME_SIZE 47                                                                             // Each frame has 47 bytes long

struct IndexedPoint {																			  // IndexedPoint structure for storing collected information
  int x;																						  // Using int because there will be big values
  int y;
  int distance;
};

struct RawPoint {                                                                                 // RawPoint structure for storing angle and distance
  float angle;                                                                                    // The angle will take non-integer values (180.35)
  int distance;
};

struct Point {
  int x;
  int y;
  bool operator==(const Point& other) const{
    return x == other.x && y == other.y;
  }
};

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
    std::vector<std::vector<uint8_t>> getMap(int RANGE, int MAP_SIZE);                             // Building a grid using the points read
    std::vector<std::vector<uint8_t>> getMap(int RANGE, int MAP_SIZE, float START_ANGLE, float END_ANGLE); // Building a grid using the points read between certain degrees
  	std::vector<IndexedPoint> getIndexedPoints(int RANGE, int MAP_SIZE);						   // Returning a vector with the points read, indexed for a certain grid size (FASTER THAN WHOLE GRID METHOD)
    std::vector<RawPoint> getRawPoints(int RANGE);												   // Returning a vector with the points read, along with their raw data (angle, distance)
//    std::vector<IndexedPoint> IndexedPointsContainer();											   // Initializing a container for IndexedPoints
//    void dynamicallyMap(std::unordered_set<Point> points,int RANGE, int MAP_SIZE, int X, int Y); // To be defined, Based on current positon, filling a vector with points scanned for a all angle view map
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
