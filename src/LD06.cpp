#include "LD06.h"
#include <HardwareSerial.h>
#include <math.h>

const uint8_t LD06::CrcTable[256] = {                                                                      // Hexadecimal checksum values stored
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
  0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
  0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
  0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
  0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
  0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
  0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
  0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
  0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
  0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
  0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
  0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
  0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
  0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
  0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
  0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
  0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
  0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
  0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
  0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
  0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
  0x7f, 0x32, 0xe5, 0xa8
};

uint8_t LD06::CalCRC(uint8_t* FRAME_START, uint8_t LENGTH){                                                // Checksum calculator based on datasheet
  uint8_t crc = 0;
  uint16_t i;
  for (i = 0; i < LENGTH; i++)
  {
    crc = CrcTable[(crc ^ *FRAME_START++) & 0xff];
  }
  return crc;
}

LD06::LD06(HardwareSerial& serialPort, int RX){ 														   // Lidar object constructor taking serial port and receive pin
  this->serialPort= &serialPort;
  this->RX = RX;
  this->PWM = 0;
}

LD06::LD06(HardwareSerial& serialPort, int RX, int PWM){ 												   // Constructor overloading with PWM pin
  this->serialPort= &serialPort;
  this->RX = RX;
  this->PWM = PWM;
}

void LD06::setup(){ 																						// Setup function to be called in void setup()
  serialPort->setRxBufferSize(4096);  																		// Setting a large enough serial buffer for storing 50 lidar frames ( 47 bytes each)
  serialPort->begin(230400, SERIAL_8N1, this->RX, -1); 													    //  Begining Serial tranmission
  if(PWM != 0)																							    // Attaching PWM pin if exits
    ledcAttach(PWM, 30000, 8);
}

bool LD06::duty(int value){
	return ledcWrite(PWM, value); 																			// Set a duty cicle - controlling lidar motor speed ( default is the most stable anyway)
}

uint16_t LD06::L2Bendian(uint8_t lsb, uint8_t msb){
  return (msb << 8) | lsb; 																					// Converting from Little to Big Endian ( 0000000011111111 -> 1111111100000000)
}

int LD06::toIndex(float COORDONATE, float RANGE, int GRID_SIZE){
    int map_coordonate = (int)((COORDONATE + RANGE) * (GRID_SIZE / (RANGE*2) )); 							// Normalizing coordonate and converting to matrix index

    if(map_coordonate > GRID_SIZE || map_coordonate < 0){                // The indexes that outrange the grid size are returned as -1
      return -1;
    }
    return map_coordonate;
}

//std::vector<IndexedPoint> LD06::IndexedPointsContainer(){
//	return points_container = std::vector<IndexedPoint>;
//}

std::vector<std::vector<uint8_t>> LD06::getMap(int RANGE, int MAP_SIZE){
    std::vector<std::vector<uint8_t>> map(MAP_SIZE, std::vector<uint8_t>(MAP_SIZE,0));					    // Initialize the matrix of points where you can dinamically set the size unlike an array
    int count = 50; 																						// Number of frames to be analyzed
    while(serialPort->available() < FRAME_SIZE * 51) 														// The wanted byte is the HEADER byte which cand be anywhere from first to 47th byte read
      delay(100);                                   														// and after that we are getting frame after frame (47 bytes)(47 bytes)...

    uint8_t buffer[FRAME_SIZE * 51];																		// Settng a buffer for the frames form serial buffer (reading them one by one will slow the procces)
    serialPort->readBytes(buffer, FRAME_SIZE * 51);

    for(int k=0; k< FRAME_SIZE * 50; k++){
    	if(buffer[k] == FRAME_HEADER){ 																		// Verifying the header for each frame otherwise something can happen (data loss) and the reading procces will be desycronized
       		LidarFrame frame; 																				// Breaking the frame into pieces
            frame.header = buffer[k];
            frame.data_length = buffer[k + 1];
            frame.speed = LD06::L2Bendian(buffer[k + 2], buffer[k + 3]);
            frame.start_angle = LD06::L2Bendian(buffer[k + 4], buffer[k + 5]);
            for(int i=0; i < POINTS_PER_FRAME; i++){ 														// 12 points per frame (harware bounding)
              frame.point[i].distance = LD06::L2Bendian(buffer[(k + 6) + i * 3], buffer[(k + 7) + i * 3]);
              frame.point[i].confidence = buffer[(k + 8) + i * 3];
            }
            frame.end_angle = LD06::L2Bendian(buffer[k + 42], buffer[k + 43]);
            frame.timestamp = LD06::L2Bendian(buffer[k + 44], buffer[k + 45]);
            frame.checksum = buffer[k + 46];

            uint8_t calculatedCRC = LD06::CalCRC(&buffer[k],FRAME_SIZE-1);                                  // Calculating the checksum of the frame

            if(calculatedCRC == frame.checksum){                                                            // Verifying the integrity of the frame

                float startangle = frame.start_angle / 100.0; 												//  Converting to real angles ( Lidar is woring with integers)
                float endangle = frame.end_angle / 100.0;
                float step = ( (endangle < startangle ? endangle + 360.0 : endangle) - startangle) / (POINTS_PER_FRAME - 1); // Calcuating the step and verifying (350 to 20 typeof degrees cases)

                for(int i=0; i < POINTS_PER_FRAME; i++){
                   if(frame.point[i].confidence > 200){ 													// Minimum confidence for point to be considered
                     float angle = startangle + step * i; 													// Calculating each point angle based on start angle and point number
                     float radians = angle * M_PI / 180.0; 													// Calculating radian angle
                     float x = frame.point[i].distance * cos(radians); 										// Getting plan coordonates from point distance and angle
                     float y = frame.point[i].distance * sin(radians);

                     int x_map = LD06::toIndex(x, RANGE, MAP_SIZE); 									    // Conveting to 0-MAP_SIZE coordonate
                     int y_map = LD06::toIndex(y, RANGE, MAP_SIZE);
					 if(x_map != -1 && y_map != -1){														// Verifying the case when index is returnes as -1 because it outrange the grid
                     	map[y_map][x_map] = 1; 																// Building bitmap
                     }
                   }
                }
            }
            count--; 							 															// One more frame analyzed
            k = k + 46; 																					// Jumpimg ahead to the next frame's header (Keep in mind frames are separated by a separation byte)
        }

        if(count == 0) 																						// Stopping when enough frames were analyzed
          break;
    }
    return map;
}

std::vector<std::vector<uint8_t>> LD06::getMap(int RANGE, int MAP_SIZE, float START_ANGLE, float END_ANGLE){// Overloading getMap function, adding angle focusing
    std::vector<std::vector<uint8_t>> map(MAP_SIZE, std::vector<uint8_t>(MAP_SIZE,0));
    int count = 50;
    while(serialPort->available() < FRAME_SIZE * 51)
      delay(100);

    uint8_t buffer[FRAME_SIZE * 51];
    serialPort->readBytes(buffer, FRAME_SIZE * 51);

    for(int k=0; k< FRAME_SIZE * 50; k++){
    	if(buffer[k] == FRAME_HEADER){
       		LidarFrame frame;
            frame.header = buffer[k];
            frame.data_length = buffer[k + 1];
            frame.speed = LD06::L2Bendian(buffer[k + 2], buffer[k + 3]);
            frame.start_angle = LD06::L2Bendian(buffer[k + 4], buffer[k + 5]);
            for(int i=0; i < POINTS_PER_FRAME; i++){
              frame.point[i].distance = LD06::L2Bendian(buffer[(k + 6) + i * 3], buffer[(k + 7) + i * 3]);
              frame.point[i].confidence = buffer[(k + 8) + i * 3];
            }
            frame.end_angle = LD06::L2Bendian(buffer[k + 42], buffer[k + 43]);
            frame.timestamp = LD06::L2Bendian(buffer[k + 44], buffer[k + 45]);
            frame.checksum = buffer[k + 46];

        	uint8_t calculatedCRC = LD06::CalCRC(&buffer[k],FRAME_SIZE-1);

            if(calculatedCRC == frame.checksum){

                float startangle = frame.start_angle / 100.0;
                float endangle = frame.end_angle / 100.0;
                float step = ( (endangle < startangle ? endangle + 360.0 : endangle) - startangle) / (POINTS_PER_FRAME - 1);

                for(int i=0; i < POINTS_PER_FRAME; i++){
                    if(frame.point[i].confidence > 200){
                      float angle = startangle + step * i;
                      if( (START_ANGLE < END_ANGLE && START_ANGLE <= angle && angle <= END_ANGLE) || (START_ANGLE > END_ANGLE && (START_ANGLE <= angle || angle <= END_ANGLE))){
                          float radians = angle * M_PI / 180.0;
                          float x = frame.point[i].distance * cos(radians);
                          float y = frame.point[i].distance * sin(radians);

                          int x_map = LD06::toIndex(x, RANGE, MAP_SIZE);
                          int y_map = LD06::toIndex(y, RANGE, MAP_SIZE);
						  if(x_map != -1 && y_map != -1){
	                          map[y_map][x_map] = 1;
                          }
                      }
                    }
                }
            }

            count--;
            k = k + 46;
        }

        if(count == 0)
          break;
    }
    return map;
}

std::vector<IndexedPoint> LD06::getIndexedPoints(int RANGE, int MAP_SIZE){									// Method to get Points along with their coordonates for a choosen grid size
  																											// along with their distance(mm)
  	int count = 50;
    while(serialPort->available() < FRAME_SIZE * 51)
      delay(50);

    std::vector<IndexedPoint> points;

    uint8_t buffer[FRAME_SIZE * 51];
    serialPort->readBytes(buffer, FRAME_SIZE * 51);

    for(int k=0; k< FRAME_SIZE * 50; k++){
    	if(buffer[k] == FRAME_HEADER){
       		LidarFrame frame;
            frame.header = buffer[k];
            frame.data_length = buffer[k + 1];
            frame.speed = LD06::L2Bendian(buffer[k + 2], buffer[k + 3]);
            frame.start_angle = LD06::L2Bendian(buffer[k + 4], buffer[k + 5]);
            for(int i=0; i < POINTS_PER_FRAME; i++){
              frame.point[i].distance = LD06::L2Bendian(buffer[(k + 6) + i * 3], buffer[(k + 7) + i * 3]);
              frame.point[i].confidence = buffer[(k + 8) + i * 3];
            }
            frame.end_angle = LD06::L2Bendian(buffer[k + 42], buffer[k + 43]);
            frame.timestamp = LD06::L2Bendian(buffer[k + 44], buffer[k + 45]);
            frame.checksum = buffer[k + 46];

        	uint8_t calculatedCRC = LD06::CalCRC(&buffer[k],FRAME_SIZE-1);

            if(calculatedCRC == frame.checksum){

                float startangle = frame.start_angle / 100.0;
                float endangle = frame.end_angle / 100.0;
                float step = ( (endangle < startangle ? endangle + 360.0 : endangle) - startangle) / (POINTS_PER_FRAME - 1);

                for(int i=0; i < POINTS_PER_FRAME; i++){
                    if(frame.point[i].confidence > 200){
                      float angle = startangle + step * i;
                      float radians = angle * M_PI / 180.0;
                      float x = frame.point[i].distance * cos(radians);
                      float y = frame.point[i].distance * sin(radians);

                      int x_map = LD06::toIndex(x, RANGE, MAP_SIZE);
                      int y_map = LD06::toIndex(y, RANGE, MAP_SIZE);
					  if(x_map != -1 && y_map != -1){
                      	points.push_back({x_map, y_map, frame.point[i].distance});							// Populating the vector with the IndexedPoint stuctures
                      }
                    }
                }
            }

            count--;
            k = k + 46;
        }

        if(count == 0)
          break;
    }
    return points;
}

std::vector<IndexedPoint> LD06::getIndexedPointsLite(int RANGE, int MAP_SIZE){									// Method to get Points along with their coordonates for a choosen grid size
  																											// along with their distance(mm)
  	int count = 30;
    while(serialPort->available() < FRAME_SIZE * 31)
      delay(50);

    std::vector<IndexedPoint> points;

    uint8_t buffer[FRAME_SIZE * 31];
    serialPort->readBytes(buffer, FRAME_SIZE * 31);

    for(int k=0; k< FRAME_SIZE * 30; k++){
    	if(buffer[k] == FRAME_HEADER){
       		LidarFrame frame;
            frame.header = buffer[k];
            frame.data_length = buffer[k + 1];
            frame.speed = LD06::L2Bendian(buffer[k + 2], buffer[k + 3]);
            frame.start_angle = LD06::L2Bendian(buffer[k + 4], buffer[k + 5]);
            for(int i=0; i < POINTS_PER_FRAME; i++){
              frame.point[i].distance = LD06::L2Bendian(buffer[(k + 6) + i * 3], buffer[(k + 7) + i * 3]);
              frame.point[i].confidence = buffer[(k + 8) + i * 3];
            }
            frame.end_angle = LD06::L2Bendian(buffer[k + 42], buffer[k + 43]);
            frame.timestamp = LD06::L2Bendian(buffer[k + 44], buffer[k + 45]);
            frame.checksum = buffer[k + 46];

        	uint8_t calculatedCRC = LD06::CalCRC(&buffer[k],FRAME_SIZE-1);

            if(calculatedCRC == frame.checksum){

                float startangle = frame.start_angle / 100.0;
                float endangle = frame.end_angle / 100.0;
                float step = ( (endangle < startangle ? endangle + 360.0 : endangle) - startangle) / (POINTS_PER_FRAME - 1);

                for(int i=0; i < POINTS_PER_FRAME; i++){
                    if(frame.point[i].confidence > 200){
                      float angle = startangle + step * i;
                      float radians = angle * M_PI / 180.0;
                      float x = frame.point[i].distance * cos(radians);
                      float y = frame.point[i].distance * sin(radians);

                      int x_map = LD06::toIndex(x, RANGE, MAP_SIZE);
                      int y_map = LD06::toIndex(y, RANGE, MAP_SIZE);
					  if(x_map != -1 && y_map != -1){
                      	points.push_back({x_map, y_map, frame.point[i].distance});							// Populating the vector with the IndexedPoint stuctures
                      }
                    }
                }
            }

            count--;
            k = k + 46;
        }

        if(count == 0)
          break;
    }
    return points;
}


std::vector<RawPoint> LD06::getRawPoints(int RANGE){														// Method to get basic data for Points such as angle(degrees) and distance(mm)
  	int count = 50;
    while(serialPort->available() < FRAME_SIZE * 51)
      delay(50);

    std::vector<RawPoint> points;

    uint8_t buffer[FRAME_SIZE * 51];
    serialPort->readBytes(buffer, FRAME_SIZE * 51);

    for(int k=0; k< FRAME_SIZE * 50; k++){
    	if(buffer[k] == FRAME_HEADER){
       		LidarFrame frame;
            frame.header = buffer[k];
            frame.data_length = buffer[k + 1];
            frame.speed = LD06::L2Bendian(buffer[k + 2], buffer[k + 3]);
            frame.start_angle = LD06::L2Bendian(buffer[k + 4], buffer[k + 5]);
            for(int i=0; i < POINTS_PER_FRAME; i++){
              frame.point[i].distance = LD06::L2Bendian(buffer[(k + 6) + i * 3], buffer[(k + 7) + i * 3]);
              frame.point[i].confidence = buffer[(k + 8) + i * 3];
            }
            frame.end_angle = LD06::L2Bendian(buffer[k + 42], buffer[k + 43]);
            frame.timestamp = LD06::L2Bendian(buffer[k + 44], buffer[k + 45]);
            frame.checksum = buffer[k + 46];

        	uint8_t calculatedCRC = LD06::CalCRC(&buffer[k],FRAME_SIZE-1);

            if(calculatedCRC == frame.checksum){

                float startangle = frame.start_angle / 100.0;
                float endangle = frame.end_angle / 100.0;
                float step = ( (endangle < startangle ? endangle + 360.0 : endangle) - startangle) / (POINTS_PER_FRAME - 1);

                for(int i=0; i < POINTS_PER_FRAME; i++){
                    if(frame.point[i].confidence > 200 && frame.point[i].distance <= RANGE/2){
                      float angle = startangle + step * i;
                      points.push_back({angle, frame.point[i].distance});									// Populating the vector with IndexedPoint structures
                    }
                }
            }

            count--;
            k = k + 46;
        }

        if(count == 0)
          break;
    }
    return points;
}

std::vector<RawPoint> LD06::getRawPointsLite(int RANGE){														// Method to get basic data for Points such as angle(degrees) and distance(mm)
  	int count = 30;
    while(serialPort->available() < FRAME_SIZE * 31)
      delay(50);

    std::vector<RawPoint> points;

    uint8_t buffer[FRAME_SIZE * 31];
    serialPort->readBytes(buffer, FRAME_SIZE * 31);

    for(int k=0; k< FRAME_SIZE * 30; k++){
    	if(buffer[k] == FRAME_HEADER){
       		LidarFrame frame;
            frame.header = buffer[k];
            frame.data_length = buffer[k + 1];
            frame.speed = LD06::L2Bendian(buffer[k + 2], buffer[k + 3]);
            frame.start_angle = LD06::L2Bendian(buffer[k + 4], buffer[k + 5]);
            for(int i=0; i < POINTS_PER_FRAME; i++){
              frame.point[i].distance = LD06::L2Bendian(buffer[(k + 6) + i * 3], buffer[(k + 7) + i * 3]);
              frame.point[i].confidence = buffer[(k + 8) + i * 3];
            }
            frame.end_angle = LD06::L2Bendian(buffer[k + 42], buffer[k + 43]);
            frame.timestamp = LD06::L2Bendian(buffer[k + 44], buffer[k + 45]);
            frame.checksum = buffer[k + 46];

        	uint8_t calculatedCRC = LD06::CalCRC(&buffer[k],FRAME_SIZE-1);

            if(calculatedCRC == frame.checksum){

                float startangle = frame.start_angle / 100.0;
                float endangle = frame.end_angle / 100.0;
                float step = ( (endangle < startangle ? endangle + 360.0 : endangle) - startangle) / (POINTS_PER_FRAME - 1);

                for(int i=0; i < POINTS_PER_FRAME; i++){
                    if(frame.point[i].confidence > 200 && frame.point[i].distance <= RANGE/2){
                      float angle = startangle + step * i;
                      points.push_back({angle, frame.point[i].distance});									// Populating the vector with IndexedPoint structures
                    }
                }
            }

            count--;
            k = k + 46;
        }

        if(count == 0)
          break;
    }
    return points;
}

//void LD06::dinamicallyMap(std::unordered_set<Point> points, int RANGE, int GRID_SIZE){
//
//  	int count = 50;
//    while(serialPort->available() < FRAME_SIZE * 51)
//      delay(100);
//
//    uint8_t buffer[FRAME_SIZE * 51];
//    serialPort->readBytes(buffer, FRAME_SIZE * 51);
//
//    for(int k=0; k< FRAME_SIZE * 50; k++){
//    	if(buffer[k] == FRAME_HEADER){
//       		LidarFrame frame;
//            frame.header = buffer[k];
//            frame.data_length = buffer[k + 1];
//            frame.speed = LD06::L2Bendian(buffer[k + 2], buffer[k + 3]);
//            frame.start_angle = LD06::L2Bendian(buffer[k + 4], buffer[k + 5]);
//            for(int i=0; i < POINTS_PER_FRAME; i++){
//              frame.point[i].distance = LD06::L2Bendian(buffer[(k + 6) + i * 3], buffer[(k + 7) + i * 3]);
//              frame.point[i].confidence = buffer[(k + 8) + i * 3];
//            }
//            frame.end_angle = LD06::L2Bendian(buffer[k + 42], buffer[k + 43]);
//            frame.timestamp = LD06::L2Bendian(buffer[k + 44], buffer[k + 45]);
//            frame.checksum = buffer[k + 46];
//
//        	uint8_t calculatedCRC = LD06::CalCRC(&buffer[k],FRAME_SIZE-1);
//
//            if(calculatedCRC == frame.checksum){
//
//                float startangle = frame.start_angle / 100.0;
//                float endangle = frame.end_angle / 100.0;
//                float step = ( (endangle < startangle ? endangle + 360.0 : endangle) - startangle) / (POINTS_PER_FRAME - 1);
//
//                for(int i=0; i < POINTS_PER_FRAME; i++){
//                    if(frame.point[i].confidence > 200){
//                      float angle = startangle + step * i;
//                      float radians = angle * M_PI / 180.0;
//                      float x = frame.point[i].distance * cos(radians);
//                      float y = frame.point[i].distance * sin(radians);
//
//                      int x_map = LD06::toIndex(x, RANGE, MAP_SIZE);
//                      int y_map = LD06::toIndex(y, RANGE, MAP_SIZE);
//					  if(x_map != -1 && y_map != -1){
//                      	points.push_back({x_map, y_map});
//                    }
//                }
//            }
//
//            count--;
//            k = k + 46;
//        }
//
//        if(count == 0)
//          break;
//    }
//    return points;
//}




