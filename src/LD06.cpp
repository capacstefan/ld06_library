#include"LD06.h"
#include<HardwareSerial.h>
#include<math.h>

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
int LD06::toIndex(float coordonate, float range, int grid){
    int map_coordonate = (int)((coordonate + range/2.0) * (grid/range)); 									// Normalizing coordonate and converting to matrix index
    return constrain(map_coordonate, 0, grid - 1); 															// Returnng the coordoante bounded by the matrix size
}

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

            float startangle = frame.start_angle / 100.0; 													//  Converting to real angles ( Lidar is woring with integers)
            float endangle = frame.end_angle / 100.0;
            float step = ( (endangle < startangle ? endangle + 360.0 : endangle) - startangle) / (POINTS_PER_FRAME - 1); // Calcuating the step and verifying (350 to 20 typeof degrees cases)

            for(int i=0; i < POINTS_PER_FRAME; i++){
              if(frame.point[i].confidence > 200){ 															// Minimum confidence for point to be considered
                float angle = startangle + step * i; 														// Calculating each point angle based on start angle and point number
                float radians = angle * M_PI / 180.0; 														// Calculating radian angle
              	float x = frame.point[i].distance * cos(radians); 											// Getting plan coordonates from point distance and angle
              	float y = frame.point[i].distance * sin(radians);

              	int x_map = LD06::toIndex(x, RANGE, MAP_SIZE); 												// Conveting to 0-MAP_SIZE coordonate
              	int y_map = LD06::toIndex(y, RANGE, MAP_SIZE);

              	map[y_map][x_map] = 1; 																	 	// Building bitmap
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

              	  map[y_map][x_map] = 1;
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




