#include"LD06.h"
#include<HardwareSerial.h>
#include<math.h>

LD06::LD06(HardwareSerial& serialPort, int RX){
  this->serialPort= &serialPort;
  this->RX = RX;
  this->PWM = 0;
}

LD06::LD06(HardwareSerial& serialPort, int RX, int PWM){
  this->serialPort= &serialPort;
  this->RX = RX;
  this->PWM = PWM;
}

void LD06::setup(){
  serialPort->setRxBufferSize(4096);
  serialPort->begin(230400, SERIAL_8N1, this->RX, -1);
  if(PWM != 0)
    ledcAttach(PWM, 30000, 8);
}

bool LD06::duty(int value){
	return ledcWrite(PWM, value);
}

uint16_t LD06::L2Bendian(uint8_t lsb, uint8_t msb){
  return (msb << 8) | lsb;
}
int LD06::toIndex(float coordonate, float range, int grid){
    int map_coordonate = (int)((coordonate + range/2.0) * (grid/range));
    return constrain(map_coordonate, 0, grid - 1);
}


std::vector<std::vector<uint8_t>> LD06::getMap(int RANGE, int MAP_SIZE){
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
                float radians = angle * M_PI / 180.0;
              	float x = frame.point[i].distance * cos(radians);
              	float y = frame.point[i].distance * sin(radians);

              	int x_map = LD06::toIndex(x, RANGE, MAP_SIZE);
              	int y_map = LD06::toIndex(y, RANGE, MAP_SIZE);

              	map[y_map][x_map] = 1;
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

std::vector<std::vector<uint8_t>> LD06::getMap(int RANGE, int MAP_SIZE, float START_ANGLE, float END_ANGLE){
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




