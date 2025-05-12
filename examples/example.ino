#include <HardwareSerial.h>
#include <math.h> // Pentru fmod() si M_PI

#define RX 3
// #define PWM 25
// #define resolution 8 // 8 biti 0-255
// #define channel 0 // 0-16 canale
// #define freq 30000 // 30khz=30000Hz
#define POINTS_PER_FRAME 12
#define HEADER 0x54
#define MAX_RANGE 3000 // Presupunem o distanță maximă de 5000 mm

HardwareSerial mySerial(2);

typedef struct __attribute__((packed)) { // pentru a nu se adauga padding
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  struct {
    uint16_t distance;
    uint8_t confidence;
  } point[POINTS_PER_FRAME];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

// Declaram matricea pentru a stoca punctele
volatile uint8_t gridMap[64][64] = {0}; // Volatile pentru a fi siguri că modificările sunt vizibile în afara ISR-urilor (dacă ar fi cazul)
volatile int count = 15;
uint16_t littleEndianToBigEndian(uint8_t lsb, uint8_t msb) {
  return (msb << 8) | lsb; // << muta la stanga n pozitii, | concateneaza
}

int toIndex(float coordonate, float range, int grid){
  int grid_coordonate = (int)((coordonate + range/2.0) * (grid/range));
  return constrain(grid_coordonate, 0, grid - 1);
}

void setup() {
  Serial.begin(230400);
  mySerial.begin(230400, SERIAL_8N1, RX, -1);
}

void loop() {
  if(mySerial.available() >= sizeof(LiDARFrameTypeDef)) {
    uint8_t buffer[sizeof(LiDARFrameTypeDef)]; // cream un vector tip bytes de dimensiune corespunzatare dimensiunii frame-ului
    mySerial.readBytes(buffer, sizeof(LiDARFrameTypeDef));
    if (buffer[0] == HEADER && count > 0) {
      LiDARFrameTypeDef frame;
      frame.header = buffer[0];
      frame.ver_len = buffer[1];
      frame.speed = littleEndianToBigEndian(buffer[2], buffer[3]);
      frame.start_angle = littleEndianToBigEndian(buffer[4], buffer[5]);

      // Gestionare 12 masurători
      for (int i = 0; i < POINTS_PER_FRAME; i++) {
        frame.point[i].distance = littleEndianToBigEndian(buffer[6 + i * 3], buffer[7 + i * 3]);
        // Calcul buffer[...] - in functie de la al catelea punct masurat suntem(fiecare punct foloseste 3 bytes)
        frame.point[i].confidence = buffer[8 + i * 3];
      }

      frame.end_angle = littleEndianToBigEndian(buffer[42], buffer[43]);
      frame.timestamp = littleEndianToBigEndian(buffer[44], buffer[45]);
      frame.crc8 = buffer[46];
      float start_angle_deg = frame.start_angle / 100.0;
      float end_angle_deg = frame.end_angle / 100.0;
      float step_deg = (end_angle_deg - start_angle_deg) / (POINTS_PER_FRAME - 1);

      for (int i = 0; i < POINTS_PER_FRAME; i++) {
        float angle_deg = fmod(start_angle_deg + step_deg * i, 360.0);
        float distance_mm = frame.point[i].distance;

        // Calcul coordonate carteziene (aproximative)
        float angle_rad = angle_deg * M_PI / 180.0;
        float x = distance_mm * cos(angle_rad);
        float y = distance_mm * sin(angle_rad);

        // Convertim coordonatele carteziene in indici in matrice
        int x_index = toIndex(x, MAX_RANGE, 64);
        int y_index = toIndex(y, MAX_RANGE, 64);

        // Verificam daca indicii sunt in limitele matricei
        // if (x_index >= 0 && x_index < 64 && y_index >= 0 && y_index < 64) {
        //   gridMap[x_index][y_index] = 1; // Marcam celula ca fiind "ocupata" (puteti folosi increderea sau distanta aici)
        // }

        Serial.print("Unghi: "); Serial.print(angle_deg);
        Serial.print("°, Distanta: "); Serial.print(distance_mm);
        Serial.print(" mm, Confidenta: "); Serial.print(frame.point[i].confidence);
        Serial.print(", x_index: "); Serial.print(x_index);
        Serial.print(", y_index: "); Serial.println(y_index);
        gridMap[x_index][y_index] = 1;
      }
      // Acum, matricea gridMap a fost populată cu datele din acest frame LiDAR.
      // Puteți procesa sau afișa matricea aici.
      // Serial.println("--- Matricea GRID ---");
      // for (int i = 0; i < GRID_SIZE; i++) {
      //   for (int j = 0; j < GRID_SIZE; j++) {
      //     Serial.print(gridMap[j][i]); // Afisam cu j inainte pentru a corespunde cu x si i cu y
      //   }
      //   Serial.println();
      // }
      // Optional: Resetati matricea pentru următorul frame, dacă doriți să vizualizați doar un singur frame la o dată.
      // memset(gridMap, 0, sizeof(gridMap));
      count--;
    }
  }
  // Nu puneți un while aici care să blocheze loop-ul.
  // loop() se execută continuu, verificând dacă sunt date disponibile de la serial.
  //delay(100); // Adăugați o mică întârziere pentru a nu suprasolicita procesorul (opțional)
  if(count == 0){
    Serial.println("--- Matricea GRID ---");
    for (int i = 0; i < 64; i++) {
      for (int j = 0; j < 64; j++)
        Serial.print(gridMap[i][j]);
      Serial.println();
    }
    count = -1;
  }
}