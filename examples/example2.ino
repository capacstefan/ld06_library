#include <WiFi.h>
#include <WebServer.h>
#include <LD06.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h> // Pentru mutex

HardwareSerial mySerial(2);
LD06 lidar(mySerial, 3);

WebServer server(80);

const char* ap_ssid = "ESP32_LIDAR_WEB";
const char* ap_password = "12345678";

std::vector<IndexedPoint> points;
TaskHandle_t lidarTaskHandle = NULL;

// Variabile partajate - le protejam cu mutex
volatile int maxLidarDistance = 800; // Valoare implicita pentru distanta
volatile int startAngle = 0;         // Valoare implicita pentru unghiul de start
volatile int endAngle = 360;         // Valoare implicita pentru unghiul de final

// Mutex pentru protectia variabilelor partajate
SemaphoreHandle_t xLidarParamsMutex;

// =================== Web Handlers ===================

void handleData() {
  String response = "[";
  // Nu e nevoie de mutex aici pentru ca "points" este scris doar de lidarTask
  // si citit doar de handleData. Daca lidarTask ar scrie si alte task-uri
  // ar citi sau scrie, ar fi nevoie de mutex. In cazul de fata, vectorul
  // "points" este actualizat atomic (referinta e schimbata).
  for (size_t i = 0; i < points.size(); i++) {
    response += "{\"x\":" + String(points[i].x) + ",\"y\":" + String(points[i].y) + "}";
    if (i < points.size() - 1) response += ",";
  }
  response += "]";
  server.send(200, "application/json", response);
}

void handleSetDistance() {
  if (server.hasArg("distance")) {
    int newDistance = server.arg("distance").toInt();
    if (newDistance > 0 && newDistance <= 10000) {
      // Achizitioneaza mutex-ul inainte de a modifica variabila partajata
      if (xSemaphoreTake(xLidarParamsMutex, portMAX_DELAY) == pdTRUE) {
        maxLidarDistance = newDistance;
        Serial.print("Distanta maxima LIDAR setata la: ");
        Serial.println(maxLidarDistance);
        xSemaphoreGive(xLidarParamsMutex); // Elibereaza mutex-ul
        server.send(200, "text/plain", "OK");
      } else {
        server.send(500, "text/plain", "Eroare la achizitia mutex-ului.");
      }
    } else {
      server.send(400, "text/plain", "Valoare invalida. Introdu un numar intre 1 si 10000.");
    }
  } else {
    server.send(400, "text/plain", "Parametru 'distance' lipsa.");
  }
}

void handleSetAngles() {
  if (server.hasArg("start") && server.hasArg("end")) {
    int newStartAngle = server.arg("start").toInt();
    int newEndAngle = server.arg("end").toInt();

    if (newStartAngle >= 0 && newStartAngle <= 360 &&
        newEndAngle >= 0 && newEndAngle <= 360 &&
        newStartAngle != newEndAngle) {
      // Achizitioneaza mutex-ul inainte de a modifica variabilele partajate
      if (xSemaphoreTake(xLidarParamsMutex, portMAX_DELAY) == pdTRUE) {
        startAngle = newStartAngle;
        endAngle = newEndAngle;
        Serial.printf("Unghiuri LIDAR setate: Start=%d, End=%d\n", startAngle, endAngle);
        xSemaphoreGive(xLidarParamsMutex); // Elibereaza mutex-ul
        server.send(200, "text/plain", "OK");
      } else {
        server.send(500, "text/plain", "Eroare la achizitia mutex-ului.");
      }
    } else {
      server.send(400, "text/plain", "Valori invalide. Unghiurile trebuie sa fie intre 0 si 360 si diferite.");
    }
  } else {
    server.send(400, "text/plain", "Parametrii 'start' sau 'end' lipsa.");
  }
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>LIDAR Live</title>
  <style>
    body {
      margin: 0;
      display: flex;
      flex-direction: column;
      align-items: center;
      background-color: #333;
      color: #FFF;
      padding-top: 3.5rem;
      height: 100vh;
      box-sizing: border-box;
    }

    .topbar {
      position: fixed;
      top: 0;
      width: 100%;
      height: 3rem;
      background-color: #222;
      color: #FFF;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.2rem;
      box-shadow: 0 2px 5px rgba(0,0,0,0.5);
      z-index: 10;
    }

    canvas {
      border: 3px solid #FFF;
      background-color: #333;
      margin-top: 1rem;
    }

    .controls {
      margin-top: 1rem;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }

    .control-group {
      display: flex;
      align-items: center;
      gap: 10px;
    }

    .controls input[type="number"] {
      width: 80px;
      padding: 5px;
    }

    .controls button {
      padding: 5px 10px;
    }
  </style>
  <script>
    async function setLidarDistance() {
      const distance = document.getElementById("distanceInput").value;
      try {
        const response = await fetch('/setDistance?distance=' + distance);
        if (response.ok) {
          console.log('Distanța a fost setată cu succes la ' + distance + ' mm.');
        } else {
          console.error('Eroare la setarea distantei:', response.statusText);
          alert('Eroare: ' + response.statusText);
        }
      } catch (error) {
        console.error('Eroare de fetch:', error);
        alert('Eroare de rețea.');
      }
    }

    async function setLidarAngles() {
      const start = document.getElementById("startAngleInput").value;
      const end = document.getElementById("endAngleInput").value;
      try {
        const response = await fetch('/setAngles?start=' + start + '&end=' + end);
        if (response.ok) {
          console.log('Unghiurile au fost setate cu succes: Start=' + start + ', End=' + end);
        } else {
          console.error('Eroare la setarea unghiurilor:', response.statusText);
          alert('Eroare: ' + response.statusText);
        }
      } catch (error) {
        console.error('Eroare de fetch:', error);
        alert('Eroare de rețea.');
      }
    }
  </script>
</head>
<body>
  <div class="topbar">LIDAR Live Viewer</div>
  <canvas id="canvas" width="512" height="512"></canvas>

  <div class="controls">
    <div class="control-group">
      <label for="distanceInput">Distance (mm):</label>
      <input type="number" id="distanceInput" value="800">
      <button onclick="setLidarDistance()">SET DISTANCE</button>
    </div>
    <div class="control-group">
      <label for="startAngleInput">Start Angle:</label>
      <input type="number" id="startAngleInput" value="0" min="0" max="360">
      <label for="endAngleInput">End Angle:</label>
      <input type="number" id="endAngleInput" value="360" min="0" max="360">
      <button onclick="setLidarAngles()">SET ANGLES</button>
    </div>
  </div>

  <script>
    const canvas = document.getElementById("canvas");
    const ctx = canvas.getContext("2d");
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;

    function interpolateColor(t) {
      let r = 0, g = 0, b = 0;
      if (t < 0.25) {
        let nt = t / 0.25;
        r = 255;
        g = Math.floor(165 * nt);
        b = 0;
      } else if (t < 0.5) {
        let nt = (t - 0.25) / 0.25;
        r = 255;
        g = Math.floor(165 + (90 * nt));
        b = 0;
      } else if (t < 0.75) {
        let nt = (t - 0.5) / 0.25;
        r = Math.floor(255 - 255 * nt);
        g = 255;
        b = 0;
      } else {
        let nt = (t - 0.75) / 0.25;
        r = 0;
        g = Math.floor(255 - 255 * nt);
        b = Math.floor(255 * nt);
      }
      return `rgb(${r},${g},${b})`;
    }

    function drawPoints(points) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      for (let p of points) {
        let dx = p.x - centerX;
        let dy = p.y - centerY;
        let dist = Math.sqrt(dx * dx + dy * dy);
        let maxDist = Math.sqrt(centerX * centerX + centerY * centerY);
        let t = dist / maxDist;
        ctx.fillStyle = interpolateColor(t);
        ctx.fillRect(p.x, p.y, 3, 3);
      }
    }

    async function update() {
      try {
        const res = await fetch("/data");
        const json = await res.json();
        drawPoints(json);
      } catch (e) {
        console.error("Fetch error:", e);
      }
    }

    setInterval(update, 50);
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// =================== LIDAR Task ===================

void lidarTask(void* parameter) {
  while (true) {
    int currentMaxLidarDistance;
    int currentStartAngle;
    int currentEndAngle;

    // Achizitioneaza mutex-ul inainte de a citi variabilele partajate
    if (xSemaphoreTake(xLidarParamsMutex, portMAX_DELAY) == pdTRUE) {
      currentMaxLidarDistance = maxLidarDistance;
      currentStartAngle = startAngle;
      currentEndAngle = endAngle;
      xSemaphoreGive(xLidarParamsMutex); // Elibereaza mutex-ul
    } else {
      // In caz de eroare, folosim valorile default sau cele anterioare
      Serial.println("Eroare la achizitia mutex-ului in lidarTask!");
      currentMaxLidarDistance = 800; // Fallback
      currentStartAngle = 0;
      currentEndAngle = 360;
    }

    // Apelam getIndexedPoints cu valorile curente, protejate
    std::vector<IndexedPoint> temp = lidar.getIndexedPoints(currentMaxLidarDistance, 511, currentStartAngle, currentEndAngle);
    points = temp; // Nota: Daca "points" ar fi citit de alte task-uri si ar putea fi modificat
                   // concomitent, ar trebui protejat si el. In acest caz, este doar scris
                   // aici si citit de server.handleClient, deci este ok.
    vTaskDelay(30 / portTICK_PERIOD_MS); //~30 FPS
  }
}

// =================== Setup / Loop ===================

void setup() {
  Serial.begin(115200);
  lidar.setup();

  // Creeaza mutex-ul
  xLidarParamsMutex = xSemaphoreCreateMutex();
  if (xLidarParamsMutex == NULL) {
    Serial.println("Eroare la crearea mutex-ului!");
    // Ar trebui gestionata o eroare aici, poate un reset sau un indicator
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  delay(1000);
  Serial.println("AP IP address: " + WiFi.softAPIP().toString());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setDistance", handleSetDistance);
  server.on("/setAngles", handleSetAngles);
  server.begin();

  xTaskCreatePinnedToCore(
    lidarTask, "LIDAR Task", 4096, NULL, 1, &lidarTaskHandle, 1
  );
}

void loop() {
  server.handleClient();
}