#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <string.h>
#include "config.h"
#include "motors.h"
#include "autopilot.h"
#include "web.h"
#include "sensors.h"
#include "mapping.h"

static WebServer server(80);

static void handleRoot() {
  static char page[6144];
  int speed = motorsSpeedPercent();
  const char* mode = autoIsEnabled() ? "Auto" : "Manual";
  
  snprintf(page, sizeof(page),
    "<html>\n"
    "<head>\n"
    "  <meta charset=\"UTF-8\">\n"
    "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
    "  <style>\n"
    "    body { background-color: blue; color: white; text-align: center; font-family: sans-serif; }\n"
    "    button { font-size: 22px; padding: 14px 22px; margin: 6px; }\n"
    "    input[type=range] { width: 70%%; height: 30px; }\n"
    "    canvas { display: block; width: min(90%%, 520px); aspect-ratio: 1; height: auto; margin: 12px auto; background: #172126; border: 1px solid #888; image-rendering: pixelated; }\n"
    "  </style>\n"
    "</head>\n"
    "<body>\n"
    "  <h1>ESP32 Robot</h1>\n"
    "  <p>Mode: %s</p>\n"
    "  <p>\n"
    "    <a href=\"/auto\"><button>Auto</button></a>\n"
    "    <a href=\"/manual\"><button>Manual</button></a>\n"
    "    <a href=\"/mapping\"><button>Mapping</button></a>\n"
    "  </p>\n"
    "  <p>\n"
    "    <a href=\"/forward\"><button>Forward</button></a><br>\n"
    "    <a href=\"/left\"><button>Left</button></a>\n"
    "    <a href=\"/stop\"><button>Stop</button></a>\n"
    "    <a href=\"/right\"><button>Right</button></a><br>\n"
    "    <a href=\"/backward\"><button>Backward</button></a>\n"
    "  </p>\n"
    "  <h3>Скорость: %d%%</h3>\n"
    "  <form action=\"/speed\" method=\"GET\">\n"
    "    <input type=\"range\" name=\"v\" min=\"0\" max=\"100\" step=\"5\" value=\"%d\" onchange=\"this.form.submit()\">\n"
    "  </form>\n"
    "  <h2>Карта помещения</h2>\n"
    "  <canvas id=\"mapCanvas\" width=\"400\" height=\"400\"></canvas>\n"
    "  <p id=\"mapUpdated\">Ожидание карты...</p>\n"
    "  <p>🟥 Препятствие &nbsp; 🟩 Свободно &nbsp; 🟨 Робот</p>\n"
    "  <script>\n"
    "    function updateMap() {\n"
    "      fetch('/api/map').then(r => { if (!r.ok) throw new Error(r.status); return r.text(); })\n"
    "        .then(text => {\n"
    "          const canvas = document.getElementById('mapCanvas');\n"
    "          const ctx = canvas.getContext('2d');\n"
    "          const rows = text.trimEnd().split('\\n');\n"
    "          const cell = canvas.width / rows[0].length;\n"
    "          ctx.fillStyle = '#172126'; ctx.fillRect(0, 0, canvas.width, canvas.height);\n"
    "          rows.forEach((row, y) => { for (let x = 0; x < row.length; x++) {\n"
    "            const c = row[x];\n"
    "            if (c === '#') ctx.fillStyle = '#e35b48';\n"
    "            else if (c === '.') ctx.fillStyle = '#88c78a';\n"
    "            else if (c === 'R') ctx.fillStyle = '#ffd166';\n"
    "            else continue;\n"
    "            ctx.fillRect(x * cell, y * cell, cell, cell);\n"
    "          }});\n"
    "          document.getElementById('mapUpdated').textContent = 'Карта обновлена: ' + new Date().toLocaleTimeString();\n"
    "        }).catch(e => { document.getElementById('mapUpdated').textContent = 'Карта недоступна: ' + e; });\n"
    "    }\n"
    "    updateMap(); setInterval(updateMap, 1500);\n"
    "  </script>\n"
    "</body>\n"
    "</html>\n", mode, speed, speed);
  server.send(200, "text/html", page);
}

static void handleApiStatus() {
  RobotStatus st = autoGetStatus();
  SensorData s = sensorsGet();

  char t[12], h[12];
  if (s.dhtOk) {
    snprintf(t, sizeof(t), "%.1f", s.temperatureC);
    snprintf(h, sizeof(h), "%.1f", s.humidity);
  } else {
    strcpy(t, "null");   // NaN в JSON недопустим
    strcpy(h, "null");
  }

  static char json[1024];
  snprintf(json, sizeof(json),
    "{\"state\":\"%s\",\"distance\":%.1f,\"servoAngle\":%d,\"servoReady\":%s,"
    "\"hits\":%d,\"stateTimeMs\":%lu,\"lastDecision\":\"%s\",\"autoEnabled\":%s,"
    "\"temp\":%s,\"hum\":%s,\"gasMv\":%d,\"gasPct\":%d,\"gasWarm\":%s,"
    "\"soundMv\":%d,\"soundPct\":%d}",
    st.stateName, st.distanceCm, st.servoAngle,
    st.servoReady ? "true" : "false",
    st.hits, st.stateTimeMs, st.lastDecision,
    st.autoEnabled ? "true" : "false",
    t, h, s.gasMv, s.gasPercent, s.gasWarm ? "true" : "false",
    s.soundMv, s.soundPercent);

  server.send(200, "application/json", json);
}

static void handleApiMap() {
  static char mapText[10101];
  size_t length = mappingCopyDisplay(mapText, sizeof(mapText));
  if (length == 0) {
    server.send(500, "text/plain", "Map buffer too small");
    return;
  }
  server.send(200, "text/plain; charset=utf-8", mapText);
}

static const char STATUS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Status</title>
<style>
  body { font-family: Arial, sans-serif; background: #1a1a1a; color: #fff; padding: 20px; text-align: center; }
  .card { background: #2a2a2a; border-radius: 10px; padding: 20px; margin: 10px auto; max-width: 500px; }
  .state { font-size: 28px; font-weight: bold; color: #4CAF50; margin-bottom: 10px; }
  .metric { display: inline-block; margin: 10px 15px; vertical-align: top; }
  .metric-label { font-size: 14px; color: #888; }
  .metric-value { font-size: 22px; font-weight: bold; }
  .decision { background: #333; padding: 15px; border-radius: 5px; margin-top: 10px; font-size: 18px; color: #ffeb3b; }
  .bar { height: 10px; background: #444; border-radius: 5px; margin-top: 6px; overflow: hidden; }
  .fill { height: 100%; width: 0; background: #4CAF50; transition: width .3s; }
  .map-canvas { display: block; width: min(100%, 520px); aspect-ratio: 1; height: auto; margin: 12px auto; background: #151a1d; image-rendering: pixelated; border: 1px solid #555; }
  .map-legend { display: flex; justify-content: center; gap: 18px; flex-wrap: wrap; color: #bbb; font-size: 14px; }
  .map-key { display: inline-flex; align-items: center; gap: 6px; }
  .swatch { width: 12px; height: 12px; display: inline-block; }
  h3 { margin: 0 0 5px; color: #aaa; font-size: 16px; }
  a.btn { display: inline-block; margin-top: 20px; padding: 10px 20px; background: #4CAF50; color: white; text-decoration: none; border-radius: 5px; }
</style>
</head>
<body>
<h1>🤖 Монитор состояния робота</h1>

<div class="card">
  <div class="state" id="state">Загрузка...</div>
  <div id="autoStatus"></div>
</div>

<div class="card">
  <div class="metric"><div class="metric-label">Дистанция</div>
    <div class="metric-value"><span id="distance">--</span> см</div></div>
  <div class="metric"><div class="metric-label">Угол серво</div>
    <div class="metric-value"><span id="servoAngle">--</span>°</div></div>
  <div class="metric"><div class="metric-label">Серво готово</div>
    <div class="metric-value" id="servoReady">--</div></div>
  <div class="metric"><div class="metric-label">Подряд препятствий</div>
    <div class="metric-value" id="hits">--</div></div>
  <div class="metric"><div class="metric-label">Время в состоянии</div>
    <div class="metric-value"><span id="stateTime">--</span> мс</div></div>
</div>

<div class="card">
  <h3>Окружающая среда (DHT22)</h3>
  <div class="metric"><div class="metric-label">Температура</div>
    <div class="metric-value"><span id="temp">--</span> °C</div></div>
  <div class="metric"><div class="metric-label">Влажность</div>
    <div class="metric-value"><span id="hum">--</span> %</div></div>
</div>

<div class="card">
  <h3>Газ (MQ5)</h3>
  <div class="metric"><div class="metric-label">Уровень</div>
    <div class="metric-value"><span id="gasPct">--</span> %</div></div>
  <div class="metric"><div class="metric-label">Сигнал</div>
    <div class="metric-value"><span id="gasMv">--</span> мВ</div></div>
  <div class="bar"><div class="fill" id="gasBar"></div></div>
  <div class="metric-label" id="gasWarm" style="margin-top:8px"></div>
</div>

<div class="card">
  <h3>Звук (Me Sound)</h3>
  <div class="metric"><div class="metric-label">Громкость</div>
    <div class="metric-value"><span id="soundPct">--</span> %</div></div>
  <div class="metric"><div class="metric-label">Сигнал</div>
    <div class="metric-value"><span id="soundMv">--</span> мВ</div></div>
  <div class="bar"><div class="fill" id="soundBar"></div></div>
</div>

<div class="card">
  <div class="metric-label">Последнее решение / действие:</div>
  <div class="decision" id="lastDecision">Загрузка...</div>
</div>

<div class="card">
  <h3>Карта помещения</h3>
  <canvas id="mapCanvas" class="map-canvas" width="400" height="400"></canvas>
  <div class="map-legend">
    <span class="map-key"><i class="swatch" style="background:#e35b48"></i>Препятствие</span>
    <span class="map-key"><i class="swatch" style="background:#88c78a"></i>Свободно</span>
    <span class="map-key"><i class="swatch" style="background:#202a31"></i>Неизвестно</span>
    <span class="map-key"><i class="swatch" style="background:#ffd166"></i>Робот</span>
  </div>
  <div class="metric-label" id="mapUpdated">Ожидание карты...</div>
</div>

<a href="/" class="btn">← Вернуться к управлению</a>

<script>
const $ = id => document.getElementById(id);
function updateStatus() {
  fetch('/api/status')
    .then(r => r.json())
    .then(d => {
      $('state').textContent = d.state;
      $('autoStatus').innerHTML = d.autoEnabled
        ? '<span style="color:#4CAF50;font-size:18px;">● АВТОПИЛОТ ВКЛЮЧЕН</span>'
        : '<span style="color:#888;font-size:18px;">● РУЧНОЙ РЕЖИМ</span>';
      $('distance').textContent = d.distance.toFixed(1);
      $('servoAngle').textContent = d.servoAngle;
      const re = $('servoReady');
      re.textContent = d.servoReady ? '✓ Да' : '✗ Нет (ждём)';
      re.style.color = d.servoReady ? '#4CAF50' : '#ff9800';
      $('hits').textContent = d.hits;
      $('stateTime').textContent = d.stateTimeMs;
      $('lastDecision').textContent = d.lastDecision;

      $('temp').textContent = d.temp === null ? '--' : d.temp.toFixed(1);
      $('hum').textContent  = d.hum  === null ? '--' : d.hum.toFixed(1);

      $('gasPct').textContent = d.gasPct;
      $('gasMv').textContent = d.gasMv;
      $('gasBar').style.width = d.gasPct + '%';
      $('gasBar').style.background = d.gasPct > 60 ? '#f44336' : d.gasPct > 30 ? '#ff9800' : '#4CAF50';
      $('gasWarm').textContent = d.gasWarm ? '' : '⏳ Датчик прогревается, показания неточные';

      $('soundPct').textContent = d.soundPct;
      $('soundMv').textContent = d.soundMv;
      $('soundBar').style.width = d.soundPct + '%';
    })
    .catch(e => console.error('Ошибка:', e));
}
setInterval(updateStatus, 700);
updateStatus();

function updateMap() {
  fetch('/api/map')
    .then(r => {
      if (!r.ok) throw new Error('HTTP ' + r.status);
      return r.text();
    })
    .then(text => {
      const canvas = $('mapCanvas');
      const context = canvas.getContext('2d');
      const rows = text.trimEnd().split('\n');
      const cellSize = canvas.width / rows[0].length;
      context.fillStyle = '#202a31';
      context.fillRect(0, 0, canvas.width, canvas.height);
      rows.forEach((row, y) => {
        for (let x = 0; x < row.length; x++) {
          const cell = row[x];
          if (cell === '#') context.fillStyle = '#e35b48';
          else if (cell === '.') context.fillStyle = '#88c78a';
          else if (cell === 'R') context.fillStyle = '#ffd166';
          else continue;
          context.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
        }
      });
      $('mapUpdated').textContent = 'Обновлено: ' + new Date().toLocaleTimeString();
    })
    .catch(e => {
      $('mapUpdated').textContent = 'Карта недоступна: ' + e.message;
    });
}
setInterval(updateMap, 1500);
updateMap();
</script>
</body>
</html>
)rawliteral";

static void handleStatusPage() {
  server.send_P(200, "text/html", STATUS_HTML);
}

static void redirectHome() {
  server.sendHeader("Location", "/");
  server.send(303);
}

// Ручная команда: выключает автопилот и выполняет движение
static void manual(Move m, const char* name) {
  Serial.printf(">>> %s\n", name);
  mappingStop();
  autoStop();
  motorsMove(m, true);
  redirectHome();
}

static void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Подключение к WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT) {
    delay(100);
    Serial.print(".");
    yield();
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP-адрес робота: ");
    Serial.println(WiFi.localIP());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("Домашний WiFi не найден. Точка доступа: %s (пароль %s)\n", AP_SSID, AP_PASS);
    Serial.print("IP-адрес робота: ");
    Serial.println(WiFi.softAPIP());   // обычно 192.168.4.1
  }
}

void webInit() {
  connectWifi();

  server.on("/", handleRoot);
  server.on("/forward",  []() { manual(M_FORWARD,  "ВПЕРЁД"); });
  server.on("/backward", []() { manual(M_BACKWARD, "НАЗАД");  });
  server.on("/left",     []() { manual(M_LEFT,     "ЛЕВО");   });
  server.on("/right",    []() { manual(M_RIGHT,    "ПРАВО");  });
  server.on("/stop",     []() { Serial.println(">>> СТОП");   mappingStop(); autoStop(); redirectHome(); });
  server.on("/manual",   []() { Serial.println(">>> MANUAL"); mappingStop(); autoStop(); redirectHome(); });
  server.on("/auto",     []() { Serial.println(">>> AUTO");   mappingStop(); autoStart(); redirectHome(); });
  server.on("/mapping",  []() { Serial.println(">>> MAPPING"); autoStop(); mappingStart(); redirectHome(); });
  server.on("/status", handleStatusPage);
  server.on("/api/status", handleApiStatus);
  server.on("/api/map", handleApiMap);
  server.on("/speed", []() {
    if (server.hasArg("v")) {
      motorsSetSpeedPercent(server.arg("v").toInt());
      if (autoIsEnabled()) autoReapplyMotion();
      else                 motorsReapply();
    }
    redirectHome();
  });

  server.begin();
  Serial.println("Веб-сервер запущен.");
}

void webUpdate() {
  server.handleClient();
}