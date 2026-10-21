#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "config.h"
#include "mapping.h"
#include "motors.h"
#include "scanner.h"
#include "autopilot.h"
#include "ultrasonic.h"
#include "web.h"
// ---------- Пины ----------

// ---------- Параметры скана ----------
const int   ANGLE_STEP = 5;
const int   N_ANGLES   = ((SERVO_LEFT - SERVO_RIGHT + ANGLE_STEP - 1) / ANGLE_STEP) + 1;
const float MIN_CM = 3.0f, MAX_CM = 200.0f;

// ---------- Параметры ICP ----------
const int   ICP_ITERS   = 40;
const float ICP_MAX_D   = 15.0f;   // см, порог пар
const float ICP_MAX_ERR = 4.0f;    // см, порог качества
const int   ICP_MIN_PAIRS = 10;

// ---------- Карта ----------
const int   GRID = 200;            // 200 x 200 клеток
const float RES  = 2.0f;           // см на клетку -> 4 x 4 м
const int   OX = GRID / 2, OY = GRID / 2;
const int8_t L_OCC = 3, L_FREE = 1, L_MAX = 20;
int8_t grid[GRID][GRID];           // 40 КБ, лог-оддс в целых

struct Pt   { float x, y; };
struct Pose { float x, y, th; };   // см, см, рад

Pt prevPts[N_ANGLES], curPts[N_ANGLES];
int nPrev = 0, nCur = 0;
Pose pose = {0, 0, 0};
static bool active = false;
static bool initialized = false;

// Делает sweep, пишет валидные точки в out (кадр робота: x вперёд, серво 90° = прямо)
int doScan(Pt* out) {
  int n = 0;
  for (int i = 0; i < N_ANGLES; i++) {
    int a = SERVO_RIGHT + (SERVO_LEFT - SERVO_RIGHT) * i / (N_ANGLES - 1);
    scannerMove(a);
    while (!scannerReady()) {
      webUpdate();
      if (!active) return 0;
      delay(5);
      yield();
    }
    float r = ultrasonicScanRead();
    if (r > MIN_CM && r < MAX_CM) {
      float phi = (a - SERVO_CENTER) * DEG_TO_RAD;
      out[n].x = r * cosf(phi);
      out[n].y = r * sinf(phi);
      n++;
    }
  }
  return n;
}

// ================= ICP =================
// Ищет позу src-кадра в dst-кадре: p_dst = R(th) * p_src + (x, y)
bool icp(const Pt* src, int ns, const Pt* dst, int nd, Pose init, Pose& out, float& meanErr) {
  float th = init.th, tx = init.x, ty = init.y;
  static Pt A[N_ANGLES], B[N_ANGLES];

  for (int it = 0; it < ICP_ITERS; it++) {
    float c = cosf(th), s = sinf(th);
    int m = 0;
    float sumErr = 0;

    for (int i = 0; i < ns; i++) {
      float mx = c * src[i].x - s * src[i].y + tx;
      float my = s * src[i].x + c * src[i].y + ty;
      float best = 1e9f; int bj = -1;
      for (int j = 0; j < nd; j++) {
        float dx = mx - dst[j].x, dy = my - dst[j].y;
        float d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; bj = j; }
      }
      if (bj >= 0 && best < ICP_MAX_D * ICP_MAX_D) {
        A[m] = {mx, my}; B[m] = dst[bj]; m++;
        sumErr += sqrtf(best);
      }
    }
    if (m < ICP_MIN_PAIRS) return false;
    meanErr = sumErr / m;

    float cax = 0, cay = 0, cbx = 0, cby = 0;
    for (int k = 0; k < m; k++) { cax += A[k].x; cay += A[k].y; cbx += B[k].x; cby += B[k].y; }
    cax /= m; cay /= m; cbx /= m; cby /= m;

    float sxx = 0, sxy = 0, syx = 0, syy = 0;
    for (int k = 0; k < m; k++) {
      float ax = A[k].x - cax, ay = A[k].y - cay;
      float bx = B[k].x - cbx, by = B[k].y - cby;
      sxx += ax * bx; sxy += ax * by; syx += ay * bx; syy += ay * by;
    }
    float dth = atan2f(sxy - syx, sxx + syy);   // замкнутое решение 2D
    float cd = cosf(dth), sd = sinf(dth);
    float tdx = cbx - (cd * cax - sd * cay);
    float tdy = cby - (sd * cax + cd * cay);

    // накапливаем: T = Td * T
    float ntx = cd * tx - sd * ty + tdx;
    float nty = sd * tx + cd * ty + tdy;
    tx = ntx; ty = nty; th += dth;

    if (fabsf(dth) < 1e-4f && fabsf(tdx) < 0.01f && fabsf(tdy) < 0.01f) break;
  }
  out = {tx, ty, th};
  return true;
}

// ================= Позы =================
Pose compose(Pose p, Pose d) {
  float c = cosf(p.th), s = sinf(p.th);
  return { p.x + d.x * c - d.y * s,
           p.y + d.x * s + d.y * c,
           p.th + d.th };
}

// ================= Карта =================
inline void cellAdd(int gx, int gy, int8_t d) {
  if (gx < 0 || gx >= GRID || gy < 0 || gy >= GRID) return;
  int v = grid[gy][gx] + d;
  if (v > L_MAX) v = L_MAX;
  if (v < -L_MAX) v = -L_MAX;
  grid[gy][gx] = v;
}

void rayUpdate(int x0, int y0, int x1, int y1) {   // Bresenham
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (!(x0 == x1 && y0 == y1)) {
    cellAdd(x0, y0, -L_FREE);
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
  cellAdd(x1, y1, L_OCC);
}

void updateGrid(const Pt* pts, int n, Pose p) {
  float c = cosf(p.th), s = sinf(p.th);
  int sx = OX + (int)lroundf(p.x / RES);
  int sy = OY + (int)lroundf(p.y / RES);
  for (int i = 0; i < n; i++) {
    float wx = c * pts[i].x - s * pts[i].y + p.x;
    float wy = s * pts[i].x + c * pts[i].y + p.y;
    rayUpdate(sx, sy, OX + (int)lroundf(wx / RES), OY + (int)lroundf(wy / RES));
  }
}

void printMap() {
  Serial.println("MAP_BEGIN");
  for (int y = GRID - 1; y >= 0; y -= 2) {         // шаг 2 -> 100x100
    for (int x = 0; x < GRID; x += 2) {
      int8_t v = max(max(grid[y][x], grid[y][x + 1]),
                     max(grid[y - 1][x], grid[y - 1][x + 1]));
      int sx = OX + (int)lroundf(pose.x / RES), sy = OY + (int)lroundf(pose.y / RES);
      if (abs(x - sx) <= 1 && abs(y - sy) <= 1) Serial.print('R');
      else if (v > 4) Serial.print('#');
      else if (v < -2) Serial.print('.');
      else Serial.print(' ');
    }
    Serial.println();
  }
  Serial.println("MAP_END");
}

size_t mappingCopyDisplay(char* out, size_t capacity) {
  const int displaySize = GRID / 2;
  const size_t required = (size_t)displaySize * (displaySize + 1) + 1;
  if (!out || capacity < required) return 0;

  const int robotX = OX + (int)lroundf(pose.x / RES);
  const int robotY = OY + (int)lroundf(pose.y / RES);
  size_t offset = 0;
  for (int row = 0; row < displaySize; row++) {
    int y = GRID - 1 - row * 2;
    for (int x = 0; x < GRID; x += 2) {
      int8_t value = max(max(grid[y][x], grid[y][x + 1]),
                         max(grid[y - 1][x], grid[y - 1][x + 1]));
      if (abs(x - robotX) <= 1 && abs(y - robotY) <= 1) out[offset++] = 'R';
      else if (value > 4) out[offset++] = '#';
      else if (value < -2) out[offset++] = '.';
      else out[offset++] = ' ';
    }
    out[offset++] = '\n';
  }
  out[offset] = '\0';
  return offset;
}

// ================= Движение =================
const float MAPPING_FORWARD_MS_PER_CM = 50.0f;

float driveForward(float cm) {
  if (cm <= 0) return 0;
  const unsigned long targetMs = (unsigned long)(cm * MAPPING_FORWARD_MS_PER_CM);
  const unsigned long startedAt = millis();
  unsigned long lastMeasure = startedAt;
  int obstacleHits = 0;

  motorsForward();
  while (millis() - startedAt < targetMs) {
    webUpdate();
    if (!active) break;
    unsigned long now = millis();
    if (now - lastMeasure >= 60) {
      lastMeasure = now;
      float distance = ultrasonicRead();
      if (distance > 0 && distance < STOP_DIST_CM) obstacleHits++;
      else obstacleHits = 0;
      if (obstacleHits >= 2) {
        Serial.printf("Mapping: препятствие на %.1f см, останавливаюсь\n", distance);
        break;
      }
    }
    delay(5);
    yield();
  }
  motorsStop();
  return min(cm, (millis() - startedAt) / MAPPING_FORWARD_MS_PER_CM);
}

void turnInPlace(float deg) {
  if (deg == 0) return;
  unsigned long turnMs = (unsigned long)(fabsf(deg) * TURN_90_MS / 90.0f);
  unsigned long startedAt = millis();
  if (deg > 0) motorsLeft();
  else         motorsRight();
  while (millis() - startedAt < turnMs && active) {
    webUpdate();
    delay(5);
    yield();
  }
  motorsStop();
}

static bool chooseSafeMotion(float& turnDeg, float& distanceCm) {
  const int angleCount = autoScanAngleCount();
  float distances[7];
  if (angleCount > (int)(sizeof(distances) / sizeof(distances[0]))) return false;

  for (int i = 0; i < angleCount; i++) {
    scannerMove(autoScanAngle(i));
    while (!scannerReady()) {
      webUpdate();
      if (!active) return false;
      delay(5);
      yield();
    }
    if (!active) return false;
    distances[i] = ultrasonicScanRead();
  }
  scannerCenterNow();

  int farthestIndex = 0;
  int bestIndex = autoChooseDirection(distances, angleCount, &farthestIndex);
  if (bestIndex < 0) {
    Serial.println("Mapping: тупик, сдаю назад и разворачиваюсь");
    turnDeg = autoScanAngle(farthestIndex) >= SERVO_CENTER ? 180.0f : -180.0f;
    return false;
  }

  turnDeg = autoScanAngle(bestIndex) - SERVO_CENTER;
  float clearance = distances[bestIndex] - STOP_DIST_CM;
  distanceCm = min(distanceCm, clearance);
  return distanceCm > 0;
}

// ================= Основной код =================
const float MAPPING_STEP_CM = 30.0f;

void mappingStart() {
  if (active) return;
  memset(grid, 0, sizeof(grid));
  pose = {0, 0, 0};
  initialized = false;
  active = true;
  motorsStop();
  scannerCenterNow();
  Serial.println("Mapping started");
}

void mappingStop() {
  if (!active) return;
  active = false;
  motorsStop();
  scannerCenterNow();
  Serial.println("Mapping stopped");
}

bool mappingIsActive() {
  return active;
}

void mappingUpdate() {
  if (!active) return;
  if (!initialized) {
    nPrev = doScan(prevPts);
    if (!active) return;
    updateGrid(prevPts, nPrev, pose);
    initialized = true;
    return;
  }
  float turnDeg = 0;
  float distanceCm = MAPPING_STEP_CM;
  if (!chooseSafeMotion(turnDeg, distanceCm)) {
    if (!active) {
      motorsStop();
      return;
    }
    motorsBackward();
    unsigned long startedAt = millis();
    while (millis() - startedAt < BACK_TIME_MS && active) {
      webUpdate();
      delay(5);
      yield();
    }
    motorsStop();
    if (!active) return;
    turnInPlace(turnDeg);
    Pose recovery = {
      -(float)BACK_TIME_MS / MAPPING_FORWARD_MS_PER_CM,
      0,
      turnDeg * DEG_TO_RAD
    };
    pose = compose(pose, recovery);
    nCur = doScan(curPts);
    if (!active) return;
    updateGrid(curPts, nCur, pose);
    memcpy(prevPts, curPts, sizeof(Pt) * nCur);
    nPrev = nCur;
    return;
  }
  if (!active) return;

  // 1. Следуем по самому свободному проходу, выбранному алгоритмом автопилота.
  if (turnDeg != 0) turnInPlace(turnDeg);
  if (!active) return;
  distanceCm = driveForward(distanceCm);
  if (!active) return;
  delay(300);                                   // дать корпусу успокоиться

  // 2. Начальное приближение из команды
  float dth = turnDeg * DEG_TO_RAD;
  Pose guess = { distanceCm * cosf(dth), distanceCm * sinf(dth), dth };

  // 3. Новый скан и ICP
  nCur = doScan(curPts);
  if (!active) return;
  Pose icpRes; float err = 0;
  Pose delta = guess;
  if (icp(curPts, nCur, prevPts, nPrev, guess, icpRes, err) && err < ICP_MAX_ERR) {
    delta = icpRes;
    Serial.printf("ICP ok: dx=%.1f dy=%.1f dth=%.1f deg err=%.2f\n",
                  delta.x, delta.y, delta.th * RAD_TO_DEG, err);
  } else {
    Serial.println("ICP failed, using commanded motion");
  }

  // 4. Обновляем позу и карту
  pose = compose(pose, delta);
  updateGrid(curPts, nCur, pose);
  memcpy(prevPts, curPts, sizeof(Pt) * nCur);
  nPrev = nCur;

}