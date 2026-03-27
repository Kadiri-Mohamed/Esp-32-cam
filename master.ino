/*
 * ================================================================
 *  master.ino — Quadruped Master Controller v2.1 (SAFETY MODE)
 *  ESP32 + PCA9685 (I2C: SDA=21, SCL=22)
 *  8 servos on channels: 0, 2, 4, 6, 8, 10, 12, 14
 *
 *  SAFETY FEATURES:
 *    • Reduced speed (STEP_LEN = 8mm, STEP_MS = 400ms)
 *    • Slow startup (gradual stand-up)
 *    • Emergency stop button support
 *    • Watchdog timer for UDP connection
 * ================================================================
 */

#include <Adafruit_PWMServoDriver.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>

// ========== USER CONFIGURATION - EDIT THESE ==========
// Step 1: Enter your WiFi network credentials
#define WIFI_SSID "Your_WiFi_Name"        // CHANGE THIS
#define WIFI_PASS "Your_WiFi_Password"    // CHANGE THIS

// Step 2: Find your laptop's IP address (run 'ipconfig' on Windows or 'ifconfig' on Mac/Linux)
#define LAPTOP_IP "192.168.1.100"         // CHANGE THIS to your laptop IP

// Step 3: Optional - Adjust safety parameters
#define STEP_LEN 8.0f           // mm foot swing (lower = slower)
#define STEP_H 8.0f             // mm foot lift height (lower = more stable)
#define STEP_MS 400             // ms per step (higher = slower)
#define MAX_SPEED_FACTOR 0.3f   // Global speed limiter (0-1)
#define WATCHDOG_TIMEOUT_MS 5000 // Auto-stop if no UDP for 5 seconds

// ========== PORTS (DO NOT CHANGE unless you modify laptop_control.py) ==========
#define VIDEO_PORT 5005
#define MASTER_PORT 5006
#define CAM_PORT 5007
#define UDP_PORT 5000
#define ANNOUNCE_PORT 4999

// ========== HARDWARE PINS ==========
#define ESTOP_PIN 15            // Emergency stop button (connect to GND)
#define I2C_SDA 21
#define I2C_SCL 22

// ========== SERVO CONFIGURATION ==========
#define SERVOMIN 150            // pulse count → 0°
#define SERVOMAX 600            // pulse count → 180°
#define SERVO_FREQ 50           // 50Hz for MG996R

// ========== LEG GEOMETRY (mm) ==========
#define L1 55.0f                // hip joint → knee joint
#define L2 65.0f                // knee joint → foot tip

// ========== FOOT POSITIONS ==========
#define STAND_Z 85.0f           // comfortable standing height
#define STAND_X 0.0f
#define SIT_Z 45.0f             // crouched sit
#define SIT_X 0.0f

// ========== SERVO CHANNEL MAP ==========
const uint8_t HIP_CH[4] = {0, 4, 8, 12};
const uint8_t KNEE_CH[4] = {2, 6, 10, 14};

// Polarity: flip to -1 if a servo moves the wrong direction
const int HIP_DIR[4] = {1, -1, 1, -1};
const int KNEE_DIR[4] = {1, -1, 1, -1};

// Servo neutral = 90°
const int HIP_NEUTRAL[4] = {90, 90, 90, 90};
const int KNEE_NEUTRAL[4] = {90, 90, 90, 90};

// ========== OBJECTS ==========
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
WebServer httpServer(80);
WiFiUDP udp;

// ========== COMMAND ENUM ==========
enum Cmd {
  CMD_STOP,
  CMD_STAND,
  CMD_SIT,
  CMD_FWD,
  CMD_BWD,
  CMD_LEFT,
  CMD_RIGHT
};
volatile Cmd currentCmd = CMD_STOP;
Cmd prevCmd = CMD_STOP;
unsigned long lastUdpTime = 0;
unsigned long lastAnnounceMs = 0;

// ========== GAIT STATE ==========
int stepPhase = 0;
unsigned long lastStepMs = 0;
bool emergencyStopped = false;

// ========== UDP ANNOUNCEMENT ==========
WiFiUDP announceUdp;

// =============================================================================
//  IK — 2-DOF planar inverse kinematics
// =============================================================================
struct LegAngles {
  float hip, knee;
};

LegAngles computeIK(float x, float z) {
  float R2 = x * x + z * z;
  float maxR = L1 + L2 - 0.5f;

  // Clamp to reachable envelope
  if (R2 > maxR * maxR) {
    float s = maxR / sqrtf(R2);
    x *= s;
    z *= s;
    R2 = maxR * maxR;
  }

  // Knee interior angle (law of cosines)
  float cos_k = (R2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  cos_k = constrain(cos_k, -1.0f, 1.0f);
  float theta2 = acosf(cos_k);

  // Hip angle from downward vertical
  float alpha = atan2f(x, z);
  float beta = atan2f(L2 * sinf(theta2), L1 + L2 * cos_k);
  float theta1 = alpha - beta;

  LegAngles a;
  a.hip = degrees(theta1);
  a.knee = degrees(theta2);
  return a;
}

// =============================================================================
//  SERVO WRITE
// =============================================================================
void setServoAngle(uint8_t ch, int angle_deg) {
  if (emergencyStopped) return;
  angle_deg = constrain(angle_deg, 0, 180);
  int pulse = map(angle_deg, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(ch, 0, pulse);
}

void setLeg(uint8_t leg, float x, float z) {
  LegAngles a = computeIK(x, z);
  int hip_ang = HIP_NEUTRAL[leg] + HIP_DIR[leg] * (int)roundf(a.hip);
  int knee_ang = KNEE_NEUTRAL[leg] + KNEE_DIR[leg] * (int)roundf(a.knee);
  setServoAngle(HIP_CH[leg], hip_ang);
  setServoAngle(KNEE_CH[leg], knee_ang);
}

void setAllLegs(float x, float z) {
  for (int i = 0; i < 4; i++)
    setLeg(i, x, z);
}

// =============================================================================
//  POSES
// =============================================================================
void poseStand() {
  if (emergencyStopped) return;
  int pulse90 = map(90, 0, 180, SERVOMIN, SERVOMAX);
  for (int i = 0; i <= 14; i += 2)
    pwm.setPWM(i, 0, pulse90);
  Serial.println("[POSE] Stand — all servos 90°");
}

void poseSit() {
  if (emergencyStopped) return;
  setAllLegs(SIT_X, SIT_Z);
  Serial.println("[POSE] Sit");
}

// SLOW STARTUP - gradually rise from sit to stand
void slowStandUp() {
  Serial.println("[SAFETY] Slow start sequence...");
  for (int step = 0; step <= 100; step++) {
    if (emergencyStopped) break;
    float factor = step / 100.0f;
    float z = SIT_Z + (STAND_Z - SIT_Z) * factor;
    for (int leg = 0; leg < 4; leg++) {
      setLeg(leg, STAND_X, z);
    }
    delay(20);
  }
  Serial.println("[SAFETY] Robot ready - reduced speed mode");
}

// =============================================================================
//  TROT GAIT (with speed reduction)
// =============================================================================
void trotStep(float dx, float turn) {
  if (emergencyStopped) return;
  
  if (millis() - lastStepMs < STEP_MS)
    return;
  lastStepMs = millis();

  // Apply speed limiter
  dx = constrain(dx, -STEP_LEN, STEP_LEN);
  turn = constrain(turn, -1.0f, 1.0f);
  
  float lx = dx - turn * STEP_LEN * 0.5f;
  float rx = dx + turn * STEP_LEN * 0.5f;

  if (stepPhase == 0) {
    // Pair A lifts (FL + RR)
    setLeg(0, lx, STAND_Z - STEP_H);
    setLeg(3, rx, STAND_Z - STEP_H);
    // Pair B planted (FR + RL)
    setLeg(1, -rx, STAND_Z);
    setLeg(2, -lx, STAND_Z);
  } else {
    // Pair B lifts (FR + RL)
    setLeg(1, rx, STAND_Z - STEP_H);
    setLeg(2, lx, STAND_Z - STEP_H);
    // Pair A planted (FL + RR)
    setLeg(0, -lx, STAND_Z);
    setLeg(3, -rx, STAND_Z);
  }

  stepPhase ^= 1;
}

// =============================================================================
//  MOTION DISPATCHER (with speed limits)
// =============================================================================
void handleMotion() {
  if (emergencyStopped) return;
  
  bool entered = (currentCmd != prevCmd);
  prevCmd = currentCmd;

  switch (currentCmd) {
  case CMD_FWD:
    trotStep(STEP_LEN * MAX_SPEED_FACTOR, 0.0f);
    break;
  case CMD_BWD:
    trotStep(-STEP_LEN * MAX_SPEED_FACTOR, 0.0f);
    break;
  case CMD_LEFT:
    trotStep(0.0f, -MAX_SPEED_FACTOR);
    break;
  case CMD_RIGHT:
    trotStep(0.0f, MAX_SPEED_FACTOR);
    break;
  case CMD_STAND:
    if (entered) poseStand();
    break;
  case CMD_SIT:
    if (entered) poseSit();
    break;
  case CMD_STOP:
    if (entered) poseStand();
    break;
  }
}

// =============================================================================
//  EMERGENCY STOP
// =============================================================================
void IRAM_ATTR emergencyStop() {
  emergencyStopped = true;
  currentCmd = CMD_STOP;
  // Disable all servos immediately
  for (int i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, 0);
  }
  Serial.println("\n[ESTOP] EMERGENCY STOP ACTIVATED!");
}

// =============================================================================
//  UDP HANDLER
// =============================================================================
void handleUDP() {
  int pkt = udp.parsePacket();
  if (pkt < 1) return;
  
  lastUdpTime = millis();  // Reset watchdog

  char buf[32];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len] = '\0';

  String cmd = String(buf);
  cmd.trim();
  cmd.toUpperCase();

  // Clear emergency stop on any command
  if (emergencyStopped && cmd != "STOP") {
    Serial.println("[ESTOP] Clear emergency stop");
    emergencyStopped = false;
  }

  if (cmd == "FWD") currentCmd = CMD_FWD;
  else if (cmd == "BWD") currentCmd = CMD_BWD;
  else if (cmd == "LEFT") currentCmd = CMD_LEFT;
  else if (cmd == "RIGHT") currentCmd = CMD_RIGHT;
  else if (cmd == "STAND") currentCmd = CMD_STAND;
  else if (cmd == "SIT") currentCmd = CMD_SIT;
  else if (cmd == "STOP") currentCmd = CMD_STOP;

  Serial.printf("[UDP] %s\n", cmd.c_str());
}

// =============================================================================
//  WATCHDOG - Stop if no UDP for too long
// =============================================================================
void checkWatchdog() {
  if (millis() - lastUdpTime > WATCHDOG_TIMEOUT_MS && currentCmd != CMD_STOP) {
    Serial.println("[WATCHDOG] No UDP received - auto stopping");
    currentCmd = CMD_STOP;
    poseStand();
  }
}

// =============================================================================
//  IP ANNOUNCEMENT
// =============================================================================
void announceIP() {
  String msg = "QUAD:" + WiFi.localIP().toString();
  announceUdp.beginPacket(IPAddress(255, 255, 255, 255), ANNOUNCE_PORT);
  announceUdp.print(msg);
  announceUdp.endPacket();
  Serial.printf("[ANNOUNCE] %s → broadcast:%d\n", msg.c_str(), ANNOUNCE_PORT);
}

// =============================================================================
//  WIFI CONNECTION
// =============================================================================
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - t0 > 15000) {
      Serial.println("\n[WiFi] TIMEOUT — check SSID/PASS in code");
      return;
    }
  }
  Serial.printf("\n[WiFi] Connected → %s\n", WiFi.localIP().toString().c_str());
}

// =============================================================================
//  HTTP WEB UI
// =============================================================================
const char INDEX_HTML[] PROGMEM = R"html(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Quadruped Control</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{
    background:#080810;color:#dde;
    font-family:'Courier New',monospace;
    display:flex;flex-direction:column;align-items:center;
    justify-content:center;min-height:100vh;gap:24px;
    user-select:none;
  }
  h1{font-size:1.2rem;letter-spacing:4px;color:#58a9f5;text-transform:uppercase}
  #status{font-size:.7rem;letter-spacing:2px;color:#556;min-height:1em}
  #speed-indicator{font-size:.6rem;color:#3fa}
  .dpad{
    display:grid;
    grid-template-columns:repeat(3,72px);
    grid-template-rows:repeat(3,72px);
    gap:8px;
  }
  .btn{
    background:#0f0f18;border:1px solid #222;border-radius:12px;
    color:#888;font-size:1.6rem;cursor:pointer;
    transition:background .1s,transform .1s,border-color .1s;
    display:flex;align-items:center;justify-content:center;
  }
  .btn:active,.btn.active{transform:scale(.9);border-color:#58a9f5}
  #btnU{background:#091a09;border-color:#1e5c1e;color:#3fa}
  #btnD{background:#1a0909;border-color:#5c1e1e;color:#f55}
  #btnL,#btnR{background:#090d1a;border-color:#1e2e5c;color:#5af}
  #btnStp{font-size:.75rem;letter-spacing:1px;background:#141400;border-color:#554;color:#cc4}
  .actions{display:flex;gap:14px}
  .act{
    padding:14px 26px;border-radius:10px;
    border:1px solid #333;background:#0c0c14;
    color:#aaa;font-family:inherit;font-size:.78rem;
    letter-spacing:2px;text-transform:uppercase;cursor:pointer;
    transition:background .15s,border-color .15s;
  }
  .act:hover{background:#161620}
  #btnStand{border-color:#58a9f5;color:#58a9f5}
  #btnSit{border-color:#fa7;color:#fa7}
  #btnEStop{border-color:#f55;color:#f55}
</style>
</head>
<body>
<h1>🐕 Quadruped (Slow Mode)</h1>
<div id="status">IDLE</div>
<div id="speed-indicator">🔵 Speed: 30% | Step: 8mm | Interval: 400ms</div>

<div class="dpad">
  <div></div>
  <div class="btn" id="btnU"   onpointerdown="go('FWD')"  onpointerup="go('STOP')">▲</div>
  <div></div>
  <div class="btn" id="btnL"   onpointerdown="go('LEFT')" onpointerup="go('STOP')">◀</div>
  <div class="btn" id="btnStp" onpointerdown="go('STOP')">STOP</div>
  <div class="btn" id="btnR"   onpointerdown="go('RIGHT')"onpointerup="go('STOP')">▶</div>
  <div></div>
  <div class="btn" id="btnD"   onpointerdown="go('BWD')"  onpointerup="go('STOP')">▼</div>
  <div></div>
</div>

<div class="actions">
  <button class="act" id="btnStand" onclick="go('STAND')">STAND</button>
  <button class="act" id="btnSit"   onclick="go('SIT')">SIT</button>
  <button class="act" id="btnEStop" onclick="go('STOP'); go('STOP');">🛑 E-STOP</button>
</div>

<script>
function go(cmd){
  document.getElementById('status').textContent = cmd;
  fetch('/cmd?c='+cmd).catch(()=>{});
}
const K = {ArrowUp:'FWD',ArrowDown:'BWD',ArrowLeft:'LEFT',ArrowRight:'RIGHT',' ':'STOP',s:'STAND',x:'SIT',e:'STOP'};
const held = new Set();
document.addEventListener('keydown',e=>{
  if(K[e.key] && !held.has(e.key)){held.add(e.key);e.preventDefault();go(K[e.key]);}
  if(e.key === 'Escape') go('STOP');
});
document.addEventListener('keyup',e=>{
  held.delete(e.key);
  if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) go('STOP');
});
</script>
</body>
</html>
)html";

void setupHTTP() {
  httpServer.on("/", HTTP_GET,
                []() { httpServer.send_P(200, "text/html", INDEX_HTML); });

  httpServer.on("/cmd", HTTP_GET, []() {
    String cmd = httpServer.arg("c");
    cmd.toUpperCase();
    cmd.trim();
    
    if (cmd == "FWD") currentCmd = CMD_FWD;
    else if (cmd == "BWD") currentCmd = CMD_BWD;
    else if (cmd == "LEFT") currentCmd = CMD_LEFT;
    else if (cmd == "RIGHT") currentCmd = CMD_RIGHT;
    else if (cmd == "STAND") currentCmd = CMD_STAND;
    else if (cmd == "SIT") currentCmd = CMD_SIT;
    else if (cmd == "STOP") currentCmd = CMD_STOP;
    
    httpServer.send(200, "text/plain", "OK");
    Serial.printf("[HTTP] %s\n", cmd.c_str());
  });

  httpServer.begin();
  Serial.printf("[HTTP] Web UI → http://%s/\n", WiFi.localIP().toString().c_str());
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Quadruped Master v2.1 (SAFETY MODE) ===");
  Serial.printf("  Step length: %d mm\n", (int)STEP_LEN);
  Serial.printf("  Step interval: %d ms\n", STEP_MS);
  Serial.printf("  Speed limit: %.0f%%\n", MAX_SPEED_FACTOR * 100);
  Serial.printf("  WiFi: %s\n", WIFI_SSID);
  Serial.printf("  Laptop IP: %s\n", LAPTOP_IP);

  // Emergency stop button
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), emergencyStop, FALLING);

  // Initialize PCA9685
  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);

  // Slow startup
  slowStandUp();

  // Connect to WiFi
  connectWiFi();
  
  // Setup UDP
  udp.begin(UDP_PORT);
  announceUdp.begin(ANNOUNCE_PORT);
  Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);
  Serial.printf("[UDP] Laptop should send commands to %s:%d\n", LAPTOP_IP, MASTER_PORT);
  
  // Setup HTTP server
  setupHTTP();
  
  // Announce IP
  announceIP();
  lastUdpTime = millis();

  currentCmd = CMD_STAND;
  Serial.println("[READY] Robot ready for commands");
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
  httpServer.handleClient();
  handleUDP();
  handleMotion();
  checkWatchdog();

  // Announce IP every 3 seconds
  if (millis() - lastAnnounceMs >= 3000) {
    lastAnnounceMs = millis();
    announceIP();
  }
}