/*
 * ASCENT — AUTONOMOUS BATTERY SWAP STATION — ARDUINO MEGA
 * Infinite swap cycle. Values follow the Team 10732 pre-field report.
 *
 * MECHANISM:
 *   X axis = 2 stepper motors on the shared X STEP/DIR signals; Y = 1 motor; Z = 1 motor
 *   (Z is the battery swap table). X and Y are the landing-station beams that align the drone.
 *   The linear actuator does ALL pushing and pulling of the battery; the servo only
 *   locks (0 deg) and releases (90 deg) the latch hook.
 *
 * STARTUP (once):
 *   X FORWARD -> X HOME (A3), then Y FORWARD -> Y FWD (D24)
 *
 * CYCLE (repeats; X and Y wait at home between swaps):
 *   1.  SCAN         -> drone <= 50 cm (5 consecutive reads to confirm)
 *   2.  CONFIRM WAIT -> 3 s
 *   3.  X REVERSE    -> X REV (D9), then Y REVERSE -> Y REV (D25)
 *                       (the X and Y beams align the drone and stop on the limit switches)
 *   4.  Z moves      -> cycle 1: Z FWD (D22) | cycle 2+: opposite of the pressed limit
 *   5.  ACTUATOR     -> EXTEND 25 s (servo 0 deg, battery locked)
 *                       RETRACT 25 s (servo 90 deg, battery released)
 *   6.  Z CHECK      -> moves to the opposite limit switch
 *   7.  ACTUATOR     -> EXTEND 25 s, RETRACT 25 s (servo stays locked)
 *   8.  Y FORWARD    -> Y FWD (D24), then X FORWARD -> X HOME (A3)
 *   9.  ACTUATOR     -> final EXTEND 25 s, RETRACT 25 s
 *   10. STANDBY      -> minimum 7 minutes (no scanning), then back to step 1
 *
 * WIRING:
 *   X: STEP=D2 DIR=D5   Y: STEP=D3 DIR=D6
 *   Z: STEP=D4 DIR=D7   ENABLE=D8
 *   X HOME=A3  X REV=D9
 *   Y FWD=D24  Y REV=D25
 *   Z FWD=D22  Z REV=D23
 *   ACT RPWM=D10(1k) LPWM=D11(1k)
 *   SERVO=A2
 *   US TRIG=A0  ECHO=A1
 *
 * Commands (serial, 115200):
 *   S  - emergency stop
 *   SW - switch states
 *   D  - read ultrasonic
 *   H  - help
 */

#include <Servo.h>

// -- Pins ----------------------------------
#define STEP_X     2
#define DIR_X      5
#define STEP_Y     3
#define DIR_Y      6
#define STEP_Z     4
#define DIR_Z      7
#define ENABLE_PIN 8
#define X_HOME     A3
#define X_REV      9
#define Y_FWD      24
#define Y_REV      25
#define Z_FWD      22
#define Z_REV      23
#define RPWM       10
#define LPWM       11
#define SERVO_PIN  A2
#define TRIG_PIN   A0
#define ECHO_PIN   A1

// -- Config --------------------------------
#define DRONE_CM              50
#define ACT_PWM               76          // 30% PWM, about 15 mm/s
#define SERVO_LOCKED_DEG      0           // battery locked
#define SERVO_RELEASED_DEG    90          // battery released
const unsigned long STEP_DELAY_FWD = 400;   // us between steps
const unsigned long STEP_DELAY_REV = 600;
const unsigned long STEP_DELAY_Z   = 800;
const unsigned long ACT_STROKE_MS  = 25000; // every actuator stroke, extend (push) or retract (pull)
const unsigned long CONFIRM_WAIT_MS = 3000; // wait after the drone is detected
const unsigned long MIN_SWAP_MS    = 420000UL; // 7 minutes between swaps

enum State {
  IDLE,
  S_WAIT_DRONE_ON_PAD,   // scan for the landed drone (5 consecutive reads)
  S1_X_FWD,
  S2_Y_FWD,
  S_WAIT,
  S_X_REV,
  S_Y_REV,
  S_Z_FWD_FIRST,
  S_ACT1_EXTEND,
  S_ACT1_RETRACT,
  S_Z_CHECK,
  S_ACT2_EXTEND,
  S_ACT2_RETRACT,
  S_Y_HOME,
  S_X_HOME,
  S_ACT4_EXTEND,
  S_ACT4_RETRACT,
  S_COUNTDOWN,
  S_WAIT_DRONE
};
State currentState = IDLE;

bool          firstCycle      = true;
int           swapCount       = 0;
bool          emergency       = false;
bool          zMoveDir        = true;
unsigned long lastStepX       = 0;
unsigned long lastStepY       = 0;
unsigned long lastStepZ       = 0;
unsigned long actStart        = 0;
unsigned long waitStart       = 0;
unsigned long swapCompleteTime= 0;
unsigned long lastPrintT      = 0;
int landCount = 0;     // consecutive landing reads
unsigned long lastPrintC      = 0;

Servo myServo;

bool switchPressed(int pin, uint8_t samples = 8, unsigned int gapUs = 300);  // forward declaration

// -- Helpers -------------------------------
void stepX() { digitalWrite(STEP_X,HIGH); delayMicroseconds(10); digitalWrite(STEP_X,LOW); }
void stepY() { digitalWrite(STEP_Y,HIGH); delayMicroseconds(10); digitalWrite(STEP_Y,LOW); }
void stepZ() { digitalWrite(STEP_Z,HIGH); delayMicroseconds(10); digitalWrite(STEP_Z,LOW); }
void stopMotors() { digitalWrite(ENABLE_PIN, HIGH); }
void actExtend()  { analogWrite(RPWM, ACT_PWM); digitalWrite(LPWM, LOW); }
void actRetract() { digitalWrite(RPWM, LOW); analogWrite(LPWM, ACT_PWM); }
void actStop()    { digitalWrite(RPWM, LOW); digitalWrite(LPWM, LOW); }

int readDist() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (!dur) return -1;
  int d = (int)(dur * 0.0343 / 2.0);
  return (d > 0 && d < 400) ? d : -1;
}

void prepareZMove(bool goFwd) {
  zMoveDir = goFwd;
  int targetPin = goFwd ? Z_FWD : Z_REV;
  unsigned long ws = millis();
  while (switchPressed(targetPin)) {
    if (millis() - ws > 3000) { Serial.println(F("[WARN] Z stuck")); break; }
    delay(10);
  }
  delay(100);
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_Z, goFwd ? HIGH : LOW);
  delayMicroseconds(200);
}

// Returns true only if the switch reads PRESSED (LOW) consistently across
// 'samples' consecutive checks, spaced 'gapUs' microseconds apart. This stops
// electrical noise from the motors being mistaken for a real switch press.
bool switchPressed(int pin, uint8_t samples, unsigned int gapUs) {
  for (uint8_t i = 0; i < samples; i++) {
    if (digitalRead(pin) != LOW) return false;   // any HIGH read = not pressed
    delayMicroseconds(gapUs);
  }
  return true;
}

void printSwitches() {
  Serial.println(F("\n--- SWITCH STATES ---"));
  Serial.print(F("X_HOME(A3): ")); Serial.println(digitalRead(X_HOME)==LOW?F("PRESSED"):F("open"));
  Serial.print(F("X_REV (D9): ")); Serial.println(digitalRead(X_REV) ==LOW?F("PRESSED"):F("open"));
  Serial.print(F("Y_FWD(D24): ")); Serial.println(digitalRead(Y_FWD) ==LOW?F("PRESSED"):F("open"));
  Serial.print(F("Y_REV(D25): ")); Serial.println(digitalRead(Y_REV) ==LOW?F("PRESSED"):F("open"));
  Serial.print(F("Z_FWD(D22): ")); Serial.println(digitalRead(Z_FWD) ==LOW?F("PRESSED"):F("open"));
  Serial.print(F("Z_REV(D23): ")); Serial.println(digitalRead(Z_REV) ==LOW?F("PRESSED"):F("open"));
  Serial.println(F("---------------------\n"));
}

// -- Step functions ------------------------
void goStep1() {
  Serial.println(F("[1] X FWD -> X HOME (A3)"));
  digitalWrite(ENABLE_PIN, LOW); digitalWrite(DIR_X, HIGH); delayMicroseconds(200);
  currentState = S1_X_FWD; lastStepX = micros();
}
void goStep2() {
  Serial.println(F("[2] Y FWD -> Y FWD (D24)"));
  digitalWrite(ENABLE_PIN, LOW); digitalWrite(DIR_Y, HIGH); delayMicroseconds(200);
  currentState = S2_Y_FWD; lastStepY = micros();
}
void goScan() {
  Serial.println(F("[SCAN] Scanning for drone landing <=50cm..."));
  landCount = 0;
  lastPrintT = millis();
  currentState = S_WAIT_DRONE_ON_PAD;
}
void goWait() {
  Serial.println(F("[WAIT] Drone detected - waiting 3s..."));
  waitStart = millis(); currentState = S_WAIT;
}
void goXRev() {
  Serial.println(F("[X REV] -> X REV (D9)"));
  digitalWrite(ENABLE_PIN, LOW); digitalWrite(DIR_X, LOW); delayMicroseconds(200);
  currentState = S_X_REV; lastStepX = micros();
}
void goYRev() {
  Serial.println(F("[Y REV] -> Y REV (D25)"));
  digitalWrite(ENABLE_PIN, LOW); digitalWrite(DIR_Y, LOW); delayMicroseconds(200);
  currentState = S_Y_REV; lastStepY = micros();
}
void goZFwdFirst() {
  Serial.println(F("[Z FWD] -> Z FWD (D22) [cycle 1]"));
  prepareZMove(true);
  currentState = S_Z_FWD_FIRST; lastStepZ = micros();
}
void goAct1Extend() {
  Serial.println(F("[ACT1] EXTENDING 25s - SERVO 0 deg (locked)"));
  myServo.write(SERVO_LOCKED_DEG);
  actExtend(); actStart = millis();
  currentState = S_ACT1_EXTEND;
}
void goAct1Retract() {
  Serial.println(F("[ACT1] RETRACTING 25s - SERVO 90 deg (released)"));
  myServo.write(SERVO_RELEASED_DEG);
  actRetract(); actStart = millis();
  currentState = S_ACT1_RETRACT;
}
void goZCheck() {
  myServo.write(SERVO_LOCKED_DEG);
  bool zFwdPressed = switchPressed(Z_FWD);
  bool zRevPressed = switchPressed(Z_REV);
  bool goFwd;
  if (zRevPressed)      { goFwd = true;  Serial.println(F("[Z] REV pressed -> going FWD")); }
  else if (zFwdPressed) { goFwd = false; Serial.println(F("[Z] FWD pressed -> going REV")); }
  else                  { goFwd = false; Serial.println(F("[Z] none pressed -> going REV (default)")); }
  prepareZMove(goFwd);
  currentState = S_Z_CHECK; lastStepZ = micros();
}
void goAct2Extend() {
  Serial.println(F("[ACT2] EXTENDING 25s"));
  actExtend(); actStart = millis(); currentState = S_ACT2_EXTEND;
}
void goAct2Retract() {
  Serial.println(F("[ACT2] RETRACTING 25s"));
  actRetract(); actStart = millis(); currentState = S_ACT2_RETRACT;
}
void goYHome() {
  Serial.println(F("[Y HOME] Y FWD -> Y FWD (D24)"));
  digitalWrite(ENABLE_PIN, LOW); digitalWrite(DIR_Y, HIGH); delayMicroseconds(200);
  currentState = S_Y_HOME; lastStepY = micros();
}
void goAct4Extend() {
  Serial.println(F("[ACT4] EXTENDING 25s (final)"));
  actExtend(); actStart = millis(); currentState = S_ACT4_EXTEND;
}
void goAct4Retract() {
  Serial.println(F("[ACT4] RETRACTING 25s (final)"));
  actRetract(); actStart = millis(); currentState = S_ACT4_RETRACT;
}
void goCountdown() {
  swapCompleteTime = millis();
  lastPrintC = millis();  // reset print timer
  Serial.println(F("\n=== CYCLE COMPLETE ==="));
  Serial.println(F("[STANDBY] 7 min before next swap"));
  Serial.println(F("Ultrasonic OFF during standby\n"));
  currentState = S_COUNTDOWN;
}
void goWaitDrone() {
  landCount = 0;
  Serial.println(F("[SCAN] Standby done! Scanning for drone...\n"));
  currentState = S_WAIT_DRONE;
}

// -- Setup ---------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(STEP_X, OUTPUT); pinMode(DIR_X, OUTPUT);
  pinMode(STEP_Y, OUTPUT); pinMode(DIR_Y, OUTPUT);
  pinMode(STEP_Z, OUTPUT); pinMode(DIR_Z, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(STEP_X,LOW); digitalWrite(DIR_X,LOW);
  digitalWrite(STEP_Y,LOW); digitalWrite(DIR_Y,LOW);
  digitalWrite(STEP_Z,LOW); digitalWrite(DIR_Z,LOW);
  digitalWrite(ENABLE_PIN, HIGH);

  pinMode(X_HOME, INPUT_PULLUP);
  pinMode(X_REV,  INPUT_PULLUP);
  pinMode(Y_FWD,  INPUT_PULLUP);
  pinMode(Y_REV,  INPUT_PULLUP);
  pinMode(Z_FWD,  INPUT_PULLUP);
  pinMode(Z_REV,  INPUT_PULLUP);
  delay(100);

  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT);
  actStop();

  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_LOCKED_DEG);

  Serial.println(F("\n================================"));
  Serial.println(F(" ASCENT SWAP SEQUENCE - MEGA"));
  Serial.println(F("================================"));
  Serial.println(F("Auto start | Scan for drone landing"));
  Serial.println(F("Threshold: 50cm | Min swap: 7min"));
  Serial.println(F("--------------------------------"));
  Serial.println(F("S=stop  SW=switches  D=dist  H=help"));
  Serial.println(F("================================\n"));
  printSwitches();

  // Startup: X and Y go to their home switches, then the station scans.
  Serial.println(F("[STARTUP] X Y homing...\n"));
  goStep1();
}

// -- Loop ----------------------------------
void loop() {
  unsigned long nowUs = micros();
  unsigned long nowMs = millis();

  if (Serial.available()) {
    String cmd = "";
    unsigned long t = millis();
    while (millis() - t < 50) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') break;
        cmd += c; t = millis();
      }
    }
    cmd.trim(); cmd.toUpperCase();
    if (cmd.length() == 0) return;

    if (cmd == "S") {
      emergency = true; currentState = IDLE;
      digitalWrite(ENABLE_PIN, HIGH); actStop();
      Serial.print(F("\n[STOP] Total swaps: ")); Serial.println(swapCount);
    }
    else if (cmd == "SW") { printSwitches(); }
    else if (cmd == "D")  { Serial.print(F("US: ")); Serial.print(readDist()); Serial.println(F("cm")); }
    else if (cmd == "H")  { Serial.println(F("S=stop  SW=switches  D=dist  H=help")); }
    else { Serial.print(F("Unknown: ")); Serial.println(cmd); }
  }

  if (emergency) return;

  switch (currentState) {

    case IDLE: break;

    // -- WAIT FOR DRONE LANDING (5 reads) ----
    case S_WAIT_DRONE_ON_PAD: {
      int d = readDist();
      if (nowMs - lastPrintT >= 500) {
        lastPrintT = nowMs;
        Serial.print(F("[LAND] US: ")); Serial.print(d);
        Serial.print(F("cm  confirm: ")); Serial.print(landCount); Serial.println(F("/5"));
      }
      if (d > 0 && d <= DRONE_CM) {
        landCount++;
        if (landCount >= 5) {
          Serial.println(F("[LAND] Drone confirmed (5/5) - waiting 3s...\n"));
          landCount = 0;
          goWait();
        }
      } else {
        landCount = 0;  // reset if any read fails
      }
      delay(200);
      break;
    }

    case S1_X_FWD:
      if (nowUs - lastStepX >= STEP_DELAY_FWD) { lastStepX = nowUs; stepX(); }
      if (switchPressed(X_HOME)) { stopMotors(); Serial.println(F("[1] X HOME OK")); delay(200); goStep2(); }
      break;

    case S2_Y_FWD:
      if (nowUs - lastStepY >= STEP_DELAY_FWD) { lastStepY = nowUs; stepY(); }
      if (switchPressed(Y_FWD)) { stopMotors(); Serial.println(F("[2] Y FWD OK")); delay(200); goScan(); }
      break;

    case S_WAIT:
      if (nowMs - waitStart >= CONFIRM_WAIT_MS) { Serial.println(F("[WAIT] 3s done")); goXRev(); }
      break;

    case S_X_REV:
      if (nowUs - lastStepX >= STEP_DELAY_REV) { lastStepX = nowUs; stepX(); }
      if (switchPressed(X_REV)) { stopMotors(); Serial.println(F("[X REV] OK")); delay(200); goYRev(); }
      break;

    case S_Y_REV:
      if (nowUs - lastStepY >= STEP_DELAY_REV) { lastStepY = nowUs; stepY(); }
      if (switchPressed(Y_REV)) {
        stopMotors(); Serial.println(F("[Y REV] OK")); delay(200);
        if (firstCycle) goZFwdFirst();
        else            goAct1Extend();
      }
      break;

    case S_Z_FWD_FIRST:
      if (nowUs - lastStepZ >= STEP_DELAY_Z) { lastStepZ = nowUs; stepZ(); }
      if (switchPressed(Z_FWD)) { stopMotors(); Serial.println(F("[Z FWD] OK")); delay(200); goAct1Extend(); }
      break;

    case S_ACT1_EXTEND:
      if (nowMs - actStart >= ACT_STROKE_MS) { actStop(); Serial.println(F("[ACT1] extended OK")); goAct1Retract(); }
      break;

    case S_ACT1_RETRACT:
      if (nowMs - actStart >= ACT_STROKE_MS) { actStop(); Serial.println(F("[ACT1] retracted OK")); goZCheck(); }
      break;

    case S_Z_CHECK:
      if (nowUs - lastStepZ >= STEP_DELAY_Z) { lastStepZ = nowUs; stepZ(); }
      {
        int targetPin = zMoveDir ? Z_FWD : Z_REV;
        if (switchPressed(targetPin)) {
          stopMotors();
          Serial.print(F("[Z] ")); Serial.println(zMoveDir ? F("FWD OK") : F("REV OK"));
          delay(200); goAct2Extend();
        }
      }
      break;

    case S_ACT2_EXTEND:
      if (nowMs - actStart >= ACT_STROKE_MS) { actStop(); Serial.println(F("[ACT2] extended OK")); goAct2Retract(); }
      break;

    case S_ACT2_RETRACT:
      if (nowMs - actStart >= ACT_STROKE_MS) { actStop(); Serial.println(F("[ACT2] retracted OK")); goYHome(); }
      break;

    case S_Y_HOME:
      if (nowUs - lastStepY >= STEP_DELAY_FWD) { lastStepY = nowUs; stepY(); }
      if (switchPressed(Y_FWD)) {
        stopMotors(); Serial.println(F("[Y HOME] OK")); delay(200);
        Serial.println(F("[X HOME] X FWD -> X HOME (A3)"));
        digitalWrite(ENABLE_PIN, LOW); digitalWrite(DIR_X, HIGH); delayMicroseconds(200);
        currentState = S_X_HOME; lastStepX = micros();
      }
      break;

    case S_X_HOME:
      if (nowUs - lastStepX >= STEP_DELAY_FWD) { lastStepX = nowUs; stepX(); }
      if (switchPressed(X_HOME)) {
        stopMotors(); Serial.println(F("[X HOME] OK"));
        swapCount++; firstCycle = false;
        swapCompleteTime = millis();
        Serial.print(F("[FINAL] Swap #")); Serial.println(swapCount);
        myServo.write(SERVO_LOCKED_DEG);
        goAct4Extend();
      }
      break;

    case S_ACT4_EXTEND:
      if (nowMs - actStart >= ACT_STROKE_MS) { actStop(); Serial.println(F("[ACT4] extended OK")); goAct4Retract(); }
      break;

    case S_ACT4_RETRACT:
      if (nowMs - actStart >= ACT_STROKE_MS) { actStop(); Serial.println(F("[ACT4] retracted OK")); goCountdown(); }
      break;

    case S_COUNTDOWN:
      if (nowMs - lastPrintC >= 10000) {
        lastPrintC = nowMs;
        unsigned long elapsed   = nowMs - swapCompleteTime;
        unsigned long remaining = (elapsed < MIN_SWAP_MS) ? (MIN_SWAP_MS - elapsed) : 0;
        Serial.print(F("[STANDBY] Next swap in: "));
        Serial.print(remaining / 60000); Serial.print(F("m "));
        Serial.print((remaining % 60000) / 1000); Serial.println(F("s"));
      }
      if (nowMs - swapCompleteTime >= MIN_SWAP_MS) { goWaitDrone(); }
      break;

    case S_WAIT_DRONE: {
      int d = readDist();
      if (nowMs - lastPrintT >= 500) {
        lastPrintT = nowMs;
        Serial.print(F("[SCAN] ")); Serial.print(d);
        Serial.print(F("cm  confirm: ")); Serial.print(landCount); Serial.println(F("/5"));
      }
      if (d > 0 && d <= DRONE_CM) {
        landCount++;
        if (landCount >= 5) {
          Serial.println(F("[SCAN] Drone confirmed (5/5)!"));
          landCount = 0;
          goWait();
        }
      } else {
        landCount = 0;
      }
      delay(200);
      break;
    }
  }
}
