#include <Arduino.h>
/*
 * 4-DOF robot arm - button control + automatic pick-and-place (Arduino Uno, 6 servos, 9 buttons)
 * Joint angles are read back from rotation matrices: theta -> R(theta) -> atan2 -> servo angle.
 *
 * Servos (Servo library):
 *   Base      pin 3                     Rz
 *   Shoulder  pin 5 + pin 6 (mirrored)  Ry
 *   Elbow     pin 9                     Ry
 *   Gripper   pin 10 + pin 11 (mirrored) Rz (both jaws open/close symmetrically)
 *
 * Buttons (one leg to the pin, the other leg to GND; INPUT_PULLUP, no resistor needed):
 *   Base      left(+) 2     | right(-) 4
 *   Shoulder  forward(+) 7  | back(-) 8
 *   Elbow     down(+) 12    | up(-) 13
 *   Gripper   open(+) A0    | close(-) A1
 *   Automatic task (start / stop): A2
 *
 * Task: pick up 3 cubes (red, green, blue) from their start positions and carry each to its own target.
 * Positions are set by the constants below (mm and degrees). Adjust them to your own arm.
 *
 * Serial monitor (9600): j <base|shoulder|elbow|grip> <degrees> | fk | home | task | stop
 *                        setstart (save the current pose as the power-on pose) | resetstart (back to default)
 *
 * Start pose: at power-on the servos go straight to this pose without sweeping.
 * Move the arm upright with the buttons, type "setstart" in the serial monitor: it will always start there.
 */
#include <Servo.h>
#include <EEPROM.h>
#include <math.h>

struct Mat3 { float m[3][3]; };

struct Joint {
  const char *name;
  char axis;
  int pin, pin2;           // pin2: second, mirrored servo (-1 if none)
  int btnPlus, btnMinus;   // button pins
  float minDeg, maxDeg;
  float deg;               // target joint angle (degrees)
  float servoNow;          // servo angle (degrees)
  bool moving;
  Mat3 R;
};

Mat3 mul(const Mat3 &a, const Mat3 &b) {
  Mat3 r;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      r.m[i][j] = 0;
      for (int k = 0; k < 3; k++) r.m[i][j] += a.m[i][k] * b.m[k][j];
    }
  return r;
}

void mulVec(const Mat3 &a, const float v[3], float out[3]) {
  for (int i = 0; i < 3; i++)
    out[i] = a.m[i][0] * v[0] + a.m[i][1] * v[1] + a.m[i][2] * v[2];
}

Mat3 Rx(float t) { float c = cosf(t), s = sinf(t); Mat3 r = {{{1,0,0},{0,c,-s},{0,s,c}}}; return r; }
Mat3 Ry(float t) { float c = cosf(t), s = sinf(t); Mat3 r = {{{c,0,s},{0,1,0},{-s,0,c}}}; return r; }
Mat3 Rz(float t) { float c = cosf(t), s = sinf(t); Mat3 r = {{{c,-s,0},{s,c,0},{0,0,1}}}; return r; }

Mat3 makeRot(char axis, float t) {
  if (axis == 'x') return Rx(t);
  if (axis == 'y') return Ry(t);
  return Rz(t);
}

float readAngle(char axis, const Mat3 &r) {
  if (axis == 'x') return atan2f(r.m[2][1], r.m[1][1]);
  if (axis == 'y') return atan2f(r.m[0][2], r.m[0][0]);
  return atan2f(r.m[1][0], r.m[0][0]);
}


Joint joints[4] = {
  {"base",     'z',  3, -1,  2,  4, -90,  90, 0, 90, false, {}},
  {"shoulder", 'y',  5,  6,  7,  8, -60,  80, 0, 90, false, {}},
  {"elbow",    'y',  9, -1, 12, 13, -80,  80, 0, 90, false, {}},
  {"grip",     'z', 10, 11, A0, A1,   0,  60, 0, 90, false, {}},
};

Servo svA[4], svB[4];
const float L1 = 60, L2 = 55, L3 = 75;   // mm
// ---------------- Start pose (stored in EEPROM) ----------------
float startDeg[4] = {0, 0, 0, 0};          // base, shoulder, elbow, gripper (joint angle, degrees)
const byte EE_MAGIC = 0xA7;

void loadStart() {
  if (EEPROM.read(0) != EE_MAGIC) return;   // nothing saved yet: default (0,0,0,0)
  for (int i = 0; i < 4; i++) EEPROM.get(1 + i * sizeof(float), startDeg[i]);
}
void saveStart(const float d[4]) {
  EEPROM.update(0, EE_MAGIC);
  for (int i = 0; i < 4; i++) { startDeg[i] = d[i]; EEPROM.put(1 + i * sizeof(float), startDeg[i]); }
}
void clearStart() { EEPROM.update(0, 0); for (int i = 0; i < 4; i++) startDeg[i] = 0; }

const float STEP = 0.7f;                 // degrees / 15 ms  (~46 degrees/s)

// ---------------- Task settings ----------------
const int   BTN_AUTO  = A2;
const float PIVOT_H   = 71;     // height of the shoulder axis above the table (mm)
const float GRIP_X    = 115;    // grasp point: along the forearm axis (mm)
const float GRIP_Z    = 11.5;   // grasp point: above the forearm plane (mm)
const float CUBE_HALF = 5;      // half the cube height (cube is 10 mm)
const float HOVER_Z   = 45;     // hover height above the cube (mm)
const float OPEN_DEG  = 30;     // gripper open angle
// Cubes are at the same distance from the base centre (120 mm) in different directions (degrees, left = +)
const float OBJ_YAW[3] = {-70, -40, -10};   // position of the red, green, blue cube
const float TGT_YAW[3] = { 55,  85,  25};   // target of the red, green, blue cube

void printRot(const Joint &j) {
  float t = j.deg * DEG_TO_RAD;
  Serial.print(F("\nR")); Serial.print(j.axis); Serial.print(F("(theta) ["));
  Serial.print(j.name); Serial.print(F("]  theta = ")); Serial.print(j.deg, 0); Serial.println(F(" deg"));
  for (int r = 0; r < 3; r++) {
    Serial.print(F("  [ "));
    for (int c = 0; c < 3; c++) { Serial.print(j.R.m[r][c], 2); Serial.print(' '); }
    Serial.println(']');
  }
  Serial.print(F("  cos = ")); Serial.print(cosf(t), 2);
  Serial.print(F("   sin = ")); Serial.println(sinf(t), 2);
  if (j.axis == 'x') Serial.println(F("  Invariant axis: X"));
  if (j.axis == 'y') Serial.println(F("  Invariant axis: Y"));
  if (j.axis == 'z') Serial.println(F("  Invariant axis: Z"));
}

void setJoint(int i, float deg, bool verbose) {
  Joint &j = joints[i];
  j.deg = constrain(deg, j.minDeg, j.maxDeg);
  j.R = makeRot(j.axis, j.deg * DEG_TO_RAD);
  if (verbose) printRot(j);
}

// ---------------- Inverse kinematics (shoulder + elbow, in the arm plane) ----------------
// Grasp point in the elbow frame: (GRIP_X, 0, GRIP_Z). R03 = Ry(shoulder + elbow).
void gripPlanar(float t2, float t3, float &r, float &z) {
  float phi = t2 + t3, c = cosf(phi), s = sinf(phi);
  r = L1 * sinf(t2) + GRIP_X * c + GRIP_Z * s;
  z = L1 * cosf(t2) - GRIP_X * s + GRIP_Z * c;
}

// Shoulder/elbow angles for a given horizontal distance (rTarget) and height above the table (zAbs).
bool solveIK(float rTarget, float zAbs, float &d2, float &d3) {
  float zr = zAbs - PIVOT_H, best = 1e9, b2 = 0, b3 = 0, r, z;
  for (int a = -60; a <= 80; a += 4)
    for (int b = -80; b <= 80; b += 4) {
      gripPlanar(a * DEG_TO_RAD, b * DEG_TO_RAD, r, z);
      float e = fabsf(r - rTarget) + fabsf(z - zr);
      if (e < best) { best = e; b2 = a; b3 = b; }
    }
  float c2 = b2, c3 = b3;
  for (float a = constrain(c2 - 4, -60.0f, 80.0f); a <= constrain(c2 + 4, -60.0f, 80.0f); a += 1)
    for (float b = constrain(c3 - 4, -80.0f, 80.0f); b <= constrain(c3 + 4, -80.0f, 80.0f); b += 1) {
      gripPlanar(a * DEG_TO_RAD, b * DEG_TO_RAD, r, z);
      float e = fabsf(r - rTarget) + fabsf(z - zr);
      if (e < best) { best = e; b2 = a; b3 = b; }
    }
  d2 = b2; d3 = b3;
  return best < 6;
}

// ---------------- Automatic task ----------------
const int NWP = 3 * 8 + 1;
float wp[NWP][4];           // base, shoulder, elbow, gripper (degrees)
bool autoOn = false, wpSet = false;
int wpIdx = 0;
unsigned long reachedAt = 0;

void addWp(int &n, float yaw, float d2, float d3, float grip) {
  wp[n][0] = yaw; wp[n][1] = d2; wp[n][2] = d3; wp[n][3] = grip; n++;
}

void buildWaypoints() {
  float h2, h3, f2, f3;
  const float R = 120;
  if (!solveIK(R, HOVER_Z, h2, h3))   Serial.println(F("IK: hover point unreachable"));
  if (!solveIK(R, CUBE_HALF, f2, f3)) Serial.println(F("IK: floor point unreachable"));
  int n = 0;
  for (int k = 0; k < 3; k++) {
    addWp(n, OBJ_YAW[k], h2, h3, OPEN_DEG);   // move above the cube, open the gripper
    addWp(n, OBJ_YAW[k], f2, f3, OPEN_DEG);   // go down
    addWp(n, OBJ_YAW[k], f2, f3, 0);          // grip
    addWp(n, OBJ_YAW[k], h2, h3, 0);          // lift
    addWp(n, TGT_YAW[k], h2, h3, 0);          // move above the target
    addWp(n, TGT_YAW[k], f2, f3, 0);          // go down
    addWp(n, TGT_YAW[k], f2, f3, OPEN_DEG);   // release
    addWp(n, TGT_YAW[k], h2, h3, OPEN_DEG);   // go up
  }
  addWp(n, startDeg[0], startDeg[1], startDeg[2], startDeg[3]);   // back to the start pose
}

bool allReached() {
  for (int i = 0; i < 4; i++) {
    float target = 90 + readAngle(joints[i].axis, joints[i].R) * RAD_TO_DEG;
    if (fabsf(target - joints[i].servoNow) > 0.6f) return false;
  }
  return true;
}

void startAuto() {
  buildWaypoints();
  autoOn = true; wpIdx = 0; wpSet = false;
  Serial.println(F("Automatic task started"));
}

void stopAuto(const __FlashStringHelper *why) {
  if (!autoOn) return;
  autoOn = false;
  Serial.println(why);
}

void stepAuto() {
  if (!autoOn) return;
  if (!wpSet) {
    for (int i = 0; i < 4; i++) setJoint(i, wp[wpIdx][i], false);
    wpSet = true; reachedAt = 0;
    Serial.print(F("step ")); Serial.print(wpIdx + 1); Serial.print('/'); Serial.println(NWP);
  } else if (allReached()) {
    if (reachedAt == 0) reachedAt = millis();
    else if (millis() - reachedAt > 250) {
      wpIdx++; wpSet = false;
      if (wpIdx >= NWP) { autoOn = false; Serial.println(F("Task finished")); }
    }
  }
}

// ---------------- Buttons, servos, serial ----------------
void readButtons() {
  for (int i = 0; i < 4; i++) {
    Joint &j = joints[i];
    bool plus  = digitalRead(j.btnPlus)  == LOW;
    bool minus = digitalRead(j.btnMinus) == LOW;
    if (plus != minus) {
      stopAuto(F("Manual input: automatic task stopped"));
      setJoint(i, j.deg + (plus ? STEP : -STEP), false);
      j.moving = true;
    } else if (j.moving) {          // button released: print the matrix
      j.moving = false;
      printRot(j);
    }
  }
  static bool prevAuto = false;
  bool a = digitalRead(BTN_AUTO) == LOW;
  if (a && !prevAuto) { if (autoOn) stopAuto(F("Automatic task stopped")); else startAuto(); }
  prevAuto = a;
}

void updateServos() {
  for (int i = 0; i < 4; i++) {
    Joint &j = joints[i];
    float target = 90 + readAngle(j.axis, j.R) * RAD_TO_DEG;
    float diff = target - j.servoNow;
    if (fabsf(diff) > 0.5f) j.servoNow += constrain(diff, -1.5f, 1.5f);
    int a = (int)(j.servoNow + 0.5f);
    svA[i].write(a);
    if (j.pin2 >= 0) svB[i].write(180 - a);
  }
}

void printFK() {
  Mat3 R02 = mul(joints[0].R, joints[1].R);
  Mat3 R03 = mul(R02, joints[2].R);
  const float up[3] = {0, 0, L1}, fw[3] = {L2, 0, 0}, tip[3] = {L3, 0, 0};
  float p1[3], p2[3], p3[3];
  mulVec(R02, up, p1); mulVec(R03, fw, p2); mulVec(R03, tip, p3);
  Serial.println(F("\nR_0_3 = Rz * Ry * Ry"));
  for (int r = 0; r < 3; r++) {
    Serial.print(F("  [ "));
    for (int c = 0; c < 3; c++) { Serial.print(R03.m[r][c], 2); Serial.print(' '); }
    Serial.println(']');
  }
  Serial.print(F("Tip position (mm): x=")); Serial.print(p1[0] + p2[0] + p3[0], 1);
  Serial.print(F(" y=")); Serial.print(p1[1] + p2[1] + p3[1], 1);
  Serial.print(F(" z=")); Serial.println(p1[2] + p2[2] + p3[2], 1);
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line == "fk") { printFK(); return; }
  if (line == "task") { startAuto(); return; }
  if (line == "stop") { stopAuto(F("Automatic task stopped")); return; }
  if (line == "home") { stopAuto(F("Automatic task stopped")); for (int i = 0; i < 4; i++) setJoint(i, startDeg[i], true); return; }
  if (line == "setstart") {
    float d[4];
    for (int i = 0; i < 4; i++) d[i] = joints[i].deg;
    saveStart(d);
    Serial.println(F("Start pose saved (the arm will go straight here at power-on)."));
    return;
  }
  if (line == "resetstart") { clearStart(); Serial.println(F("Start pose reset to default (0,0,0,0).")); return; }
  if (line.startsWith("j ")) {
    int sp = line.indexOf(' ', 2);
    if (sp < 0) return;
    String name = line.substring(2, sp);
    float deg = line.substring(sp + 1).toFloat();
    for (int i = 0; i < 4; i++)
      if (name == joints[i].name) { stopAuto(F("Manual input: automatic task stopped")); setJoint(i, deg, true); return; }
    Serial.println(F("Unknown joint: base | shoulder | elbow | grip"));
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(BTN_AUTO, INPUT_PULLUP);
  for (int i = 0; i < 4; i++) {
    pinMode(joints[i].btnPlus, INPUT_PULLUP);
    pinMode(joints[i].btnMinus, INPUT_PULLUP);
  }
  loadStart();
  for (int i = 0; i < 4; i++) {
    setJoint(i, startDeg[i], false);
    // send the servo straight to the start angle (no sweep)
    joints[i].servoNow = 90 + readAngle(joints[i].axis, joints[i].R) * RAD_TO_DEG;
    int a = (int)(joints[i].servoNow + 0.5f);
    svA[i].write(a);
    svA[i].attach(joints[i].pin);
    svA[i].write(a);
    if (joints[i].pin2 >= 0) { svB[i].write(180 - a); svB[i].attach(joints[i].pin2); svB[i].write(180 - a); }
  }
  Serial.println(F("Robot arm ready. Hold a button to move. A2: automatic task. Serial: j | fk | home | task | stop | setstart"));
}

void loop() {
  handleSerial();
  readButtons();
  stepAuto();
  updateServos();
  delay(15);
}
