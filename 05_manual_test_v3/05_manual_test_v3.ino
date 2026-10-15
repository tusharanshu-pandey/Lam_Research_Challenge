/*
 * LAM Stage 2 - BRING-UP 05 v3 : MANUAL TOGGLE TEST + FLOW READOUT + FLOAT SWITCH
 * Pin map from "LAM Stage 2 PCB", T-Works Foundation, REV 1.0
 *
 * Serial Monitor @ 115200, line ending "New Line".
 *
 *   1   toggle Pump 1 (P-01)
 *   2   toggle Pump 2 (P-02)
 *   3   toggle Pump 3 (P-03)
 *   a   toggle solenoid valve OV-01   (on/off only)
 *   f   toggle the flow readout on / off
 *   l   toggle LEVEL PROTECTION on / off      <-- new  (starts OFF)
 *   w   live RAW view of the ON/OFF switch + float   <-- new
 *   s   STOP everything immediately
 *   k   calibration marker: start / stop
 *   z   zero the totaliser
 *   d1 75   set pump 1 duty (d2 / d3 for the others)
 *   ?   help and current status
 *
 * SEVERAL COMMANDS AT ONCE: separate them with a space, comma, '&' or '+' and they all
 * run together in the order written -   "2 f"   "2&f"   "2, 1, a, f"   "d1 60 1 f"
 * Handy for starting a pump and the flow clock in the same keystroke, so the readout's
 * t = 0 lines up with the pump starting.
 *
 * THE FLOAT IS ALWAYS MONITORED. Every change of state is printed, whether or not
 * protection is enabled - so you can check the polarity and trip height by hand
 * without the pumps being switched off underneath you.
 *
 * With 'l' ON, the float behaves as it will in the real Case 1 demo:
 *   float HIGH  -> the PROTECTED pump is switched off immediately, the others keep running
 *   float clear -> it restarts after FLOAT_CLEAR_HOLD_MS
 *
 * PROTECTED_PUMP (below) selects which driver channel the float cuts. It must be the
 * pump that FILLS the sump; the pump that empties the sump has to keep running or the
 * level never falls.
 *
 * NOTE ON P-02: this board is missing the R_PWM2 trace, so pump 2 is driven on
 * L_PWM2. Its PWM pins are swapped in pumps[] below, so the pump turns the OTHER WAY
 * - swap M+ / M- at that driver's screw terminals.
 */

#include <Arduino.h>

// ===================== PIN MAP (schematic REV 1.0) =====================
const int PIN_P1_RPWM = 33;
const int PIN_P1_LPWM = 32;
const int PIN_P1_REN  = 26;
const int PIN_P1_LEN  = 13;

const int PIN_P2_RPWM = 19;   // trace missing on this PCB - not usable
const int PIN_P2_LPWM = 18;   // used as the PWM line for P-02 instead
const int PIN_P2_REN  = 17;
const int PIN_P2_LEN  = 16;

const int PIN_P3_RPWM = 25;
const int PIN_P3_LPWM = 23;
const int PIN_P3_REN  = 22;
const int PIN_P3_LEN  = 21;

const int PIN_OV    = 27;     // SOLENOID_VALVE JST pin 2
const int PIN_FLOW  = 14;     // FLOW_SENSOR JST pin 2 (FS)
const int PIN_FLOAT = 35;     // FLOAT_SWITCH (G35, input-only, 10k pull-up R10 on PCB)
const int PIN_SW    = 34;     // ON/OFF_SWITCH (G34, input-only, 10k pull-up R2 on PCB)
const int PIN_LED   =  2;

const bool OV_ON_STATE = HIGH;

// Pin level when the float says "sump level HIGH".
// The JST's other pin is GND and the PCB pulls the line up, so a CLOSED contact = LOW.
// If P-01 cuts out when the sump is EMPTY, change this to HIGH.
const bool FLOAT_HIGH_LEVEL_STATE = LOW;

// Same arrangement on the ON/OFF control: R2 pulls G34 up to 3V3, the other side
// of the JST goes to GND, so a MADE contact reads LOW.
//
// NOTE: this is a MOMENTARY push button (TSL-012 illuminated tactile), not a
// latching toggle - it only reads LOW while a finger is on it. The demo firmware
// therefore latches run/stop in software from press events. This sketch just
// watches it, so a press here shows up as a press and a release.
const bool SWITCH_PRESSED_STATE = LOW;

// ===================== SETTINGS =====================
const uint32_t PWM_HZ    = 10000;
const uint8_t  PWM_BITS  = 10;
const uint32_t MAX_RUN_S = 120;
const uint32_t WINDOW_MS       = 500;    // measurement / print window
const uint32_t HEADER_EVERY_MS = 10000;  // reprint the column header this often

const uint32_t FLOAT_DEBOUNCE_MS   = 300;
const uint32_t FLOAT_CLEAR_HOLD_MS = 3000;   // level must stay clear this long before it restarts

const uint32_t SW_DEBOUNCE_MS = 50;          // the PCB already has R2/C5, so this is short
const uint32_t SW_MONITOR_MS  = 200;         // 'w' live raw view refresh

// WHICH PUMP THE FLOAT CUTS.  0 = pumps[0] "P-01", 1 = pumps[1] "P-02", 2 = pumps[2] "P-03".
// Set this to whichever driver channel your FILLING pump (the one feeding the sump) is
// actually plugged into. The pump that EMPTIES the sump must keep running, or the level
// never falls and the float never clears.
const int PROTECTED_PUMP = 2;

float CAL_A = 0.0133309f;
float CAL_B = -0.38390f;

uint8_t defaultDuty[3] = {50, 100, 50};

// ===================== internals =====================
struct Pump {
  const char* name;
  int rpwm, lpwm, ren, len, ch;
  bool running;          // what the operator asked for
  uint8_t duty;
  uint32_t startedAt;
};

// P-02 has its PWM pins SWAPPED on purpose (missing R_PWM2 trace)
Pump pumps[3] = {
  {"P-01", PIN_P1_RPWM, PIN_P1_LPWM, PIN_P1_REN, PIN_P1_LEN, 0, false, 50,  0},
  {"P-02", PIN_P2_LPWM, PIN_P2_RPWM, PIN_P2_REN, PIN_P2_LEN, 1, false, 100, 0},
  {"P-03", PIN_P3_RPWM, PIN_P3_LPWM, PIN_P3_REN, PIN_P3_LEN, 2, false, 50,  0}
};

bool ovOpen    = false;
bool showFlow  = false;
bool levelProt = false;       // 'l' toggles; starts OFF so bring-up is not disturbed

// float state
bool     floatHigh = false, floatCand = false;
uint32_t floatChangeT = 0, floatClearSince = 0;
bool     p1Allowed = true;    // false while the level protection is holding the protected pump off

// ON/OFF switch state. Monitored only - this sketch never acts on it. The point
// is to prove the switch and its wiring work before the demo firmware relies on it.
bool     swOn = false, swCand = false;
uint32_t swChangeT = 0, swEdges = 0, lastMonMs = 0, pressStart = 0;
bool     swMonitor = false;   // 'w' toggles the live raw view

volatile uint32_t gPulses     = 0;
volatile uint32_t gLastEdgeUs = 0;

void IRAM_ATTR onFlowPulse() {
  uint32_t now = micros();
  if (now - gLastEdgeUs < 300) return;
  gLastEdgeUs = now;
  gPulses++;
}

uint32_t lastWinMs = 0, lastWinPulses = 0;
uint32_t lastHeaderMs = 0;    // when the column header was last reprinted
uint32_t flowStartMs = 0;     // set when 'f' turns the readout on; time column counts from here
double   volTotalL = 0;
bool     calRunning = false;
uint32_t calP0 = 0, calT0 = 0;

// The protected pump runs only if the operator asked AND the level allows it
bool pumpShouldRun(int i) {
  if (!pumps[i].running) return false;
  if (i == PROTECTED_PUMP && levelProt && !p1Allowed) return false;
  return true;
}

void applyPump(int i) {
  Pump &p = pumps[i];
  uint8_t pct = pumpShouldRun(i) ? p.duty : 0;
  uint32_t maxv = (1u << PWM_BITS) - 1;
  uint32_t duty = (maxv * pct) / 100;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(p.rpwm, duty);
#else
  ledcWrite(p.ch, duty);
#endif

  digitalWrite(p.lpwm, LOW);
  digitalWrite(p.ren, pct > 0 ? HIGH : LOW);
  digitalWrite(p.len, pct > 0 ? HIGH : LOW);
}

void applyAll() { for (int i = 0; i < 3; i++) applyPump(i); }

void togglePump(int i) {
  Pump &p = pumps[i];
  p.running = !p.running;
  p.startedAt = p.running ? millis() : 0;
  applyPump(i);
  Serial.printf("# %s %s (duty %u %%)%s\n", p.name, p.running ? "START" : "STOP", p.duty,
                (i == PROTECTED_PUMP && p.running && levelProt && !p1Allowed)
                  ? "  [HELD OFF by level protection]" : "");
}

void stopAll() {
  for (int i = 0; i < 3; i++) { pumps[i].running = false; pumps[i].startedAt = 0; applyPump(i); }
  ovOpen = false;
  digitalWrite(PIN_OV, !OV_ON_STATE);
  Serial.println("# *** ALL STOPPED - pumps off, valve closed ***");
}

void toggleValve() {
  ovOpen = !ovOpen;
  digitalWrite(PIN_OV, ovOpen ? OV_ON_STATE : !OV_ON_STATE);
  Serial.printf("# OV-01 %s\n", ovOpen ? "OPEN - coil energised" : "CLOSED - coil off");
}

void toggleFlow() {
  showFlow = !showFlow;
  if (showFlow) {
    lastWinMs = millis(); lastWinPulses = gPulses;
    flowStartMs = lastWinMs;          // time column restarts at 0 every time 'f' is pressed
    lastHeaderMs = lastWinMs - HEADER_EVERY_MS;   // force the header on the first line
    Serial.println("# FLOW READOUT ON - send 'f' again to stop (t restarts at 0)");
  } else Serial.println("# FLOW READOUT OFF");
}

void toggleLevelProt() {
  levelProt = !levelProt;
  if (levelProt) {
    p1Allowed = !floatHigh;
    floatClearSince = 0;
    Serial.printf("# LEVEL PROTECTION ON  - float is currently %s\n", floatHigh ? "HIGH (P-01 held off)" : "clear");
  } else {
    Serial.println("# LEVEL PROTECTION OFF - float is monitored but ignored");
  }
  applyAll();
}

// The ON/OFF switch is ALWAYS monitored, exactly like the float. Every debounced
// change prints with a running count, so plugging the switch in and flicking it a
// few times is enough to prove the wiring - no command needed.
//
// 'w' adds a live RAW view that bypasses the debounce. Use that one to catch a
// dirty or intermittent contact: a good switch shows a clean CLOSED/open that
// follows your hand, a bad one flickers or never changes at all.
void updateSwitch(uint32_t now) {
  bool raw = (digitalRead(PIN_SW) == SWITCH_PRESSED_STATE);
  if (raw != swCand) { swCand = raw; swChangeT = now; }

  if (now - swChangeT >= SW_DEBOUNCE_MS && swOn != swCand) {
    swOn = swCand;
    if (swOn) {
      swEdges++;
      pressStart = now;
      Serial.printf("# BUTTON pressed    (press #%lu)\n", (unsigned long)swEdges);
    } else {
      Serial.printf("# BUTTON released   (held %lu ms)\n", (unsigned long)(now - pressStart));
    }
  }

  if (swMonitor && now - lastMonMs >= SW_MONITOR_MS) {
    lastMonMs = now;
    int rs = digitalRead(PIN_SW), rf = digitalRead(PIN_FLOAT);
    Serial.printf("  RAW   SW(G34)=%d %-7s   FSW(G35)=%d %-6s   presses=%lu\n",
                  rs, rs == SWITCH_PRESSED_STATE   ? "PRESSED" : "up",
                  rf, rf == FLOAT_HIGH_LEVEL_STATE ? "CLOSED"  : "open",
                  (unsigned long)swEdges);
  }
}

void updateFloat(uint32_t now) {
  bool raw = (digitalRead(PIN_FLOAT) == FLOAT_HIGH_LEVEL_STATE);
  if (raw != floatCand) { floatCand = raw; floatChangeT = now; }

  if (now - floatChangeT >= FLOAT_DEBOUNCE_MS && floatHigh != floatCand) {
    floatHigh = floatCand;
    Serial.printf("# FLOAT -> %s\n", floatHigh ? "HIGH (sump full)" : "clear");
  }

  bool before = p1Allowed;
  if (floatHigh) { p1Allowed = false; floatClearSince = 0; }
  else if (!p1Allowed) {
    if (floatClearSince == 0) floatClearSince = now;
    if (now - floatClearSince >= FLOAT_CLEAR_HOLD_MS) p1Allowed = true;
  }

  if (before != p1Allowed && levelProt) {
    Serial.printf("# LEVEL PROTECTION: %s %s\n", pumps[PROTECTED_PUMP].name,
                  p1Allowed ? "released - restarting" : "CUT OFF");
  }
  if (before != p1Allowed) applyPump(PROTECTED_PUMP);
}

void updateFlow(uint32_t now) {
  uint32_t dt = now - lastWinMs;
  if (dt < WINDOW_MS) return;

  uint32_t p  = gPulses;
  uint32_t dN = p - lastWinPulses;
  lastWinPulses = p; lastWinMs = now;

  float freqHz = dN * 1000.0f / dt;
  float Qlmin  = (dN == 0) ? 0.0f : CAL_A * freqHz + CAL_B;
  if (Qlmin < 0) Qlmin = 0;
  volTotalL += Qlmin * dt / 60000.0;

  if (showFlow) {
    // repeat the column header every HEADER_EVERY_MS so long runs stay readable
    if (now - lastHeaderMs >= HEADER_EVERY_MS) {
      lastHeaderMs = now;
      Serial.println("#   time_s   pulses   freq_Hz   Q_L/min    Q_mL/s    total_L  float  sw");
    }

    Serial.printf("  %7.1f  %7lu  %8.1f  %8.3f  %8.2f  %9.4f   %d     %d\n",
                  (now - flowStartMs) / 1000.0, (unsigned long)p, freqHz,
                  Qlmin, Qlmin * 1000.0f / 60.0f, volTotalL,
                  floatHigh ? 1 : 0, swOn ? 1 : 0);
  }
}

void status() {
  Serial.println("# ---------------- status ----------------");
  for (int i = 0; i < 3; i++) {
    bool actually = pumpShouldRun(i);
    Serial.printf("#   %s : %-7s  duty %3u %%%s\n",
                  pumps[i].name, pumps[i].running ? "RUNNING" : "off", pumps[i].duty,
                  (pumps[i].running && !actually) ? "   [held off]" : "");
  }
  Serial.printf("#   OV-01: %s\n", ovOpen ? "OPEN" : "closed");
  Serial.printf("#   float: %s | level protection: %s (cuts %s)\n",
                floatHigh ? "HIGH" : "clear", levelProt ? "ON" : "off",
                pumps[PROTECTED_PUMP].name);
  Serial.printf("#   ON/OFF button: %s  (raw G34 = %d, %lu press%s since boot)%s\n",
                swOn ? "PRESSED" : "up", digitalRead(PIN_SW),
                (unsigned long)swEdges, swEdges == 1 ? "" : "es",
                swEdges == 0 ? "   <-- never pressed: press it, or send 'w'" : "");
  Serial.printf("#   flow readout: %s | total %.4f L\n", showFlow ? "ON" : "off", volTotalL);
  Serial.println("# keys: 1 2 3 = pumps | a = valve | f = flow | l = level protection");
  Serial.println("#       w = raw switch/float view | s = stop all | k = cal marker");
  Serial.println("#       z = zero | d1 75 = duty | ? = help");
  Serial.println("#   combine with space / & / + :  \"2 f\"  \"2&f\"  \"d1 60 1 f\"");
  Serial.println("# ----------------------------------------");
}

// ---- command handling -------------------------------------------------------
// One command. Returns false if it was not recognised.
bool runCommand(const String &s, int dutyArg) {
  if      (s == "1") togglePump(0);
  else if (s == "2") togglePump(1);
  else if (s == "3") togglePump(2);
  else if (s == "a") toggleValve();
  else if (s == "f") toggleFlow();
  else if (s == "l") toggleLevelProt();
  else if (s == "s") stopAll();
  else if (s == "?") status();
  else if (s == "z") { volTotalL = 0; Serial.println("# totaliser zeroed"); }
  else if (s == "w") {
    swMonitor = !swMonitor;
    if (swMonitor) {
      lastMonMs = 0;
      Serial.println("# RAW BUTTON MONITOR ON - press and hold the button, the line should follow it");
      Serial.println("#   SW  PRESSED = finger down    up = released / not plugged in");
      Serial.println("#   if it never changes: check the JST, and that it is in ON/OFF_SWITCH not FLOAT_SWITCH");
    } else Serial.println("# RAW SWITCH MONITOR OFF");
  }
  else if (s == "k") {
    if (!calRunning) {
      calRunning = true; calP0 = gPulses; calT0 = millis();
      Serial.println("# CAL START - put the hose in the container NOW");
    } else {
      uint32_t N = gPulses - calP0;
      float    T = (millis() - calT0) / 1000.0f;
      calRunning = false;
      Serial.printf("# CAL STOP, pulses=%lu, time_s=%.2f, freq_Hz=%.2f\n",
                    (unsigned long)N, T, T > 0 ? N / T : 0.0f);
      Serial.println("# weigh the water (1 L = 1.000 kg) and add a row to trials.csv");
    }
  }
  else if (s.length() >= 2 && s.charAt(0) == 'd') {
    int n = s.charAt(1) - '1';
    int v = (s.length() > 2) ? s.substring(2).toInt() : dutyArg;
    if (n >= 0 && n <= 2 && v >= 0 && v <= 100) {
      pumps[n].duty = v;
      if (pumps[n].running) applyPump(n);
      Serial.printf("# %s duty -> %d %%\n", pumps[n].name, v);
    } else Serial.println("# usage: d1 75   (pump 1..3, duty 0..100)");
  }
  else return false;
  return true;
}

// A whole line. Several commands can be given at once, separated by spaces, commas,
// '&' or '+'  -  e.g.  "2 f"   "2&f"   "2, 1, a, f"   "d1 60 1 f"
// They run in the order written, within the same pass of loop(), so a pump start and
// the flow clock reset land effectively together.
void runCommandLine(const String &line) {
  int i = 0, n = line.length();
  String tokens[12];
  int count = 0;

  while (i < n && count < 12) {
    char c = line.charAt(i);
    if (c == ' ' || c == '\t' || c == ',' || c == '&' || c == '+') { i++; continue; }
    int start = i;
    while (i < n) {
      char d = line.charAt(i);
      if (d == ' ' || d == '\t' || d == ',' || d == '&' || d == '+') break;
      i++;
    }
    tokens[count++] = line.substring(start, i);
  }

  for (int t = 0; t < count; t++) {
    String tok = tokens[t];

    // "d1 75" arrives as two tokens - pair them up
    int arg = -1;
    if (tok.length() == 2 && tok.charAt(0) == 'd' && t + 1 < count) {
      String nxt = tokens[t + 1];
      bool numeric = nxt.length() > 0;
      for (unsigned int c = 0; c < nxt.length(); c++)
        if (!isDigit(nxt.charAt(c))) { numeric = false; break; }
      if (numeric) { arg = nxt.toInt(); t++; }
    }

    if (!runCommand(tok, arg))
      Serial.printf("# unknown '%s' - send '?' for help\n", tok.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_OV,  OUTPUT);
  pinMode(PIN_FLOW,  INPUT_PULLUP);
  pinMode(PIN_FLOAT, INPUT);          // G35 is input-only; the PCB supplies the pull-up
  pinMode(PIN_SW,    INPUT);          // G34 likewise - R2 on the PCB pulls it up

  for (int i = 0; i < 3; i++) {
    pinMode(pumps[i].lpwm, OUTPUT); digitalWrite(pumps[i].lpwm, LOW);
    pinMode(pumps[i].ren,  OUTPUT); digitalWrite(pumps[i].ren,  LOW);
    pinMode(pumps[i].len,  OUTPUT); digitalWrite(pumps[i].len,  LOW);
    pumps[i].duty = defaultDuty[i];
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pumps[i].rpwm, PWM_HZ, PWM_BITS);
#else
    ledcSetup(pumps[i].ch, PWM_HZ, PWM_BITS);
    ledcAttachPin(pumps[i].rpwm, pumps[i].ch);
#endif
  }

  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onFlowPulse, RISING);
  lastWinMs = millis();

  // seed the float state so the first reading is not reported as a change
  floatCand = floatHigh = (digitalRead(PIN_FLOAT) == FLOAT_HIGH_LEVEL_STATE);
  swCand    = swOn      = (digitalRead(PIN_SW)    == SWITCH_PRESSED_STATE);
  p1Allowed = !floatHigh;

  stopAll();
  Serial.println("\n# LAM Stage 2 - manual toggle test v3 (PCB REV 1.0)");
  Serial.println("# PUMPS MUST BE PRIMED. P-02 runs on L_PWM2 (missing R_PWM2 trace).");
  Serial.println("# Level protection starts OFF - send 'l' to enable it.");
  status();
}

void loop() {
  uint32_t now = millis();

  bool anyRunning = pumpShouldRun(0) || pumpShouldRun(1) || pumpShouldRun(2);
  digitalWrite(PIN_LED, anyRunning ? HIGH : ((now / 1000) % 2));

  updateSwitch(now);
  updateFloat(now);
  updateFlow(now);

  if (MAX_RUN_S > 0) {
    for (int i = 0; i < 3; i++) {
      if (pumps[i].running && now - pumps[i].startedAt > MAX_RUN_S * 1000UL) {
        Serial.printf("# auto-stop: %s ran %lu s\n", pumps[i].name, (unsigned long)MAX_RUN_S);
        pumps[i].running = false;
        applyPump(i);
      }
    }
  }

  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  runCommandLine(line);
}