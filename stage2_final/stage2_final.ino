/* ---------------------------------------------------------------------------
 * LAM Research Challenge 3.0 - Stage 2
 * ESP32 NodeMCU-32S on the LAM Stage 2 PCB (REV 1.0)
 *
 * Case 1  Process fluid transfer with automatic sump level control.
 * Case 2  Flush with all three pumps, then shutdown when the flush tank empties.
 *
 * Push button: short press starts the sequence, 1.5 s hold aborts it.
 * Serial @ 115200 is telemetry only - the system takes no commands.
 * ------------------------------------------------------------------------- */

#include <Arduino.h>

/* ---------------------------------------------------------------- pin map */
const int PIN_P1_RPWM = 33, PIN_P1_LPWM = 32, PIN_P1_REN = 26, PIN_P1_LEN = 13;
const int PIN_P2_RPWM = 19, PIN_P2_LPWM = 18, PIN_P2_REN = 17, PIN_P2_LEN = 16;
const int PIN_P3_RPWM = 25, PIN_P3_LPWM = 23, PIN_P3_REN = 22, PIN_P3_LEN = 21;

const int PIN_OV    = 27;   /* OV-01 solenoid */
const int PIN_FLOW  = 14;   /* FT-01 pulse input */
const int PIN_FLOAT = 35;   /* input-only, PCB pull-up */
const int PIN_SW    = 34;   /* input-only, PCB pull-up */
const int PIN_LED   =  2;

/* Both contacts pull to GND, so closed reads LOW. */
const bool FLOAT_HIGH_LEVEL_STATE = LOW;
const bool SWITCH_PRESSED_STATE   = LOW;
const bool OV_ON_STATE            = HIGH;

/* Driver channels in process-role order. Pump-02's R_PWM trace is absent on
 * this PCB, so it is modulated on L_PWM and its motor leads are swapped. */
const int CH_FLUSH = 0;   /* U2 -> Pump-03 */
const int CH_DRAIN = 1;   /* U3 -> Pump-02 */
const int CH_FILL  = 2;   /* U4 -> Pump-01 */

/* ------------------------------------------------------------ calibration */
/* Q = CAL_A * f + CAL_B, six-point least-squares fit. R2 = 0.9716. */
const float CAL_A =  0.012809f;
const float CAL_B = -0.38382f;
const float K_PULSES_PER_L = 4684.0f;

/* --------------------------------------------------------------- settings */
const uint8_t DUTY_FILL  =  50;
const uint8_t DUTY_DRAIN = 100;
const uint8_t DUTY_FLUSH =  50;

const uint32_t PWM_HZ   = 10000;
const uint8_t  PWM_BITS = 10;

const uint32_t WINDOW_MS       = 500;
const uint32_t HEADER_EVERY_MS = 10000;

const uint32_t SW_DEBOUNCE_MS      = 50;
const uint32_t LONG_PRESS_MS       = 1500;
const uint32_t FLOAT_DEBOUNCE_MS   = 300;
const uint32_t FLOAT_CLEAR_HOLD_MS = 3000;

const uint32_t CASE1_DURATION_S = 120;   /* transfer + level-control demo */
const uint32_t DRAIN_S          = 15;    /* clear the sump back to T-01 */
const uint32_t WAIT_BV_S        = 20;    /* operator opens BV-01 */

/* The flush tank has no level sensor, so "empty" is inferred from flow
 * collapsing - but only after enough volume has passed that the 4.2 L tank
 * could actually be empty, which rules out a false stop on the sump heel. */
const uint32_t FLUSH_GRACE_S       = 12;
const float    FLUSH_END_Q_LMIN    = 0.40f;
const uint32_t FLUSH_END_CONFIRM_S = 4;
const float    FLUSH_END_MIN_VOL_L = 2.5f;
const uint32_t FLUSH_MAX_S         = 150;  /* backstop if flow never collapses */

const uint32_t SEQUENCE_MAX_S = 1200;      /* global watchdog */

/* Staggered starts so three motors do not draw inrush together. */
const uint32_t FLUSH_STAGGER_1_MS = 400;
const uint32_t FLUSH_STAGGER_2_MS = 800;

enum State { ST_IDLE, ST_CASE1, ST_DRAIN, ST_WAIT_BV, ST_FLUSH, ST_DONE };

/* ---------------------------------------------------------------- actuators */
struct Pump {
  const char* name;
  int pwmPin;        /* the input that is modulated */
  int lowPin;        /* the opposite input, held LOW */
  int ren, len, ch;
  bool     running;
  uint8_t  duty;
};

Pump pumps[3] = {
  { "Pump-03", PIN_P1_RPWM, PIN_P1_LPWM, PIN_P1_REN, PIN_P1_LEN, 0, false, DUTY_FLUSH },
  { "Pump-02", PIN_P2_LPWM, PIN_P2_RPWM, PIN_P2_REN, PIN_P2_LEN, 1, false, DUTY_DRAIN },
  { "Pump-01", PIN_P3_RPWM, PIN_P3_LPWM, PIN_P3_REN, PIN_P3_LEN, 2, false, DUTY_FILL  }
};

/* Which pumps the float may cut. Pump-02 is the only sump outlet and is never
 * cut, or the level could never fall again. */
bool protect[3] = { false, false, false };

bool ovOpen      = false;
bool levelAllows = true;

/* ------------------------------------------------------------------ inputs */
bool     floatHigh = false, floatCand = false;
uint32_t floatChangeT = 0, floatClearSince = 0;

bool     swPressed = false, swCand = false;
uint32_t swChangeT = 0, pressStart = 0;
bool     longFired = false;
bool     evShortPress = false, evLongPress = false;

volatile uint32_t gPulses     = 0;
volatile uint32_t gLastEdgeUs = 0;

uint32_t lastWinMs = 0, lastWinPulses = 0, lastHeaderMs = 0;
uint32_t flowStartMs = 0, totBasePulses = 0;
double   volEqnL = 0;   /* litres by integrating Q = a*f + b */
float    lastQ = 0;

State    state = ST_IDLE;
uint32_t stateSince = 0, seqStart = 0, flushLowSince = 0;
bool     flushEmptyConfirmed = false;
double   flushBaseL = 0;
uint8_t  flushStaggerStep = 0;

void IRAM_ATTR onFlowPulse() {
  uint32_t now = micros();
  if (now - gLastEdgeUs < 300) return;   /* ~3.3 kHz ceiling; 6 L/min is ~590 Hz */
  gLastEdgeUs = now;
  gPulses++;
}

double volPulseL() { return (double)(gPulses - totBasePulses) / K_PULSES_PER_L; }

const char* stateName() {
  switch (state) {
    case ST_IDLE:    return "IDLE";
    case ST_CASE1:   return "CASE1";
    case ST_DRAIN:   return "DRAIN";
    case ST_WAIT_BV: return "WAIT_BV";
    case ST_FLUSH:   return "FLUSH";
    default:         return "DONE";
  }
}

/* ------------------------------------------------------------------ output */
bool pumpShouldRun(int i) {
  if (!pumps[i].running) return false;
  if (protect[i] && !levelAllows) return false;
  return true;
}

void applyPump(int i) {
  Pump &p = pumps[i];
  uint8_t  pct  = pumpShouldRun(i) ? p.duty : 0;
  uint32_t maxv = (1u << PWM_BITS) - 1;
  uint32_t duty = (maxv * (uint32_t)pct) / 100;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(p.pwmPin, duty);
#else
  ledcWrite(p.ch, duty);
#endif

  digitalWrite(p.lowPin, LOW);
  digitalWrite(p.ren, pct > 0 ? HIGH : LOW);
  digitalWrite(p.len, pct > 0 ? HIGH : LOW);
}

void applyAll() { for (int i = 0; i < 3; i++) applyPump(i); }

void setPump(int i, bool on) {
  pumps[i].running = on;
  applyPump(i);
  Serial.printf("# %s %s (duty %u %%)\n", pumps[i].name, on ? "START" : "STOP", pumps[i].duty);
}

void setValve(bool open) {
  ovOpen = open;
  digitalWrite(PIN_OV, open ? OV_ON_STATE : !OV_ON_STATE);
  Serial.printf("# OV-01 %s\n", open ? "OPEN" : "CLOSED");
}

void stopAll() {
  for (int i = 0; i < 3; i++) { pumps[i].running = false; applyPump(i); }
  ovOpen = false;
  digitalWrite(PIN_OV, !OV_ON_STATE);
  Serial.println("# ALL STOPPED - pumps off, OV-01 closed");
}

/* ----------------------------------------------------------------- sensing */
void updateSwitch(uint32_t now) {
  bool raw = (digitalRead(PIN_SW) == SWITCH_PRESSED_STATE);
  if (raw != swCand) { swCand = raw; swChangeT = now; }

  if (now - swChangeT >= SW_DEBOUNCE_MS && swPressed != swCand) {
    swPressed = swCand;
    if (swPressed) { pressStart = now; longFired = false; }
    else if (!longFired) evShortPress = true;
  }

  if (swPressed && !longFired && now - pressStart >= LONG_PRESS_MS) {
    longFired   = true;
    evLongPress = true;
  }
}

void updateFloat(uint32_t now) {
  bool raw = (digitalRead(PIN_FLOAT) == FLOAT_HIGH_LEVEL_STATE);
  if (raw != floatCand) { floatCand = raw; floatChangeT = now; }

  if (now - floatChangeT >= FLOAT_DEBOUNCE_MS && floatHigh != floatCand) {
    floatHigh = floatCand;
    Serial.printf("# FLOAT -> %s\n", floatHigh ? "HIGH (sump full)" : "clear");
  }

  bool before = levelAllows;
  if (floatHigh) { levelAllows = false; floatClearSince = 0; }
  else if (!levelAllows) {
    if (floatClearSince == 0) floatClearSince = now;
    if (now - floatClearSince >= FLOAT_CLEAR_HOLD_MS) levelAllows = true;
  }

  if (before != levelAllows) {
    Serial.printf("# LEVEL PROTECTION: protected pumps %s\n",
                  levelAllows ? "released" : "CUT OFF");
    applyAll();
  }
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
  lastQ = Qlmin;
  volEqnL += Qlmin * dt / 60000.0;

  if (now - lastHeaderMs >= HEADER_EVERY_MS) {
    lastHeaderMs = now;
    Serial.println("#   time_s   pulses   freq_Hz   Q_L/min    Q_mL/s   total_L  tot_eqn  fl  state");
  }

  Serial.printf("  %7.1f  %7lu  %8.1f  %8.3f  %8.4f   %d  %s\n",
                (now - flowStartMs) / 1000.0, (unsigned long)p, freqHz,
                Qlmin, volPulseL(), floatHigh ? 1 : 0, stateName());
}

/* ----------------------------------------------------------- state machine */
void enter(State s, uint32_t now) {
  state = s; stateSince = now;
  Serial.printf("\n# ======== STATE -> %s ========\n", stateName());
}

void finishRun(uint32_t now, const char* reason) {
  stopAll();
  Serial.printf("\n# ####### RUN COMPLETE - %s #######\n", reason);
  Serial.printf("# run time     : %.1f s\n", (now - seqStart) / 1000.0);
  Serial.printf("# flush volume : %.4f L\n", volPulseL() - flushBaseL);
  Serial.printf("# total volume : %.4f L\n", volPulseL());
  Serial.println("# press the button once to reset.");
}

void sequenceTick(uint32_t now) {
  bool shortPress = evShortPress; evShortPress = false;
  bool longPress  = evLongPress;  evLongPress  = false;

  if (longPress && state != ST_IDLE) {
    Serial.println("\n# !! BUTTON HELD - ABORTING TO SAFE STATE !!");
    stopAll();
    enter(ST_IDLE, now);
    return;
  }

  if (state != ST_IDLE && state != ST_DONE && now - seqStart > SEQUENCE_MAX_S * 1000UL) {
    Serial.println("\n# !! SEQUENCE WATCHDOG EXPIRED !!");
    stopAll();
    enter(ST_DONE, now);
    return;
  }

  switch (state) {

    case ST_IDLE:
      if (shortPress) {
        seqStart      = now;
        totBasePulses = gPulses;
        volEqnL       = 0;
        lastWinMs     = now;
        lastWinPulses = gPulses;
        flowStartMs   = now;
        lastHeaderMs  = now - HEADER_EVERY_MS;
        flushBaseL          = 0;
        flushLowSince       = 0;
        flushEmptyConfirmed = false;

        Serial.println("\n# ########## CASE 1: PROCESS FLUID TRANSFER ##########");
        protect[CH_FILL]  = true;
        protect[CH_DRAIN] = false;
        protect[CH_FLUSH] = false;
        setValve(true);
        setPump(CH_DRAIN, true);
        setPump(CH_FILL,  true);
        enter(ST_CASE1, now);
      }
      break;

    case ST_CASE1:
      /* Hand over only once the dwell is done and the sump has recovered, so
       * the transition never lands in the middle of a level cut-off. */
      if (now - stateSince >= CASE1_DURATION_S * 1000UL && levelAllows && !floatHigh) {
        Serial.println("\n# ####### DRAIN: isolating the process tank #######");
        setValve(false);
        setPump(CH_FILL, false);   /* its suction is dead-ended with OV-01 shut */
        enter(ST_DRAIN, now);
      }
      break;

    case ST_DRAIN:
      /* Pump-02 pushes the process fluid left in the sump back to T-01 before
       * any flush water arrives, so the two mix as little as possible. */
      if (now - stateSince >= DRAIN_S * 1000UL) {
        setPump(CH_DRAIN, false);
        Serial.println("#");
        Serial.println("#   >>> OPEN THE MANUAL BALL VALVE BV-01 NOW <<<");
        Serial.printf ("#   >>> the flush starts in %lu s\n", (unsigned long)WAIT_BV_S);
        Serial.println("#");
        enter(ST_WAIT_BV, now);
      }
      break;

    case ST_WAIT_BV:
      if (now - stateSince >= WAIT_BV_S * 1000UL) {
        Serial.println("\n# ########## CASE 2: FLUSH ##########");
        protect[CH_FILL]  = true;
        protect[CH_FLUSH] = true;   /* it feeds Pump-01's suction through the tee */
        protect[CH_DRAIN] = false;
        flushBaseL       = volPulseL();
        flushLowSince    = 0;
        flushStaggerStep = 0;
        enter(ST_FLUSH, now);
      }
      break;

    case ST_FLUSH:
      if (flushStaggerStep == 0) {
        setPump(CH_DRAIN, true);
        flushStaggerStep = 1;
      } else if (flushStaggerStep == 1 && now - stateSince >= FLUSH_STAGGER_1_MS) {
        setPump(CH_FILL, true);
        flushStaggerStep = 2;
      } else if (flushStaggerStep == 2 && now - stateSince >= FLUSH_STAGGER_2_MS) {
        setPump(CH_FLUSH, true);
        flushStaggerStep = 3;
        Serial.println("# all three pumps running simultaneously");
      }

      {
        float flushedL = volPulseL() - flushBaseL;
        bool  armed    = (now - stateSince >= FLUSH_GRACE_S * 1000UL) &&
                         (flushedL >= FLUSH_END_MIN_VOL_L);

        if (lastQ < FLUSH_END_Q_LMIN) {
          if (flushLowSince == 0) flushLowSince = now;
          if (armed && now - flushLowSince >= FLUSH_END_CONFIRM_S * 1000UL) {
            flushEmptyConfirmed = true;
            Serial.printf("# FLUSH TANK EMPTY at %.1f s, %.3f L flushed\n",
                          (now - seqStart) / 1000.0, flushedL);
            finishRun(now, "FLUSH WATER EXHAUSTED");
            enter(ST_DONE, now);
          }
        } else {
          flushLowSince = 0;
        }
      }

      if (!flushEmptyConfirmed && now - stateSince >= FLUSH_MAX_S * 1000UL) {
        finishRun(now, "FLUSH TIME LIMIT - tank not confirmed empty");
        enter(ST_DONE, now);
      }
      break;

    case ST_DONE:
      if (shortPress) {
        Serial.println("# reset - ready to run again");
        enter(ST_IDLE, now);
      }
      break;
  }
}

/* ------------------------------------------------------------ setup / loop */
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_LED,   OUTPUT);
  pinMode(PIN_OV,    OUTPUT);
  pinMode(PIN_FLOW,  INPUT_PULLUP);
  pinMode(PIN_FLOAT, INPUT);   /* input-only pins: the PCB supplies the pull-up */
  pinMode(PIN_SW,    INPUT);

  for (int i = 0; i < 3; i++) {
    pinMode(pumps[i].lowPin, OUTPUT); digitalWrite(pumps[i].lowPin, LOW);
    pinMode(pumps[i].ren,    OUTPUT); digitalWrite(pumps[i].ren,    LOW);
    pinMode(pumps[i].len,    OUTPUT); digitalWrite(pumps[i].len,    LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pumps[i].pwmPin, PWM_HZ, PWM_BITS);
#else
    ledcSetup(pumps[i].ch, PWM_HZ, PWM_BITS);
    ledcAttachPin(pumps[i].pwmPin, pumps[i].ch);
#endif
  }

  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onFlowPulse, RISING);

  /* Seed the inputs so the first read is not reported as a change. */
  floatCand = floatHigh = (digitalRead(PIN_FLOAT) == FLOAT_HIGH_LEVEL_STATE);
  swCand    = swPressed = (digitalRead(PIN_SW)    == SWITCH_PRESSED_STATE);
  levelAllows = !floatHigh;
  lastWinMs   = flowStartMs = millis();

  stopAll();

  Serial.println("\n# =====================================================");
  Serial.println("# LAM Research Challenge 3.0 - Stage 2");
  Serial.printf ("# CAL: Q = %.6f*f %+.5f   K = %.0f pulses/L\n",
                 CAL_A, CAL_B, K_PULSES_PER_L);
  Serial.println("# Press the button to start. Hold 1.5 s to abort.");
  Serial.println("# =====================================================");
}

void loop() {
  uint32_t now = millis();

  updateSwitch(now);
  updateFloat(now);
  updateFlow(now);
  sequenceTick(now);

  /* steady = running, fast blink = open BV-01, slow blink = idle or done */
  bool anyRunning = pumpShouldRun(0) || pumpShouldRun(1) || pumpShouldRun(2);
  if      (state == ST_WAIT_BV) digitalWrite(PIN_LED, (now / 120) % 2);
  else if (anyRunning)          digitalWrite(PIN_LED, HIGH);
  else                          digitalWrite(PIN_LED, (now / 1000) % 2);
}
