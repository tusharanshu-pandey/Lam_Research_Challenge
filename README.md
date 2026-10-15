# Lam Research Challenge 2026 — Stage 2

**Hardware Hustle**

| | |
|---|---|
| **Team ID** | LRC-26-0001 |
| **Team name** | Bright Minds |

---

## Videos Links

They are shared by link:

- **Demonstration video:** https://drive.google.com/file/d/1t7MTM1Gt9xGObPCNsvqR0XhmHdTB4jCt/view?usp=drive_link
- **Journey video:** https://drive.google.com/file/d/1RYA7X7ApofKEfZm2DOThlvdmiFzrXw2A/view?usp=drive_link

---


## Repository contents

| Path | What it is |
|---|---|
| `LAM_Research_Stage_2_with_Annexure.pdf` | Completed answer paper + annexure |
| `stage2_final/` | **The submitted controller firmware** (Annexure A-6) |
| `05_manual_test_v3/` | Bench sketch used for data collection |
| `Serial_output_3.txt` | Serial log of the valve-position and duty sweeps used for Q2.1 |
| `figures/` | Graphs plotted from the logs and the calibration trials |
| `annexure_assets/` | Serial Monitor captures reproduced in the annexure |
| `1790306239938-STAGE_2_Question_Paper_Lam_Research_Challenge_3.0 (1).pdf` | The question paper |

### Figures

| File | Figure |
|---|---|
| `fig1_q21_valve_vs_flow.png` | A-1 — Q2.1 flow vs ball-valve position |
| `fig2_q22_calibration.png` | A-2 — Q2.2 calibration equation |
| `fig3_q22_errors.png` | A-3 — Q2.2 calibration error by trial |
| `fig4_q12_velocity.png`, `fig5_q12_reynolds.png` | Q1.2 velocity and Reynolds number |
| `fig6_flow_vs_time.png`, `fig7_duty_vs_flow.png` | Supporting run data |

---

## The submitted firmware

### [`stage2_final/stage2_final.ino`](stage2_final/stage2_final.ino)

**This is the controller firmware that was submitted.** It is reproduced in full as
**Annexure A-6** of the submission PDF, and the file in this repository is identical to
that listing, line for line.

It is a single autonomous build. One short press of the panel button runs the whole
sequence to completion; a 1.5 s hold aborts to a safe state. **Serial carries telemetry
only and accepts no commands**, so the sequence provably cannot be driven from the
laptop — which is what the brief's "without manual intervention" requires.

```
IDLE --press--> CASE1 --t--> DRAIN --t--> WAIT_BV --t--> FLUSH --empty--> DONE
  ^                                                                        |
  +--------------- 1.5 s hold (abort), or press from DONE ------------------+
```

| State | Actuators | Purpose |
|---|---|---|
| `CASE1` | OV-01 open, Pump-01 50 %, Pump-02 100 % | Process-fluid transfer; the float switch cuts Pump-01 on a sudden charge |
| `DRAIN` | OV-01 shut, Pump-02 only | Pushes the fluid left in the sump back to T-01 before flush water arrives |
| `WAIT_BV` | all off | Operator opens the manual ball valve BV-01 |
| `FLUSH` | OV-01 shut, all three pumps, staggered starts | Flush water through the lines |
| `DONE` | all off | Totals printed |

Pump-02 is never cut by the float — it is the only sump outlet, so cutting it would mean
the level could never fall again. Pump-03 is cut alongside Pump-01 because it feeds
Pump-01's suction through the equal tee.

T-02 has no level sensor, so "flush tank empty" is inferred from flow collapsing — but
only once enough volume has actually passed FT-01 that the 4.2 L tank could be empty.
That volume gate is what stops the sump heel from triggering a false shutdown.

### Building

Arduino IDE 2.x or `arduino-cli`, with the **esp32** core installed.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 stage2_final
arduino-cli upload  --fqbn esp32:esp32:esp32 -p COM21 stage2_final
```

Serial Monitor at **115200**. Set the **upload speed to 115200** as well — 921600
corrupts on this board. The code builds against both ESP32 core 2.x and 3.x (the
`ledc` API changed between them and both paths are compiled in).

### [`05_manual_test_v3/05_manual_test_v3.ino`](05_manual_test_v3/05_manual_test_v3.ino)

Bench / manual-test sketch, kept for reference. It exposes a serial command shell used
to drive individual pumps and the valve by hand while collecting the Q2.1–Q2.3 data and
the calibration trials. **It is not the submitted controller** and plays no part in the
autonomous demonstration.

---

## Flow transmitter calibration

FT-01 is a YF-S401 Hall-effect turbine transmitter. Its pulse frequency was calibrated
against six timed jug-and-stopwatch catches:

```
Q = 0.012809 · f − 0.38382        Q in L/min, f in Hz
```

| | |
|---|---|
| R² | 0.9716 |
| K-factor from the slope | 4684 pulses / litre |
| Maximum calibration error | 6.86 % (0.151 L/min, trial 4) |
| Average calibration error | 4.12 % (0.096 L/min) |

The datasheet claims 5880 pulses/litre. The measured figure is about 20 % away from it,
which is exactly why the brief asks for the transmitter to be calibrated rather than
trusted.

---

## Hardware notes

Two things about this particular PCB are worth recording, because the firmware works
around both:

- **Pump-02's `R_PWM` trace is absent on this board.** The channel is therefore
  modulated on `L_PWM` (GPIO18) with GPIO19 idling low, which reverses the motor — so
  that driver's M+ / M− leads are swapped at the screw terminals. Its unused `RPWM`
  terminal is pulled to GND through 10 k so the floating input cannot chatter.
- **GPIO34 and GPIO35 are input-only and have no internal pull-up.** The float switch
  and the ON/OFF button rely on the PCB's own 10 k pull-ups, so they are configured as
  `INPUT`, never `INPUT_PULLUP`. Both contacts pull to GND, so a closed contact reads
  LOW.

The ON/OFF control is a momentary tactile button, not a latching toggle, so run/stop is
latched in software and driven by press events. Abort is deliberately the *long* press:
a knock against the button part-way through a recorded run must not be able to kill it.
