# Vehicle Control Code

## Structure
```
/sensors     → sensor-reading modules (camera, distance, IMU, encoders, etc.)
/actuators   → motor/servo control (driving motor(s), steering actuator)
/logic       → decision-making: state machine, lane-following, obstacle response, parking
```

## Hard constraints from the rules (double-check your code respects these)
- **Autonomous only** — no RF/Bluetooth/Wi-Fi/wired remote control while running (rule 11.6, 11.10)
- **Max 2 driving motors**, physically connected to the driving axle (rule 11.13) — no
  independent per-wheel "differential" driving (rule 11.5) — this gets teams **disqualified**
- **Steering must be a genuine steering actuator** (not differential/skid steering) — rule 11.3
- **Single start button** after power-on; no additional interactions allowed at start (rule 9.10–9.11)
- No entering data via physical adjustments, visual/audio signals, or calibration after the
  round has been randomized (rule 9.9, 11.7) — this is grounds for disqualification

## Code documentation requirement
Per rule 7, code must be well-commented since judges may not have access to your specific
dev environment (EV3, Spike, Arduino IDE, etc.). Comment generously — this also feeds
directly into your Engineering Journal's "Software Architecture" score.

## Commit reminder
Push at least 3 commits on schedule:
1. ≥2 months before competition (≥1/5 of final code)
2. ≥1 month before competition
3. ≥2 weeks before competition (this is the one judges score — make sure everything
   important is in by then)
