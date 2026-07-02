# Engineering Journal

This journal + the GitHub repo are scored out of **30 points** using 5 criteria (0/2/4/6 each).
See WRO Future Engineers Rules, Appendix C for full rubric and examples.

Put your actual journal here as a PDF (`engineering-journal.pdf`). Use this file as a planning
outline — for each section, aim for the "Level 6 (Advanced)" bar: justified decisions, testing,
tradeoffs, and evidence of iteration — not just descriptions of what the robot looks like.

## 1. Mobility and Mechanical Design (max 6 pts)
- Chassis design choices — why this layout?
- Steering and drive mechanism — front/rear/4WD, gear ratios tested
- Torque/speed reasoning — what tradeoffs did you measure?
- Mechanical stability/rigidity
- Diagrams (link to `/schemes`)
- Evidence of testing/iteration that changed the design

## 2. Power and Sensor Architecture (max 6 pts)
- Power budget: current draw per subsystem, regulator/battery selection
- Sensor selection and placement — why each sensor, why that position?
- Calibration method
- Wiring diagram (link to `/schemes`)
- Failure points identified + mitigations

## 3. Software Architecture and Obstacle Strategy (max 6 pts)
- Code modularity/structure (link to `/src`)
- State machine or control flow diagram
- Lane-following + obstacle-obedience strategy (algorithm used: PID, CV method, etc.)
- Edge cases handled
- Testing/tuning process + metrics used

## 4. Systems Thinking and Engineering Decisions (max 6 pts)
- How do subsystems (mobility, power, sensors, software, frame) interact?
- Explicit constraints identified (weight, power, processing, time)
- At least one "we chose X instead of Y because..." tradeoff with data/test evidence
- Risk/failure mode identification + mitigation
- Iteration history (version 1 → 2 → 3, what changed and why)

## 5. Reproducibility and GitHub Quality (max 6 pts)
- Is the GitHub repo structured clearly?
- ≥3 meaningful commits with clear messages, on schedule (2mo / 1mo / 2wk before)
- README ≥5000 characters, in English
- CAD, code, wiring info all included
- Could another team rebuild your robot from this documentation alone?

---
### Scoring reminder
| Score | Meaning |
|-------|---------|
| 6 | Advanced engineering — fully justified, tested, tradeoffs shown |
| 4 | Competent engineering — clear, structured, reproducible |
| 2 | Limited evidence — present but incomplete/weak justification |
| 0 | No evidence / missing / irrelevant |

Max total: **30 points** (5 criteria × 6 points)
