# [Team Name] — WRO Future Engineers 2026: Self-Driving Cars

> ⚠️ **Note:** This README is a scored deliverable (WRO Rule 7). It must be written in English,
> contain **at least 5000 characters**, and clearly describe your electromechanical components,
> how the code relates to them, and how to build/compile/upload the code to the vehicle's
> controllers. Delete this note and all bracketed placeholders before submission.

## Team
- **Team name:**
- **Country:**
- **Team members:** [Name 1], [Name 2], [Name 3]
- **Coach:**

## Vehicle Overview
[1–2 paragraphs: what is your vehicle, its general architecture (SBC/SBM used, drive type —
front/rear/four-wheel drive with steering actuator per rule 11.3), and how it approaches the
Open Challenge and Obstacle Challenge.]

## Mobility Management
[Describe the chassis, drivetrain, and steering mechanism. Explain WHY you chose this
configuration — torque/speed tradeoffs, gear ratios tested, stability considerations. Remember:
rule 11.3 requires 4-wheel vehicle, one driving axle + one steering actuator — NO differential
wheeled/skid-steer bases allowed. Max 2 driving motors (rule 11.13), physically connected to
the axle (gearbox or direct).]

## Power Management
[Describe your power architecture: battery choice(s), voltage regulation, current draw per
subsystem (a power budget strengthens your score — see Appendix C, Criterion 2). Include a
wiring diagram — see `/schemes`.]

## Sense Management
[Describe every sensor used, why you chose it, and where/how it's mounted. Cameras count as
sensors (rule 11.11) — smartphones can be used as cameras too. Explain your calibration
process and any failure points you identified/mitigated.]

## Obstacle Management (Obstacle Challenge strategy)
[Explain your strategy for detecting and responding to red/green traffic pillars, staying in lane,
and performing parallel parking. Include your control logic (state machine, PID, computer
vision method, etc.) and how you tuned it.]

## Software Architecture
[Explain the code structure/modules and how they map to the electromechanical components
above. Include a flowchart or state machine diagram — see `/schemes`. Reference relevant
files in `/src`.]

## Build / Compile / Upload Instructions
[Step-by-step: what hardware/software is needed, how to flash/compile/upload the code onto
your specific controller (e.g., Raspberry Pi, Arduino, EV3, Spike). Judges may not have access
to your specific dev environment (rule 7), so be explicit.]

## Repository Structure
```
/src                    → vehicle control code (see subfolders)
  /sensors               → sensor-reading modules
  /actuators              → motor/servo control modules
  /logic                  → state machine / decision logic
/models
  /3d-printed             → STL/3MF files for 3D-printed parts
  /laser-cut               → DXF/SVG files for laser-cut/CNC parts
/schemes                 → wiring diagrams, flowcharts, CAD screenshots
/photos
  /vehicle                 → photos from every side, top, and bottom (rule 7)
  /team                    → team photo (rule 7)
/video                   → links to YouTube videos (Open + Obstacle Challenge)
/engineering-journal      → Engineering Journal PDF (see Appendix C scoring rubric)
```

## Videos
- Open Challenge: [YouTube link — must show ≥30s of autonomous driving]
- Obstacle Challenge: [YouTube link — must show ≥30s of autonomous driving]

## AI Usage Disclosure
[If any AI tool was used (code assistance, journal writing, image generation, etc.), state which
tool, for what purpose, and to what extent.]

## Commit History Reminder (Rule 7)
This repo must contain at least 3 commits:
- [ ] Commit 1 — no later than **2 months before competition**, containing ≥1/5 of final code
- [ ] Commit 2 — no later than **1 month before competition**
- [ ] Commit 3 — no later than **2 weeks before competition** (this is the one judges mainly score)

Repository must be **public** from submission and stay public for **at least 12 months** after
the competition.
