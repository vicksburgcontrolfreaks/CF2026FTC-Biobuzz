# Gamepad controls — the op modes that shoot

Read off the code on **2026-11-14**, not from memory. Same bindings on **6494 and
5618**: there are no gamepad bindings in `RobotProfile`, so a control change is
one change for both robots.

Covers `DriveTeleOp` (match), `Pollen Shot Tune` and `Shot Map`. The bench and
bring-up tools (Turret Test, Tag Pose Test, Shooter Test, Deflector Zero,
Indexer Load Test, …) are **not** in here — they are single-gamepad tools with
their own one-off layouts, documented in each class header.

> If a binding here disagrees with the robot, the **code** is right and this file
> is stale. The class header of each op mode is the second source; this file is a
> convenience, not the truth.

---

## DriveTeleOp — the match layout

### Gamepad 1 — driver

| Control | Does |
|---|---|
| **Left stick** | drive, field-centric (forward = away from OUR drivers) |
| **Right stick** | turn |
| **X** | "the robot faces away from me right now" — re-references the DRIVING STICKS only, and rumbles. It does **not** move the pose, so it cannot throw the turret's aim off. (Was BACK until 2026-11-06; BACK didn't register reliably on 6494.) |
| **D-pad Down** | **WALL RE-ANCHOR** — "we are square on this wall, collector-first." Fixes a drifted pose from the wall plus the camera's bearing to the hive tags. Drive squarely into the wall, **STOP**, then press. Long rumble = taken, two blips = refused (the screen says why). Gamepad 1 in **both** layouts. |
| everything else | unused on gamepad 1 |

### Gamepad 2 — operator

| Control | Does |
|---|---|
| **B** (hold) | **FIRE.** Spins the flywheel up, waits for it to reach speed **once**, then runs the indexer for as long as you hold it. Release and both stop. No queue, no pauses, no timeouts — see `Launcher.DIRECT_FEED`. |
| **A** | **COLLECTOR ON / OFF.** Press once for on, again for off — a **latch**, not a hold (2026-11-14). It keeps running while you drive, aim and fire. |
| **X** (hold) | collector **out** — and **drops the A latch**. |
| **RT** (hold) | same as B, with the collector helping push balls up. Works whatever the A latch says and leaves it as it found it. |
| **LT** (hold) | collector + indexer **backward**. The only indexer-backward control, and backward **WINS** over fire — it is the jam-clearing control. |
| **Y** | **PRE-SPIN** the flywheel on / off. Not needed to shoot: B and RT spin it up themselves. Use it before a volley so the first shot doesn't wait for spin-up. |
| **Right stick X** | turret by hand — overrides aiming while held |
| **Left stick** | **FLICK** up / down: deflector (shot arc) one step, 1° per flick in TeleOp. Return to centre between flicks. Needs the pre-match zero. |
| **D-pad Up** | park the turret forward / resume aiming |
| **D-pad L / R** | aim at our **AUDIENCE** / **SCORING** cell (with "Our side", until the robot next crosses the middle of the field) |
| **D-pad Down** | nothing — it's the driver's wall re-anchor on gamepad 1 |
| **Bumpers** | unused (2026-11-06: they don't register well on our gamepads) |
| **Rumble** | short = ready (at speed + on target) · 3 blips = nectar detected · long = JAM fault, everything stopped |

### The RT / Y interaction, stated because it surprises people

`flywheelOn = (B or RT held) || Y pre-spin`.

- RT **overrides** Y while held — you never have to press Y to shoot.
- Releasing RT stops the wheels **only if Y is off**. With Y latched, the wheels
  keep spinning after you let go. That is what pre-spin is for.

### SOLO (INIT menu "Gamepads: one" — bench only, hidden at an event)

Everything on **gamepad 1**:

| Control | Does |
|---|---|
| Left stick / Right stick | drive / turn |
| X | heading re-reference |
| B (hold) | FIRE |
| **A** | collector ON / OFF (latch) |
| RT (hold) | collector + indexer forward |
| LT (hold) | collector + indexer **BACKWARD** — this stands in for the missing collector-out |
| Y | pre-spin on / off |
| D-pad Up | park the turret / resume aiming |
| D-pad Down | wall re-anchor (same as the driver layout) |
| D-pad L / R | aim at our AUDIENCE / SCORING cell |

**Not available solo**, because they need a stick or button that is driving:
collector-out (X), turret by hand, deflector arc. The turret aims itself and the
deflector holds what Start gave it; D-pad Up (park) is how you stop a bad aim
with one gamepad. Since X is gone, **A is the only way out of the collector
latch** in solo — which is fine, because A is the toggle.

---

## Pollen Shot Tune

Direct, no gating. Gamepad 1 holds the knobs, gamepad 2 runs the launcher.

| Gamepad 1 — knobs | Does |
|---|---|
| Left stick / Right stick X | drive (field-centric) |
| D-pad Up / Down | pick the knob to tune (`>` marks it) |
| D-pad R / L | step it up / down |
| A (hold) | use the **fine** step instead of the coarse one |
| X | heading re-reference |
| Y | reset the picked knob to its starting value |
| B | clear the shot log |

| Gamepad 2 — launcher | Does |
|---|---|
| B (hold) | flywheel on + indexer forward |
| RT (hold) | flywheel on + indexer + collector forward |
| **BACK** | **LATCH all-forward on** — flywheel, indexer and collector keep running with nothing held, so your hands are free for the knobs. Y, LT or BACK again stops it. (Was "hold A and tap RT" until 2026-11-14.) |
| LT (hold) | indexer + collector backward |
| **A** | **collector ON / OFF (latch)** — same as DriveTeleOp |
| **X** (hold) | collector out — and drops the A latch |
| Y | flywheel on / off; also drops the all-forward latch |
| Right stick X | turret by hand (overrides aiming while held) |
| Left stick | flick up / down: deflector to the next preset (quick jump) |
| D-pad Up | park the turret / resume aiming |
| D-pad L / R | aim at our AUDIENCE / SCORING cell |
| Rumble | short = flywheel at speed |

---

## Shot Map

**Either gamepad**, so one person can run a session solo. `A` here is the data
button, not the collector — left that way deliberately (Shane, 2026-11-14): it
is the whole ladder workflow, and this op mode already latches the collector on
LT + Y.

| Control | Does |
|---|---|
| Left stick / Right stick X | drive (**gamepad 1 only**, field-centric) |
| **A** | record **HIT** |
| **D-pad Down / Up** | record **SHORT** / **LONG** |
| **D-pad Left / Right** | record went **LEFT** / **RIGHT** |
| X | undo the last press |
| B | save this spot, start the next |
| RT (hold) | feed: indexer + collector forward — sends exactly one ball per press |
| Y | flywheel on / off |
| **LT + Y** | **COLLECTOR latch on / off** — runs hands-free so one person can keep loading. RT still sends exactly one ball. |
| RB / LB | RPM up / down by `RPM_STEP` |
| LT + RB / LB | aim trim left / right by `AIM_STEP_DEG` |
| **BACK** (gp1) | "I'm parked on the start spot" — re-sync the pose exactly |
| **LT + BACK** | "I'm parked on a wall, collector-first" — wall re-anchor at **any** wall. Prefer plain BACK when you really are on the start spot: that pose is known, this one is measured. |

**Press what the ball flew, not what the scoreboard said.** Went in but clearly
sailing high → **LONG**. Scraped in at the near lip → **SHORT**. Went in the way
you want every ball to go → **HIT**. A rung scores (highs − lows) / n, so HIT
counts as zero and a lucky make recorded as HIT just softens the answer.

---

## Latches, all of them in one place

A latch is a motor running with nobody touching anything, so every one of these
puts a line on the Driver Station saying so.

| Op mode | Latch | Button | What drops it |
|---|---|---|---|
| DriveTeleOp | collector | **A** | A again, or X (collector out) |
| DriveTeleOp | flywheel pre-spin | **Y** | Y again |
| Pollen Shot Tune | collector | **A** | A again, or X |
| Pollen Shot Tune | everything forward | **BACK** | BACK again, Y, or LT |
| Shot Map | collector | **LT + Y** | LT + Y again, or stopping the op mode |
| Shot Map | flywheel | **Y** | Y again |

---

## Changed on 2026-11-14

- **A latches the collector** in `DriveTeleOp` and `Pollen Shot Tune` (was hold).
- **X drops that latch.** It had to: `Launcher.updateCollector()` takes
  `collectIn` over `collectOut`, so a latched collector would have made
  collector-out a dead button with nothing on screen explaining why.
- **Pollen's solo all-forward latch moved from "hold A + tap RT" to BACK** — A
  cannot be a toggle and a modifier at the same time.
- **RT was already right** and did not change: flywheel + indexer + collector
  while held, all off on release (with the Y caveat above).
- **Shot Map untouched.**
