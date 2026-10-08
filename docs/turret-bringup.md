# Turret Bring-Up & Tuning

How to test the turret subsystem the first time it goes on the robot.

**Status:** code written and compiling, **never run on hardware** (turret not built
yet as of 2026-10-08). Nothing in here is confirmed. Every number in `Turret.java`
is a starting guess until you measure it.

Related files:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretTest.java`

---

## The hardware, and why the code looks like this

| Part | Detail |
|---|---|
| Servo | Dual-mode servo run in **continuous rotation (CR) mode** |
| Gearing | 30T drive gear → 69T turret gear = **2.3:1 reduction** |
| Encoder | REV Through Bore, into a **motor port** (4-pin cable), relative/quadrature |
| Home sensor | REV magnetic limit switch, digital port |
| Travel | **210°** total |

**Why CR mode and not a normal servo:** 210° of turret travel × 2.3 = **483°** the
servo shaft has to turn. A standard servo maxes out near 270–300°, so it physically
cannot reach. CR mode has no travel limit.

**What that costs us:** a CR servo takes a *power*, not a position, and has no idea
where it is pointed. So the encoder is not a nice-to-have — it is the only thing
stopping the turret from winding itself into a hard stop. That is why `Turret` has
a state machine and a homing routine instead of a plain `setAngle()`.

### The four states

| State | Meaning |
|---|---|
| `NOT_HOMED` | Angle unknown. Soft limits **OFF**. Jog only. This is how it boots. |
| `HOMING` | Creeping toward the magnet. Aborts on timeout or stall. |
| `READY` | Angle is real. Soft limits ON. `setTargetAngle()` works. |
| `FAILED` | Homing failed. Power cut. Re-press X to retry. |

---

## Step 0 — Driver Station configuration

Add three devices. The names must match exactly:

| Config name | Device type | Port |
|---|---|---|
| `turret` | Continuous Rotation Servo | servo port |
| `turretEncoder` | any motor type | the motor port the Through Bore cable plugs into |
| `turretHome` | REV Touch Sensor | digital port |

Two gotchas:

- **`turretEncoder` is a fake motor.** It exists only so the SDK exposes
  `getCurrentPosition()` on that port. **Never call `setPower` on it.** There is no
  motor there, but the SDK does not know that.
- **The touch sensor shows up on the odd pin.** A REV touch/magnetic switch in
  digital port pair 0-1 configures as port **1**, not 0. Same as the existing
  `limitSwitch` device.

---

## Step 1 — Safety setup

- Robot **on blocks**, turret free to rotate through its whole range.
- Hands clear of the gears.
- Finger on the Driver Station stop button for the first homing run.
- Open **Panels**: connect to the robot's wifi, go to **panels.bylazar.com**.
  Every `Turret` constant is tunable there *while the op mode runs* — no redeploy.
  (Panels, not FTC Dashboard. Dashboard broke Driver Station pairing for this team
  and was reverted in `1692cbd` — do not add it back here.)

Run the **Turret Test** op mode (TeleOp, group "Test").

### Controls

| Input | Does |
|---|---|
| Right / left trigger | Jog positive / negative power (raw, analog) |
| `Y` | Toggle JOG ↔ HOLD (closed loop) |
| `X` | Start homing |
| `B` | "The turret is at 0° right now" — zeroes the encoder reference |
| D-pad left / right | HOLD mode: nudge target by `NUDGE_DEG` |
| `A` | HOLD mode: send target to 0° |

---

## Step 2 — Encoder direction

Tap the **right trigger**. Watch `Angle (deg)` on telemetry.

- Goes **up** → correct, move on.
- Goes **down** → set `ENCODER_DIRECTION = -1` in Panels.

Do not rewire anything to fix this. It is a sign flip.

---

## Step 3 — Find the deadband

Squeeze the right trigger **slowly**. Watch `Jog request` and the turret itself.

Note the `Jog request` value where the turret *actually starts turning*. Below that
the servo only hums, draws current, and heats up.

Set `MIN_POWER` to that value **plus about 0.02** of margin.

This matters: `MIN_POWER` is what the closed loop bumps small corrections up to. Set
it too low and the turret stops short of its target and buzzes forever.

---

## Step 4 — Measure `TICKS_PER_DEGREE`

The most important number in the file. Do not skip it.

1. Line the turret up straight ahead by eye.
2. Press **`B`** (this calls the current spot 0° and zeroes the reference).
3. Jog the turret a **measured 90°** — protractor, or tape marks on the ring.
4. Read **`Ticks since ref`** off telemetry.
5. `TICKS_PER_DEGREE = that number ÷ 90`

Sanity check against theory (Through Bore = 8192 ticks per rev of its own shaft):

| Encoder mounted on | Expected ticks/degree |
|---|---|
| The **turret** axis | 8192 / 360 = **22.76** |
| The **servo** shaft | 8192 × 2.3 / 360 = **52.33** |

If your measurement is near one of those, good. If it is near neither, stop and
figure out why before homing — every angle in the code depends on this.

Repeat the measurement 2–3 times. Backlash in the 30T/69T mesh will give you slightly
different answers; average them.

---

## Step 5 — Find the magnet

**Jog slowly** toward the end of travel where the magnet is. Watch `Home switch`
flip from `open` to `PRESSED`.

Confirm the assumption baked into the code:

```
HOME_SWITCH_ANGLE = -105.0   // magnet is at the negative end of travel
HOMING_DIRECTION  = -1.0     // so drive negative to find it
```

- Magnet at the **other** end → flip the sign on **both**.
- Magnet in the **middle** of travel → **stop.** Homing needs different logic,
  because it can creep the wrong way into a hard stop. Ask before running Step 6.

Also note: does the switch stay pressed over a range, or only at one spot? A wide
magnet means the homed zero drifts depending on approach speed.

---

## Step 6 — Homing

Finger on stop. Press **`X`**.

Expected:

1. Slow creep toward the magnet at `HOMING_POWER` (0.25).
2. Switch trips → power cuts instantly.
3. State goes `HOMING` → `READY`.
4. Turret drives itself to `PARK_ANGLE` (0°) and holds.

If it goes `FAILED`, read the `Note` line:

| Note | Means | Fix |
|---|---|---|
| `HOMING TIMED OUT - never saw the magnet` | 6 s elapsed, no switch | Wrong `HOMING_DIRECTION`, or switch not wired/configured |
| `HOMING STALLED - hit a hard stop...` | Encoder stopped moving while powered | Driving away from the magnet — flip `HOMING_DIRECTION` |

The stall detector is the gear-saver. If it trips, **do not raise `HOMING_POWER` to
push through it.** It is telling you the truth.

---

## Step 7 — Tune the closed loop

Press **`Y`** for HOLD mode, then D-pad left/right to nudge the target ±10°.

| What you see | Meaning | Fix |
|---|---|---|
| Overshoots, swings back, oscillates | `kP` too high | Lower `kP` (start 0.010, try 0.006) |
| Crawls, takes forever | `kP` too low | Raise `kP` |
| Stops short and **buzzes** | Can't overcome friction | Raise `MIN_POWER` |
| Jitters on target constantly | Tolerance too tight | Raise `TOLERANCE_DEG` |

**Do not add I or D terms.** A CR servo's response is sloppy and laggy; an I-term
winds up against that and gives you a turret that hunts back and forth forever. Get
`kP` + `TOLERANCE_DEG` + `MIN_POWER` working first. If P alone genuinely is not good
enough, that is a conversation, not a quick constant tweak.

---

## Step 8 — Soft limits

Once homed, in JOG mode, hold the trigger toward each end.

Expected: power cuts at ±105° and `Note` reads `at MAX limit` / `at MIN limit`, and
jogging **back** still works. If the turret sails past a limit, `TICKS_PER_DEGREE` is
wrong — go back to Step 4.

---

## Step 9 — Write the numbers down

Panels edits are **not saved**. They vanish when the op mode stops.

Record what you measured:

| Constant | Guess in code | Measured |
|---|---|---|
| `TICKS_PER_DEGREE` | 22.76 | |
| `ENCODER_DIRECTION` | 1.0 | |
| `MIN_POWER` | 0.12 | |
| `kP` | 0.010 | |
| `TOLERANCE_DEG` | 2.0 | |
| `HOME_SWITCH_ANGLE` | -105.0 | |
| `HOMING_DIRECTION` | -1.0 | |
| `MIN_ANGLE` / `MAX_ANGLE` | -105 / 105 | |

Then edit the defaults in `Turret.java` so they survive a redeploy, rebuild, and
commit. **Only then** does this count as tested, and only then is it worth opening a
PR or wiring the turret into a real TeleOp.

---

## After it works

Not built yet, deliberately — do not start these until Steps 2–9 pass:

- **Integration.** Add a `Turret` field to `RobotHardware` so TeleOp and Auto share
  one instance. It is kept separate for now so a turret config problem cannot break
  the shooter tests.
- **Auto-aim.** `ShootAprilTag.alignRotationOnly()` currently aims by spinning the
  **whole drivetrain**. A working turret should replace that: feed the AprilTag
  bearing into `setTargetAngle()` and let the drivetrain hold still. Much faster, and
  it stops wrecking the robot's odometry heading mid-shot.
- **Homing in init.** Homing after `waitForStart()` burns match time. Moving it into
  the init loop needs a rules check first.
