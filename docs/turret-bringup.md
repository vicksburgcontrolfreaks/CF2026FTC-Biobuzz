# Turret Bring-Up & Tuning

How to test the turret subsystem the first time it goes on the robot.

**Status:** code written and compiling. The turret is built, and Steps 4-5 are done
by hand-turning it, but it has **never run under power** as of 2026-10-15. Nothing
past Step 5 is confirmed — `MIN_POWER`, `kP` and `TOLERANCE_DEG` are still guesses.

Related files:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretTest.java`

---

## The hardware, and why the code looks like this

| Part | Detail |
|---|---|
| Servo | Dual-mode servo run in **continuous rotation (CR) mode** |
| Gearing | 20T pinion → 69T turret gear = **3.45:1 reduction** |
| Encoder | REV Through Bore V2 (REV-11-3174) on **its own gear** meshing the ring — *not* the servo pinion. 4-pin cable into the **`rf` drive motor's spare encoder port** |
| Home sensor | REV magnetic limit switch, digital port |
| Travel | **234.5°** total (from CAD, 2026-10-15) |

### Sharing the `rf` motor port

Every motor port has a motor connector *and* an encoder connector. The right front
drive motor doesn't use its encoder (odometry comes from the goBILDA Pinpoint over
I2C), so that connector was free and the turret encoder went there. No extra port
burned — nice.

The cost is coupling, so `Turret` is careful about it. It **only reads** `rf` —
never power, direction, run mode, or encoder reset. Keep two things true:

1. **`rf` stays in `RUN_WITHOUT_ENCODER`** (the default). Put it in
   `RUN_USING_ENCODER` and the hub would try to regulate the drive wheel's speed
   from the *turret's* position. Nothing does this today, and Pedro Pathing's
   `Mecanum` class doesn't either — verified in the 2.0.3 jar.
2. **Nobody calls `STOP_AND_RESET_ENCODER` on `rf`**, which would zero the turret
   mid-match.

One trap already handled in code: the SDK negates `getCurrentPosition()` for a
`REVERSE` motor (`DcMotorImpl.adjustPosition`), and this repo sets `rf`'s direction
*inconsistently* — Pedro's `Constants` says `REVERSE`, `RobotHardware` says
`FORWARD`. Left alone, turret angles would invert between TeleOp and auto.
`Turret.readRawTicks()` cancels the motor's direction out, so `ENCODER_DIRECTION`
is the only thing that controls turret sign. Don't "fix" turret sign by editing
`rf`'s direction — that belongs to the drivetrain.

**Why CR mode and not a normal servo:** 234.5° of turret travel × 3.45 = **809°** the
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

Add **two** new devices. The names must match exactly:

| Config name | Device type | Port |
|---|---|---|
| `turret` | Continuous Rotation Servo | servo port |
| `turretHome` | REV Touch Sensor | digital port |

**There is no separate encoder device.** The turret encoder is wired into the spare
encoder connector on the **right front drive motor's** port, so the code reads it off
the existing `rf` motor. Nothing new to configure for it.

Two gotchas:

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

## Step 4 — Range and `TICKS_PER_DEGREE` — **DONE 2026-10-13**

Measured with the **Turret Range Test** op mode (no power; push the turret by hand
to one hard stop, then slowly to the other, passing the magnet on the way).

| Landmark | Encoder ticks | Angle |
|---|---|---|
| Hard stop, clockwise end | **-9617** | -117.25° |
| Magnet switch (center of band) | **-124** | **-18.1°** |
| Hard stop, counter-clockwise end | **+12832** | +117.25° |

Travel is **234.5°**, measured in CAD on 2026-10-15. The first draft of this doc
guessed "about 210°," which is where the old `TICKS_PER_DEGREE = 106.90` came from.
That guess was low by 24.5°, which made every reported angle read about 12% small.

```
span             = 12832 - (-9617) = 22449 ticks
TICKS_PER_DEGREE = 22449 / 234.5   = 95.73
```

Ticks **increase counter-clockwise**, which matches `ENCODER_DIRECTION = 1.0`
(positive angle = counter-clockwise, viewed from above).

### Why the range sweep beats the old protractor method

Pushing to both stops captures the whole 234.5° in one reading, so any slop in where
you stopped is spread over the full range instead of 90° — roughly twice as accurate,
and no protractor needed. The stops are a reference the robot already has.

### Resolved: do NOT recompute this from the gear teeth

The tooth counts and the encoder spec do **not** predict the measured span, and that
is expected. All three of these are confirmed independently:

- **Encoder: REV Through Bore V2 (REV-11-3174), 8192 counts/rev.** Verified by taping
  the encoder gear and sweeping stop to stop — about **2.6 turns** counted by hand,
  against the **2.74** that 22449 / 8192 implies. Match.
- **Gearing: 69T / 20T = 3.45:1**, from tooth count.
- **Travel: 234.5°**, from CAD.

Together those predict a span of `8192 × 3.45 × (234.5 / 360) = 18,410` ticks, but
the encoder counted **22,449** — 22% more. The CAD explains it: **the encoder is not
on the servo's pinion at all.** It has its own gear meshing the ring independently.

![Turret gear train](images/turret-gear-train.png)

Two small gears mesh the 69T ring: the **20T servo pinion** (top left, under the
servo block) and the **encoder's own gear** (bottom left, beside the REV-11-3174).
So 69T / 20T = 3.45:1 describes the *drive* path only — the encoder's path is a
separate ratio, which the measured span puts at **4.21:1** (an encoder gear of
roughly 16T). Ø142.00 confirms the ring itself: a module-2, 69T gear has an outside
diameter of (69 + 2) × 2 = 142 mm.

Reading the encoder off the ring rather than off the servo pinion is the better
arrangement anyway — backlash in the servo's gear train does not corrupt the
position reading.

**Why it changes nothing:** `TICKS_PER_DEGREE` is measured ticks over measured
degrees, so it is right whatever the gear train turns out to be — the tooth counts
were only ever a cross-check on it. `Turret.gearRatio()` reports the tooth-count
ratio to telemetry only; nothing in the control path reads it.

---

## Step 5 — The magnet — **DONE 2026-10-13**

The magnet reads `PRESSED` over about a **5.5° band** (roughly 530 ticks), centered at **-18.1°**.

**It is near the MIDDLE of travel** — 99.2° from the clockwise stop, 135.3° from
the counter-clockwise one. That is why homing searches both directions (Step 6).

The 5.5° band costs less accuracy than it looks like: homing always approaches from
the same side, so it always trips on the same edge of the band. Width only hurts if
the approach direction or speed varies, and `HOMING_POWER` is fixed.

Polarity was confirmed with the **Magnetic Sensor Test** op mode: `open` with the
magnet away, `PRESSED` with it near.

---

## Step 6 — Homing

Finger on stop. Press **`X`**.

Because the magnet sits mid-travel, the turret may start on either side of it, so
homing **searches both ways**:

1. Creeps in `HOMING_DIRECTION` (negative by default) at `HOMING_POWER`.
2. If it reaches a hard stop or runs out of time first, it **reverses once** and
   searches the other way. `Note` reads `hit a stop, reversing search`.
3. Switch trips → power cuts instantly, state goes `HOMING` → `READY`.
4. Turret drives itself to `PARK_ANGLE` (0°, straight ahead) and holds.

Worst case is about 135° of travel to find the magnet, so expect a few seconds.

If it goes `FAILED`, read the `Note` line:

| Note | Means | Fix |
|---|---|---|
| `HOMING FAILED - hit stops both ways...` | Swept the whole range, never saw the magnet | Sensor not wired, not configured, or wrong name |
| `HOMING FAILED - timed out both ways...` | Never reached a stop or the magnet | Turret too slow at this power — raise `HOMING_TIMEOUT_SEC` |

The stall detector is the gear-saver. If homing reverses immediately in *both*
directions, do **not** raise `HOMING_POWER` to push through — either the turret
isn't moving at all, or `STALL_TICKS` is too tight for how slowly it creeps.

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

Expected: power cuts at ±114° — the soft limits, 3° inside the ±117.25° stops — and `Note` reads `at MAX limit` / `at MIN limit`, and
jogging **back** still works. If the turret sails past a limit, `TICKS_PER_DEGREE` is
wrong — go back to Step 4.

---

## Step 9 — Write the numbers down

Panels edits are **not saved**. They vanish when the op mode stops.

Record what you measured. Anything still blank is a starting guess, not a result:

| Constant | Original guess | Measured | In code? |
|---|---|---|---|
| `TICKS_PER_DEGREE` | 22.76 | **95.73** | ✅ |
| `HOME_SWITCH_ANGLE` | -105.0 | **-18.09** | ✅ |
| `MIN_ANGLE` / `MAX_ANGLE` | -105 / 105 | stops at ±117.25, limits set **±114** | ✅ |
| `HOMING_DIRECTION` | -1.0 | -1.0 (searches both ways now) | ✅ |
| `ENCODER_DIRECTION` | 1.0 | 1.0 — ticks rise counter-clockwise | ✅ |
| `MIN_POWER` | 0.12 | *not yet measured* | — |
| `kP` | 0.010 | *not yet tuned* | — |
| `TOLERANCE_DEG` | 2.0 | *not yet tuned* | — |
| `HOMING_TIMEOUT_SEC` | 6.0 | *check against real travel time* | — |

The measured values are already committed as the defaults in `Turret.java`, so they
survive a redeploy. The remaining three come out of Steps 3 and 7, which need the
turret actually under power.

**Only once those are filled in** does this count as tested, and only then is it
worth opening a PR or wiring the turret into a real TeleOp.

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
