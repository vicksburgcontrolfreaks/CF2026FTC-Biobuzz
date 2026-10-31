# Open work — 2026 BIOBUZZ

Started 2026-10-27. Things that need the robot, or need a human decision, and
would otherwise get lost in commit messages.

---

## Needs the robot

### 1. Pull the robot config XML into the repo
**Blocked on:** building the config once on the Driver Station.

The Driver Station stores each configuration as XML on the Robot Controller
under `/sdcard/FIRST/settings/`. Build it by hand once, then it never has to be
thumb-typed again. Full steps in [hardware-config.md](hardware-config.md).

```
adb connect 192.168.43.1:5555
adb shell ls /sdcard/FIRST/settings/
adb pull /sdcard/FIRST/settings/<Name>.xml docs/robot-config/
```

Commit the result. After that, any hub is one `adb push` from a working config,
and wiring changes become reviewable diffs.

Do **not** use `TeamCode/src/main/res/xml/hardware2025.xml` as a template — it
is not a loadable FTC config and never was.

While entering it, two names still need correcting from the hand-written map:
- `Pivot Sensor` → **`pivotHome`**
- `Color Sensor` → **`colorSensor`**

### 2. Re-zero and re-verify the turret
**Why:** the turret encoder cable moved from `rf` to `lf` on 2026-10-27, so
reads stop crossing RS485 to the Expansion Hub.

The angle math did not change — `readRawTicks()` cancels the motor direction
either way — but the plug physically moved, so the 2026-10-22 on-robot
confirmation of auto-aim **does not carry over**.

- Run **Magnetic Sensor Test** — switch fires, zone width looks sane
- Run **Turret Range Test** — homing finds zero, travel matches the 286° in
  [turret-bringup.md](turret-bringup.md)
- Run **Turret Aim Test** — confirm `ENCODER_DIRECTION` is still `1.0`; if the
  reported angle goes DOWN on positive power, set it to `-1.0`. That is the only
  place turret sign should ever be adjusted
- While there, capture the two numbers never collected: `Frame age` in ms, and
  how far `Bearing raw` jitters. Those would let `DEADBAND_DEG` come down from 5.0

### ~~3. Measure the pollen / nectar pivot offsets~~ — DROPPED 2026-10-31
The pivot does not need homing at all, so there are no offsets to measure. A
servo's `setPosition` is absolute, so the positions measured on 2026-10-30 are
already everything the pivot needs. Full reasoning is in `ShooterPivot`'s class
comment; the short version is that the 2026-10-01 → 2026-10-30 shift was
**+0.059 pollen but +0.075 nectar**, and a homing sensor finds one reference
point, which can only correct a constant offset. Re-homing would have left
~0.016 of error on one grip — a third of nectar's entire 0.044 of clearance.

**The replacement procedure, after any work on the pivot servo, horn or
linkage:** run **Pivot Jog Test**, re-measure both grips against real balls,
paste the numbers into `ShooterPivot.POLLEN_POSITION` / `NECTAR_POSITION`. The
op mode prints them in copy-paste form. That is this mechanism's calibration.

Homing was kept, not deleted: the sensor is now optional (`new
ShooterPivot(servo)`), `POLLEN_OFFSET`/`NECTAR_OFFSET` are `NaN` so a future
powered sensor cannot silently send both grips to the same gap, and
`isUsingHomedOffsets()` reports which numbers are live. Fit a beam break and
measure the offsets and the path wakes up with no other changes. Worth
revisiting only if the linkage starts slipping often enough that re-measuring
by hand becomes the bottleneck.

---

## Confirmed on the robot 2026-10-30

### Shooter pivot travel, measured with no sensor

Measured with **Pivot Jog Test** against real balls. The `pivotHome` beam break
has no power, so homing cannot run -- these are absolute servo positions, which
is all a standard servo ever needed. **Lower is a wider gap.**

| | position |
|---|---|
| widest (`MIN_SAFE_POSITION`) | 0.166 |
| open, as jogged | 0.2 |
| nectar grip | 0.21 |
| pollen grip | 0.539 |
| tightest (`MAX_SAFE_POSITION`) | 0.679 |

`OPEN_POSITION` was lowered from the jogged 0.2 to **0.17** on 2026-10-31, when
open started being used for ball ENTRY and not just extraction — 0.2 left only
0.01 of clearance over the nectar grip. It is the one chosen rather than
measured number in `ShooterPivot`.

Everything that commands the pivot is now clipped to the band, which is what
makes driving it into a hard stop impossible. It was clipped to the SDK range
of 0.0-1.0 before, and the pivot got slammed into a stop twice on 2026-10-30.

The 2026-10-01 bench values were pollen 0.48 and nectar 0.135. Both moved up by
roughly a constant 0.06-0.08, which looks like a re-seated linkage or servo horn
rather than anything changing about the balls.

**Worth a look: nectar grip 0.21 sits only 0.044 from the widest position, while
there is 0.14 of unused travel above the pollen grip.** So the pivot can barely
open wider than its own nectar grip -- there is almost no extraction clearance
for a stuck nectar ball, and no margin at all for an oversized one. Re-seating
the linkage lower in its range would buy that margin back from travel that is
currently doing nothing.

---

### Ball type IS readable from indexer current

Twenty labeled passes with **Indexer Load Test**, at 50 RPM, normalized to 12V:

| | n | min | mean | max |
|---|---|---|---|---|
| empty indexer (baseline peak) | 3 | 0.71 | **0.79** | 0.86 A |
| pollen, rise above baseline | 10 | -0.24 | **-0.06** | 0.06 A |
| nectar, rise above baseline | 10 | 1.38 | **2.09** | 2.63 A |

**Gap between the groups: 1.32 A.** Wider than either group's own spread (0.30
for pollen, 1.25 for nectar), which is the test that matters -- the separation
is bigger than the noise it has to survive. Threshold set at the midpoint,
**0.72 A above baseline**, so either misclassification needs a 0.66 A excursion.

Lives in `subsystems/BallLoadClassifier.java`, with `LoadSortTest` driving the
pivot from it. What invalidates the calibration is documented there: feed speed
(these are 50 RPM numbers), how the pass starts, and mechanical changes.

**Pollen is indistinguishable from an empty indexer** -- -0.06 A mean rise is
zero within noise, because a pollen ball costs the indexer essentially nothing.
Good for reliability, since the real question is only "is there extra load",
but it means current sensing can never be a ball-PRESENCE sensor. The result
enum is called `POLLEN_OR_EMPTY` so nobody forgets. With `sensor1-3` retired on
2026-10-27 there is no ball-presence signal on the robot at all.

Still to spot-check: the 12V normalization never had to work (all 20 samples
came from a similar battery state), and warm-vs-cold mechanism and scuffed
balls were not varied.

---

## Confirmed on the robot 2026-10-29

- Collector and indexer directions are both correct as set in `RobotHardware`
  (`collector` REVERSE, `indexer` FORWARD). The indexer direction was a guess
  until now
- Collector RPM reads sane, so the 20:1 ratio constant is right
- Still open: a MECHANICAL problem in the collection path, not a code problem
- Not captured yet: the `Indexer Amps` jam number any future unjam logic needs

---

## Needs a decision, not the robot

- **Every shooter RPM number predates the gear ratio fix and is now 3x off.**
  Resolved 2026-10-29: the ratio is 3:1, the HD Hex's internal gearbox, now in
  `RobotHardware.SHOOTER_GEAR_RATIO`. `ShootAprilTag` and `TestTeleOp` both said
  1:1 before, so every RPM either of them commanded was really a THIRD of what it
  claimed. Anything tuned by feel against the old numbers has to be re-tuned.
- **Two shooter targets are now physically unreachable.** An HD Hex free-speeds
  near 6000 RPM at the motor, so through 3:1 the flywheel tops out near 2000 RPM
  with no load at all. But `ShootAprilTag.SHOOTER_VELOCITY_RPM` defaults to 2000
  (exactly free speed) and `TestTeleOp` lets you dial 3000 (1.5x past it). Both
  saturate the motor, and a saturated velocity PID looks exactly like a broken
  one. Measure the real no-load top speed, then cap both around 85% of it.
  `ShooterVelocityTest` still hardcodes its own 3.0 and 3000 cap -- fold it into
  `RobotHardware` when that branch merges.
- **`ShootAprilTag.shoot()` is a first draft.** The old flipper sequence was
  translated onto the indexer motor, but nobody has confirmed how the real
  shooter loads. Does the indexer pulse per ball or run continuously? Should the
  pivot be at pollen/nectar spacing before firing?
- **Jade needs to rename her servo.** `jade/color-sensing-subsystem` and
  `jade/shooter-indexer-tuning` still use `indexer` as a `Servo`. That name is a
  motor now, so those branches break when this merges. Her call, not ours
- **Wire `ColorSensing` into `CollectionSystem`.** Needs
  `jade/color-sensing-subsystem` merged first — `CollectionSystem.getColorSensor()`
  is the seam, deliberately left unconnected so there is one classifier, not two
- **`theo/turret-autoaim` still has no PR to `main`.**
  `elliott/hardware-config-2026` merged the branch directly (same SHAs, so a
  later `main` merge stays clean), but `main` itself is untouched
- **Decide how one codebase runs both robots before this branch merges.** The
  2025 and 2026 robots should drive the same but have different mechanisms.
  `RobotHardware.init()` looks up all 12 devices with `hardwareMap.get()`, so
  one missing device crashes INIT (2026-10-27: `indexer` on the 2025 robot).
  Once this merges to `main`, **no TeleOp on `main` will run on the 2025 robot,
  not even drive.** First question: will the 2025 robot keep being used for
  drive practice or auto testing after the 2026 robot drives? Options:
  1. **Retire it.** Tag the last commit that works on the 2025 robot and move on.
  2. **Drive-only TeleOp (recommended).** Use only the Pedro follower
     (`startTeleopDrive()` / `setTeleOpDrive()`), nothing from `RobotHardware`,
     so it runs on either robot. Do it together with the two cleanups below.
  3. **Optional mechanisms.** Each subsystem uses `hardwareMap.tryGet()` and
     turns itself off if its devices are missing, with a loud warning on the
     Driver Station. Most robust, most work, and a quietly disabled mechanism
     at competition is worse than a crash at INIT.

  Two cleanups whichever option is picked:
  - **Delete the drive `setDirection()` calls in `RobotHardware.init()`.**
    `Constants.createFollower()` runs afterward and Pedro's `Mecanum` sets all
    four directions from `Constants.java`, so these do nothing and contradict
    it. Drive directions live only in `pedroPathing/Constants.java`.
  - **Check drive directions on the 2026 robot once** (stick forward / strafe
    right / turn clockwise). `connor/drive-motor-directions` fixed and tested
    them on the 2025 robot only. Same Constants should work if the motors and
    Pinpoint pods are mounted the same way, but that has not been checked.
