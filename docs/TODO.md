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

### 3. Measure the pollen / nectar pivot offsets
**Why:** `ShooterPivot.POLLEN_OFFSET` and `NECTAR_OFFSET` are both `0.0`
placeholders. Homing works and then parks the pivot at a gap that is almost
certainly useless.

Run **Shooter Pivot Home Test** and follow the steps in its header:
1. Block the beam with a finger, confirm `Beam Broken` reads `true`. Flip
   `BEAM_BROKEN_STATE` if not
2. Home a few times, confirm `Zero Drift` stays under 0.005. If not, raise
   `FINE_DWELL_MS` — the servo is not finishing its steps before the beam is read
3. Jog onto a real pollen ball until it just fits, read `Offset From Zero`,
   write it into `POLLEN_OFFSET`. Repeat with nectar
4. Verify with B and X

Until step 2 passes, do not bother with step 3 — the offsets would be measured
against an unreliable zero.

---

## Needs a decision, not the robot

- **The shooter gear ratio is contradictory.** `ShootAprilTag` says direct drive
  (1:1); `ShooterVelocityTest` on `elliott/shooter-velocity-test` says 3:1. Both
  cannot be right and every shooter RPM number depends on it. TODO'd in both files
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
- **Indexer direction is a guess.** `RobotHardware` sets it `FORWARD`; flip it if
  a ball goes the wrong way. Confirm in **Collection System Test** before trusting
  any firing sequence
