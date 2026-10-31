# Reassembly checklist — bringing the robot back up after a teardown

What to run, in what order, after the robot has been apart. Written 2026-10-31,
when the 2026 robot was disassembled and rebuilt.

**Why there is a whole document for this.** A teardown is the single most
invalidating event for this codebase. Almost every number in the repo was
measured against a specific assembly, and the code has no way to notice that
assembly is gone — it keeps commanding the old positions and the shots quietly
go bad, or worse, something gets driven into a hard stop. The order below is a
dependency order, not a preference: each phase re-earns what the next one
assumes.

Work top to bottom. If a step fails, fix it before moving on rather than
skipping ahead — a bad measurement is worse than a missing one, because a
missing one is obvious.

Related:
- [hardware-config.md](hardware-config.md) — what is plugged in where
- [turret-bringup.md](turret-bringup.md) — the turret's own first-run procedure
- [TODO.md](TODO.md) — the open items this checklist feeds

---

## READ FIRST: do not command the pivot yet

`ShooterPivot.MIN_SAFE_POSITION` (0.166) and `MAX_SAFE_POSITION` (0.679) were
measured against the **old** linkage seating. Every pivot command is clipped to
that band, and that clip is the only thing standing between the servo and a hard
stop. If the linkage went back together even slightly differently, 0.166 may now
be *past* a stop — and pressing a powered servo into a stop is what wrecked one
on 2026-10-30. Twice.

**So `Pivot Jog Test` (step 12) runs before any other op mode that commands the
pivot.** Until it has, do not run:

- `Load Sort Test`
- `Shooter Pivot Home Test`
- `TestTeleOp`

---

## Phase 0 — before the battery goes in

No op modes. Paperwork, and it is the cheapest it will ever be.

**1. Check config names against [hardware-config.md](hardware-config.md).**
Exact string matches; `Color Sensor` and `colorSensor` are different devices to
the SDK, and the wrong one crashes `init()` rather than warning. Two renames
from TODO item 1 are still outstanding:

| hand-written map says | code expects |
|---|---|
| `Pivot Sensor` | `pivotHome` |
| `Color Sensor` | `colorSensor` |

**2. Pull the config XML into the repo** (TODO item 1). You are rebuilding the
config anyway, so this is the moment. After it is committed, any hub is one
`adb push` from a working config and wiring changes become reviewable diffs.

```
adb connect 192.168.43.1:5555
adb shell ls /sdcard/FIRST/settings/
adb pull /sdcard/FIRST/settings/<Name>.xml docs/robot-config/
```

**3. Confirm the turret encoder went back into `lf`'s encoder port, not `rf`.**
It was deliberately moved on 2026-10-27 so reads stop crossing RS485 to the
Expansion Hub. It is an easy thing to put back where it used to live.

**4. Ask whoever rebuilt it: was the pivot linkage re-seated lower in its
range?** That was the recommendation from the 2026-10-30 finding — nectar grips
only 0.044 from the widest position while 0.14 of travel above pollen does
nothing, so there is almost no clearance to get a nectar ball in or out. If they
acted on it, the pivot numbers moved a lot. If they did not, now is the easy
time to ask.

---

## Phase 1 — does everything exist? (no motion)

**5. `Digital Port Scan`** — `pivotHome` and `turretHome` present and readable.

**6. `Camera Test`** — the webcam enumerates.

**7. `TestTeleOp`, INIT ONLY. Do not press play.** `RobotHardware.init()` calls
`hardwareMap.get()` on all 12 devices, so one missing or misnamed device is a
crash at INIT. Reaching INIT without an exception is the fastest whole-robot
inventory check there is.

**8. `Magnetic Sensor Test`** — the turret home switch fires and the zone width
looks sane.

---

## Phase 2 — directions and signs

Nothing is calibrated yet. This phase only asks "which way does it go". Go slow
and use low powers.

**9. `DriveTeleOp`** — stick forward drives forward, strafe right goes right,
turn clockwise turns clockwise.

This has **never been verified on the 2026 robot at all**;
`connor/drive-motor-directions` fixed and tested it on the 2025 robot only.
`Load Recovery Test`'s `SNAP_SIGN` depends on the answer, and so does every
auto path.

Worth knowing while you are in here: `RobotHardware.init()` sets all four drive
directions **exactly inverted** from `pedroPathing/Constants.java`, and
Constants only wins because `createFollower()` runs afterward. Drive directions
live in `Constants.java` and nowhere else. The `RobotHardware` lines are dead
and contradictory and are slated for deletion (TODO, two-robots cleanup).

**10. `Collection System Test`** — `collector` REVERSE and `indexer` FORWARD
still correct, and the indexer feeds **up** toward the shooter. These were
confirmed 2026-10-29, but a motor that was unplugged and remounted can come back
reversed.

**11. `Shooter Flywheel Test`** — both wheels spin inward, not fighting each
other.

---

## Phase 3 — re-establish limits

**12. `Pivot Jog Test`** ← the gate for everything pivot-related.

Re-measure and write in all five constants. The op mode prints them in
copy-paste form:

```
MIN_SAFE_POSITION   widest, backed off the hard stop
MAX_SAFE_POSITION   tightest, backed off the hard stop
OPEN_POSITION       ball entry and extraction
POLLEN_POSITION     pollen grip, against a real ball
NECTAR_POSITION     nectar grip, against a real ball
```

The old values are in [TODO.md](TODO.md) for comparison, not for reuse. If the
new numbers differ from the old ones by a roughly constant amount, the linkage
or horn was re-seated. If they differ by *different* amounts, the geometry
changed — which is the whole reason this mechanism re-measures instead of homing
(see `ShooterPivot`'s class comment).

**Record `OPEN_POSITION` deliberately, not just as jogged.** It is used for ball
*entry* now, not only extraction, so it wants real clearance below the nectar
grip. 0.17 was chosen rather than measured on 2026-10-31 for exactly that
reason.

**13. `Turret Range Test`** — homing finds zero, travel matches the 286° in
[turret-bringup.md](turret-bringup.md).

---

## Phase 4 — re-earn the measurements

**14. `Indexer Load Test`** — 20 labeled passes, ten of each ball, at 50 RPM.
Re-derive `BallLoadClassifier.BASELINE_AMPS` and `NECTAR_RISE_AMPS`.

**Do not skip this assuming the old 0.79 / 0.72 survived.** `BallLoadClassifier`
lists "mechanical changes" as invalidating the calibration, and a teardown is
the largest one available. What you are checking is not just the two numbers but
that the *gap between the groups is still wider than either group's own spread* —
that is the test that decides whether current sensing works at all. It was
1.32 A against spreads of 0.30 and 1.25 on 2026-10-30.

While you are here, capture the `Indexer Amps` jam number nobody has ever
written down. Any future unjam logic needs it.

**15. `Shooter Flywheel Test`** — measure the real no-load top speed.

This has never been done, and two shooter targets are currently unreachable
because of it: `ShootAprilTag.SHOOTER_VELOCITY_RPM` defaults to 2000, which is
about free speed through the 3:1, and `TestTeleOp` lets you dial 3000. Both
saturate the motor, and a saturated velocity PID looks exactly like a broken
one. Measure the top speed, then cap both around 85% of it.

---

## Phase 5 — integration

**16. `Load Recovery Test`** — direction check first, then the comparison.

Direction check, with **no ball in the robot**: set `SNAP_POWER` to ~0.3 from
the dashboard, press A, and confirm the robot lurches **forward**. Backward
sends a ball toward the front of the robot and possibly out of it. Set
`LoadRecovery.SNAP_SIGN = -1.0` if it goes the wrong way, then restore
`SNAP_POWER` to 1.0.

Then put a ball in the dead space between collector and indexer by hand and run
both maneuvers (A = chassis shake, B = collector pulse) against the same ball.
Full list of what to collect is TODO item 4.

**17. `Load Sort Test`** — the end-to-end chain, and the thesis of
`shane/indexer-load-sensing`.

- `Nectar caught early` should flip to **YES while A is still held**. If it only
  flips on release, the mid-pass check is not seeing the current rise
- the pollen move should finish before the ball reaches the flywheels. Pollen
  travels from open to 0.539 now, which is further than it used to, and the
  ~0.6-1.2 s of ramp travel it relies on is an estimate in a comment that has
  never been measured. A late close does not jam — it just fails to grip
- ten of each ball, and watch `Margin from threshold` stay well clear

**18. `Turret Aim Test`** — confirm `ENCODER_DIRECTION` is still 1.0 (if the
reported angle goes DOWN on positive power, set it to -1.0; that is the only
place turret sign should ever be adjusted). Capture `Frame age` in ms and how
far `Bearing raw` jitters, which is what would let `DEADBAND_DEG` come down
from 5.0.

---

## When you are done

Update the numbers in [TODO.md](TODO.md) under a new "Confirmed on the robot"
heading with the date, and say what moved. The point of recording the old values
next to the new ones is that the *difference* is diagnostic — a constant shift
means a re-seated part, and a non-constant one means the geometry changed.
