# Odometry turret aiming -- where we stopped

**Branch:** `shane/deflector-autohome` (pushed). **Date:** 2026-11-10.
Code is deployed to 6494 and compiles; unit tests pass.

## Done today

- Deflector and turret both auto-home correctly off the menu **A-lock**.
- `TurretGeometry.TURRET_OFFSET_X = -4.5`, `TURRET_OFFSET_Y = 0.0`
  (Shane, tape measure: turret axis 4.5 in. behind centre, laterally centred).
  These were `Double.NaN`, so `isConfigured()` was false and the aim test
  refused to aim at all.
- `TagLocalizer.CAMERA_FORWARD` 4.0 -> **4.5** (measured: camera ~4.5 in. in
  front of the turret axis).
- Fixed `OdometryAim`'s header: it claimed +/-140 limits and an 80 deg dead
  zone. Real limits are -90/+90 (6494), so the blind arc is the robot's whole
  rear half -- **180 deg**.
- Fixed the aim test's crash on start (see below).

## The crash, and why

`Turret Odometry Aim Test` threw, the instant you pressed play:

```
NullPointerException: Pose.getHeading() on a null object reference
  at com.pedropathing.VectorCalculator.setTeleOpMovementVectors(:328)
  at com.pedropathing.follower.Follower.setTeleOpDrive(:353)
  at TurretOdometryAimTest.runOpMode(TurretOdometryAimTest.java:102)
```

`VectorCalculator` keeps its OWN copy of the pose. Only `follower.update()`
fills it in -- `startTeleopDrive()` does not -- and `setTeleOpDrive()` reads
that copy's heading unconditionally. Fix: one `follower.update()` after
`startTeleopDrive()`, which is what every stock Pedro tuner in
`pedroPathing/Tuning.java` already does. We were the only place missing it.

## NEXT THING TO DO: run the spin test

`Turret Odometry Aim Test`, group **Test**. Defaults put the robot at (72, 72)
heading 90 deg with the target at (72, 132) -- straight ahead, 60 in. out, so a
correctly homed turret reads about 0 when you press play. Either set the robot
on that spot or change `START_X/Y/HEADING_DEG` to where it actually is.

INIT -> **X** to home -> **A** to aim. Then, in order:

1. **Strafe sideways** -- easy case, the offset barely matters.
2. **Drive a straight line** past the target.
3. **Spin in place** -- this is the case the 4.5 in. was for.

Sight along the shooter assembly at a tall thin object on the target mark; the
deflector works fine as a pointer (no laser needed).

**Watch `Turret axis x, y`.** At heading 90 deg it should read 4.5 in. BELOW the
robot's y, and as you spin it should trace a 4.5 in. radius circle around the
robot's position. If it does not move while spinning, the offset is not
reaching the math -- say so.

### Why the offset is specifically the rotation number

The turret axis is a physical point 4.5 in. off the pose origin, so when the
robot rotates, that point swings around the origin. `atan(4.5 / 72)` = 3.6 deg
at a 72 in. shot, and it CHANGES SIGN across a spin: ~7 deg of wander, while
being perfect standing still. That is the symptom Shane wanted gone.

## Then: did the turret speed-up help?

`Aim error (deg)` is `Wants - actual`, so during a steady spin it IS the
tracking lag.

1. Spin in place at a steady, repeatable rate. Note the peak `Aim error`.
2. In Panels set `Turret.kP` back to **0.010** and `MAX_POWER` to **0.55**
   (both live, no redeploy). Same spin.
3. Compare peaks.

Halving the lag means the speed-up earned its keep. Similar numbers mean we
were power-limited, not gain-limited, and `MAX_POWER` is the lever. Buzzing or
hunting when you stop spinning means kP is too high -- lower kP, not MAX_POWER.
Re-initialising restores 0.020 / 0.80, so you cannot strand yourself.

## Caveats to settle

- **"Centre" has to mean the POSE ORIGIN**, normally the odometry pod centre,
  not the chassis centre. If they differ, -4.5 is off by the gap. The spin test
  tells them apart: a wrong **X** drifts one way through the first half of a
  spin and the other way through the second half; a wrong **Y** shifts the
  whole spin one direction.
- **6494's phantom sideways odometry drift is still open** (`docs/TODO.md`
  section 0). Odometry aiming is only as good as the pose feeding it. If the
  spin test is erratic in a way that fits NEITHER pattern above, suspect the
  pose before the geometry.
- Aiming at the hive is blocked on three unmeasured numbers:
  see **`docs/hive-opening-measure.md`**.
- The turret's 150 deg INIT homing sweep is still not confirmed legal inside
  the 18 in. starting cube. Settle it when the turret gets flipped.

## After the spin test passes

Wire odometry aiming into `DriveTeleOp`. Recommended design: the camera
corrects the POSE (`VisionPoseFilter` / `TagLocalizer`), and the turret aims
from the pose. Today `DriveTeleOp` aims camera-only via `TurretAim` and never
touches any of the odometry path.

Note for that work: with the turret facing forward (`TURRET_ZERO_DEG = 0`) and
limits of -90/+90, the hive is UNREACHABLE whenever the robot's back is to it.
Driving so the hive stays in the front half is a real constraint. Flipping the
turret to face away from the collector moves the blind arc to the front.

---

# On-robot results, 2026-11-11 (Shane)

## Spin test: PASSED

"Looked pretty good." `TURRET_OFFSET_X/Y = -4.5, 0` stands. No numbers recorded,
so if the aim is ever suspected again, re-run it and write them down this time.

## Hive aiming: unblocked

`Target x, y` now reads real numbers instead of NaN. Every hive number used for
aiming is measured or is a manual value whose assumption Shane checked. See
`HiveTags` and `HiveGeometry` -- each constant carries its own provenance.

## Tag detection range: fine WHERE WE SHOOT FROM, not field-wide

Shane, clarified 2026-11-11: from the **shooting positions** the camera sees
**one to four tags whenever the hive is in view, and nothing is ever too far**.
Range is not a limit inside shooting range.

But NOT everywhere. With the hive facing up, the tags cannot be seen from the
far side of the field, or from much of the blue side outside our shooting range.
An earlier version of this note said "there is nowhere too far", which
overstated it -- that was only ever true of the places we shoot from.

Two consequences, and the distinction matters:

- **Tag correction is available where it counts.** Any scheme that corrects the
  aim or the pose while lining up a shot can expect tags.
- **It is NOT a continuous field-wide service.** Do not design anything that
  assumes tags during a full-field traverse -- an auto path crossing to the far
  side will lose them. Pose correction has to degrade gracefully into
  odometry-only, not depend on a fix always being there.
- **Seeing only 1-2 tags is the NORMAL case, not an edge case.** That is what
  made the cell-centring fix in `TurretAim.bearingOf()` load bearing: before it,
  the aim point moved up to 6.5 in. depending on which tags were in frame.

## Turret gain comparison: ANSWERED. kP is not the lever, MAX_POWER is.

Took three tries, because the measurement kept reporting its own limits:

1. Peak 245 both runs -- built on `Solution.desiredDeg`, which "may be
   unreachable", so it saturated on the 180 deg blind arc.
2. Peak "about 44" -- against a 45 deg magnitude cap. Read the cap.
3. Peak in the 40s even on a moderate spin -- the recovery coming OUT of the
   blind arc, where the turret restarts from a travel limit against a target
   already sweeping.

Each one is excluded now (see `TurretOdometryAimTest`: command-jump detection,
re-entering-reach detection, and a settling period that ends when the turret
first gets back inside `TOLERANCE_DEG`). The peak also says WHY it is paused.

**With all that excluded, steady-state tracking lag still exceeds 40 deg during
a spin, and the op mode's saturation line fires.** The turret is at full power
once the error passes `MAX_POWER / kP` = 0.80 / 0.020 = 40 deg, so past that kP
does nothing. The 2026-11-10 speed-up is NOT the thing to tune further.

So: **the turret cannot track a spinning robot at MAX_POWER 0.80.** Three ways
out, in increasing order of how much they actually fix it:

- **Raise MAX_POWER** (0.80 -> up to 1.0). Cheap, but 0.80 was chosen because
  6494 has NO hard stops and a faster turret coasts further past the +90 soft
  limit toward the tight cable end. Trades aim tracking for overrun risk.
- **Don't spin fast while holding aim.** A real driving constraint, free.
- **FEEDFORWARD -- the actual fix, and not yet built.** A pure P loop always
  lags a moving target; it only ever reacts to error that has already happened.
  But the target's motion is PREDICTABLE: most of it is just the robot rotating,
  and the pose already knows the robot's angular rate. Lead the turret's target
  by (angular rate x latency) and most of the lag cancels with no extra power.
  This is why more MAX_POWER is treating the symptom.

## An aim error of -9.6 deg while parked

Reported at a standstill on the tape mark, and it resolved as the robot moved.
Almost certainly the target sitting past a travel limit rather than anything
lagging: the old `Aim error` was `desiredDeg - getAngle()`, so it included the
part the turret is not allowed to reach. The `Reach` line said so but was easy
to miss next to a big number.

Now split in telemetry: **`Aim error`** is the tracking lag (command vs actual)
and **`Total off target`** is the whole miss including anything out of reach.
They agree only while `Reach` says OK.

## Hive aiming on the robot: WORKS, with a small random wander

Shane, 2026-11-11: "the targetting is fairly good. Sometimes it seems like it is
slightly right or left but it seems sort of random. When moving around it still
sort of comes back to a decent spot."

So the geometry is right -- a systematic error would lean one way, and this does
not. What is left is a few degrees of scatter, and the prime suspect is NOT the
geometry and NOT the pose:

**MEASURED 2026-11-11, AND IT IS NOT THE TURRET.** The theory was that
`Turret.MIN_POWER / Turret.kP` = 0.12 / 0.020 = 6 deg -- the smallest correction
the loop can command -- was too coarse for the 2 deg `TOLERANCE_DEG` band, so
the turret could never sit still. Shane ran the settle test in `Turret Test`
(HOLD, 2 deg bumper nudge, repeated both directions):

> **Overshoots were 0 or 1 every time. Worst distance past the target was
> 1.29 deg, and often very close to 0.**

So the turret settles cleanly in one move or one small ring. The theory is dead,
and with it the warning that tightening `TOLERANCE_DEG` would backfire.

**THEREFORE THE WANDER IS THE POSE.** Odometry hive aiming uses no camera at
all, so the only two contributors are the turret and the pose. The turret is now
measured and exonerated, which leaves 6494's odometry -- and the phantom
sideways drift in `docs/TODO.md` section 0 is already an open bug with exactly
the right shape: a few degrees, wandering, not leaning one way, and recovering
as the robot moves. **That is the next real job, and it is a bigger one than
anything in the turret.**

### What the measurement says about TOLERANCE_DEG

It CAN come down, but not far, and it is not worth much.

The worst overshoot was 1.29 deg. The turret stops once it is inside
`TOLERANCE_DEG`, so a band narrower than the overshoot means it lands outside,
corrects again, and starts hunting -- the failure that was feared, just at a
different threshold. **1.5 deg is defensible** (above 1.29 with a little room).
**1.0 is not.**

That buys 0.5 deg, about half an inch at 60 in., against a pose error of several
degrees. Real, but not the thing to spend time on. Earlier today this was ranked
third behind the tag-centring fix and the pose; that ranking was right, and the
mid-day reversal of it -- "tightening the tolerance would make it worse" -- was
based on the theory this test just killed.
