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
