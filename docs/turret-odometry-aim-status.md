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

## Turret lead (feedforward): TUNED on 6494, and it works

Shane, 2026-11-11, stepping `AimLead.LEAD_SEC` with the D-pad in
`Turret Odometry Aim Test` and watching `Aim error PEAK`:

> **"somewhere between 0.1 and 0.15 were the best results. Anything over that
> and it starts to lead."**

Stored as **0.12** in `RobotProfile` for 6494. Per robot, because the right
value is `1 / (servo deg per sec * Turret.kP)` and both differ. 5618 is 0, i.e.
off, until someone tunes it the same way -- another robot's lead would aim ahead
of the target.

### What that number tells us about the servo

Rearranged: servo speed = `1 / (LEAD_SEC * kP)` = `1 / (0.12 * 0.020)` =
**about 420 deg/sec at full power**, so roughly **330 deg/sec at MAX_POWER
0.80**.

That closes the loop on the saturation Shane measured. A fast FTC chassis spins
somewhere around 180-360 deg/sec, which is right at or past that ceiling -- so
"the turret cannot track a full-speed spin" is now a measured number rather than
an inference, and no amount of kP or lead changes it. The levers are MAX_POWER
(0.80 -> 1.0 buys about 80 deg/sec, against the overrun risk on a robot with no
hard stops) or not spinning that fast while holding aim.

Also worth noting it means the earlier guess of ~200 deg/sec was low by half,
which is why theory said 0.25 and the robot said 0.12.

## NOT YET IN THE OP MODES THE DRIVERS USE

**`DriveTeleOp`, `RouteAuto` and `PollenShotTune` aim by CAMERA (`TurretAim`),
not by odometry.** They never call `OdometryAim`, so there is no desired angle
to lead and the tuned lead above reaches none of them. It cannot be switched on
there with a flag.

Getting it to the drivers means the "After the spin test passes" job above:
wiring odometry aiming into `DriveTeleOp`. Which rests on the POSE -- so it
wants to wait until after the current mechanical changes and the odometry
retune, or it gets tuned against a pose that is about to move.

**What the drivers CAN usefully test today:** the cell-centring fix in
`TurretAim.bearingOf()` is already in `DriveTeleOp` and is a real change to how
camera aiming behaves -- before it, the aim point moved up to 6.5 in. depending
on which tags were in frame, and 1-2 tags is the normal case. `CENTRE_ON_CELL`
turns it off if it misbehaves.

---

# 2026-11-12 / 11-13: what the robot said, and what it ruled out

A long bench session on 6494, mostly measurement. Read this before touching the
aim again -- several of the obvious theories are already dead, and two of them
were mine.

## THE BUG THAT WAS ACTUALLY BREAKING MATCHES: gamepad 1 X

`DriveTeleOp`'s X ("the robot faces away from me right now") used to overwrite
the POSE heading with `awayFromDriversHeading()`. Harmless while the turret
aimed by camera. Fatal once it aims by odometry: press X while the robot faces
any other direction and the heading is wrong by that much, the turret aims at
where the hive would be for the fake heading, clips, and sits on a soft limit.

Worse, it hides itself. Once the camera/odometry disagreement passes
`MAX_TRIM_DEG` (15), `updateTrim()` rejected every measurement, so the aim ran
on pure bad odometry with the camera locked out -- a consistent one-sided miss
that showing the camera more tags cannot fix, because the measurements were
being computed and binned. Shane reproduced it twice: once with X, once by
forcing the trim off with "Aim test: no camera trim".

FIXED two ways. X now moves `driverHeadingOffset`, which only field-centric
driving reads; `follower.setPose()` is gone from the op mode. And a sustained
rejection now drops `poseTrusted`, clears the stale trim, and hands the turret
back to the camera, saying so on the Driver Station.

**If an aim ever goes consistently wrong again, read the "Aim by" line first.**
A trim that is frozen rather than tracking is this failure.

## MEASURED: the tag fixes are good, centred

With the robot parked and the turret held still, 8 captures in Tag Pose Test:

    0.6 in, 0.8 deg

That is the noise floor of the whole chain -- tag solve, camera mount, turret
encoder, everything. **Localization is viable.** This was the open question
behind wiring `TagLocalizer`, and the answer is yes.

## MEASURED: TICKS_PER_DEGREE = 78.51 is CORRECT. Do not change it.

Confirmed three independent ways:

- REV Through Bore V2 spec, 8192 counts/rev
- CAD gear ODs: ring 142 mm at module 2 = 69T, encoder gear 44 mm = 20T
- On the robot, 2026-11-13: pushing the turret by hand in Turret Range Test and
  counting ring teeth past a fixed reference. 41775 ticks over 102 tooth
  intervals = **409.56 ticks/tooth against 409.6 predicted (0.01%)**

Watch the fencepost: the raw counts were 37 / 24 / 44 teeth "lined up", which is
36 / 23 / 43 INTERVALS. Divided by the as-counted numbers the three runs scatter
over 7.8 ticks and miss; divided by N-1 they scatter over 1.4 and land exactly on
prediction. Count intervals, not teeth.

Also measured: the encoder does NOT slip. Returning the turret to a tape mark
five times repeated within 50 ticks (0.7 deg), which is hand-positioning.

## RULED OUT, with the reasoning, so nobody re-derives it

**The turret encoder scale.** See above. A tag sweep across the turret's travel
suggested a 10% scale error; the tooth count says 3%, and corrected for the
fencepost, 0%. The sweep was wrong, not the gears. Nothing was changed.

**`TurretAim.bearingOf()`'s row-offset sign.** The theory was that
`HiveTags.rowOffsetOf()` -- a hive-frame offset -- is subtracted straight from a
camera-frame x with no account of which side the row is viewed from, so it would
double the error instead of removing it. Testable prediction: the miss gets
WORSE with fewer tags in frame, since four tags cancel and one does not. Tested
by covering 3 of 4 tags: barely changed. **Dead.** The sign is fine, or at least
is not what is biting. (Note "Aim test: no cell centring" is a no-op whenever 4
tags are visible, so that row only tests anything at 1-2 tags.)

**The deflector being removed.** Cannot throw -- `Deflector` uses `tryGet` for
both devices and `isPresent()` guards every method.

## STILL OPEN: why the fix degrades across the turret's travel

Parked, sweeping only the turret, the fix moves:

| sweep | position spread | heading spread | fitted slope |
|---|---|---|---|
| 49 deg | 4.13 in | 7.0 deg | 0.143 |
| 49 deg | 4.33 in | 4.9 deg | 0.100 |
| 23 deg | 1.31 in | 2.6 deg | 0.108 |

Facts worth keeping:

- The robot never moved, so this is all in the tag -> turret base -> robot chain.
- `4.2 in / sin(5.9 deg) ~ 40 in`, the tag range. The position error IS the
  heading error projected over the range -- one root cause, not two.
- The slope reproduces from different robot positions and sweep spans, so it is
  not the pose or the hive position. SPREAD does not reproduce, because it
  depends on which angles were sampled -- compare slopes, not spreads.
- Residual about the fitted line is 0.46 deg, at the noise floor. It is a clean
  linear-in-turret-angle effect, NOT backlash and NOT a nonlinearity.
- The encoder is exact, so the slope is NOT the encoder.

**CONFIRMED 2026-11-13: frame-edge bias.** The turret can only hold
tags from +29 to -20 before losing them, which is roughly the camera's whole
usable FOV. A sweep therefore walks the tags from one edge of the image to the
other, and uncorrected lens distortion biases a solved pose most at the edges.
That would be smooth, repeatable, roughly linear in angle, and confined to the
FOV -- which is what the table shows.

### How it was confirmed, 2026-11-13

Park, home, aim the turret at the cell, capture. Then ROTATE THE ROBOT in place,
re-aim the turret at the cell so the tags stay CENTRED, capture again. Five or
six headings. Same range of turret angles as the sweep, but the tags never leave
the middle of the image.

Robot x/y does not change when it rotates in place, so position SPREAD is still
the right statistic. (The heading slope is NOT -- the robot really did rotate,
so the slope and the implied TICKS_PER_DEGREE line read nonsense on this test.)

    noise floor, one angle one spot ......... 0.60 in
    turret angle varied, tags CENTRED ....... 1.26 in   (~1.1 in attributable)
    turret angle varied, tags across frame .. 4.2  in   (~4.2 in attributable)

About **4x more error with the tag at the edge of the image than centred, over
the same turret travel.** So the turret chain contributes **at most ~1.1 in, and
that is an upper bound** -- rotating the robot by hand moves it a little for
real, so part of the 1.26 is genuine movement rather than error.

Consequences:

- **4 in was a worst case the sweep manufactured** by pointing the camera away
  from the target. In normal use the turret points at the cell, the tags ARE
  centred, and repeatability is ~1.26 in or better.
- There is an obvious filter rule to add: reject a fix whose tag is far from the
  image centre. `AprilTagDetection` carries `center` in pixels and
  `ftcPose.bearing`, so gating on roughly |bearing| < 10 deg would drop the bad
  fixes automatically instead of trusting nobody takes one.
- Still worth checking whether the SDK has a real intrinsic calibration for this
  webcam, since that is the likely root cause of the edge bias.

## The camera-mount question: SETTLED, and not by localization

Taking the camera off the turret would allow a 360 deg turret (no cable across
the joint) and would delete the turret chain from localization. Measurement says
do not bother:

- the turret chain's encoder is exact, so there is nothing to fix there;
- the turret chain's total contribution is at most ~1.1 in (above);
- the dominant error is frame-edge bias, which a body-mounted camera does NOT
  fix -- and moving it gives up the turret's habit of centring tags, which is
  worth more than the ~1.1 in it would recover.

So **decide the 360 deg turret on strategy, not on localization accuracy.** And
cost it properly: `Turret` is built around `MIN_ANGLE`/`MAX_ANGLE`, a magnet
band, a one-direction 150 deg homing search, and `OdometryAim` has a blind-arc
and hysteresis layer that exists only because travel is less than a circle.
Check too whether anything besides the camera crosses the rotating joint.

## NEXT: the measurement that decides whether to wire TagLocalizer

Absolute accuracy under OPERATING conditions, which is not what the sweep did:

1. Park, home the turret, and AIM IT AT THE CELL so tags sit near frame centre.
2. Set the truth (gamepad, see below) and read `Error`.
3. 3-4 captures, then repeat at four or five robot positions, including the
   spots you actually shoot from.

Consistently inside ~2 in -> wire `TagLocalizer` + `VisionPoseFilter` into the
pose. Both are written and unit-tested; the blocker is that `TurretCamera` builds
its processor with `FieldAprilTags.getLibrary()` and no `setCameraPose()`, so
`detection.robotPose` is null and `TagLocalizer.solve()` returns null on its
first line. Tag Pose Test configures its own processor to get around that.

Position-dependent and large with tags CENTRED -> the problem is upstream in
`HiveTags` (`RED_HIVE_X`, `CELL_ARM`, `CELL_DROP` are still made up) and the hive
needs measuring before any of this works. See `docs/hive-opening-measure.md`.

Two known gaps before `TagLocalizer` goes live: `turretDegAtFrame` is the CURRENT
turret angle, not the angle at frame capture (fine parked, wrong while slewing --
needs a timestamped angle history), and setting a camera pose on the processor
`TurretAim` reads may change whether `ftcPose` stays camera-relative. VERIFY that
before touching `TurretCamera`; `bearingOf()` depends on it. A second processor
on the same portal avoids the question at a CPU cost.

## Setting the truth in Tag Pose Test, without Panels

`TRUTH_X` / `TRUTH_Y` are editable from gamepad 1: **D-pad Up/Down** picks X, Y
or heading, the **triggers** move it (RT up, LT down). **X** loads
`GARDEN_EDGE_START` for when the robot is parked there.

Coordinates are Pedro field inches, `FieldConstants`: (0,0) is the corner where
the RED alliance wall meets the AUDIENCE wall, +x toward blue, +y away from the
audience, heading 0 facing +x. **Measure to the robot's CENTRE OF ROTATION** --
that is what the Pinpoint reports (`strafePodX(0)`, `forwardPodY(6.89)`), and
what `RobotProfile` means by "4.5 in behind centre".

## Late 2026-11-13: start here tomorrow

Everything above was written mid-session. Four things changed after it, and the
last one puts a question mark over the accuracy numbers.

### CAMERA_PITCH_DEG 35 -> 38, measured

Phone inclinometer on the camera body: 38. Two independent numbers agreed --
Tag Pose Test had the turret base solving 3 deg off level, and the base is
physically level, so the level error WAS the declared pitch being 3 deg wrong.

A prediction was written into that commit: level should fall under 1 deg, z
from +3.8 to about +1.2, and the position error should get slightly WORSE
(a too-low pitch reads range too long, which was partly cancelling the range
under-read). The re-run could not test it -- see "the readings were bouncing".

THIS ALSO MOVED CAMERA AIMING. applyCamera() pushes the same number into
TurretAim, which DriveTeleOp uses in matches. 38 should aim better than 35, but
that is unverified. **Check DriveTeleOp still tracks the cell before trusting
the robot in a match.** That is the one outstanding risk from today.

### The readings were BOUNCING, and nothing said so

z was reported as "+3.8" at one spot and "-2 to +4" at another. A single sample
of an unstable quantity reads exactly like a measurement. Tag Pose Test now
shows z, level and error as a min..max RANGE over the last second and prints
BOUNCING past 1.5 of swing. If it says BOUNCING, do not write the number down.

### Tag count decides fix quality -- this is the big one

Observed directly: with FOUR tags the turret base solved to z ~ 0, which is
where it physically is. With ONE tag it solved to z ~ 3 and swung -2..+4.

That is planar pose ambiguity: a single AprilTag has two nearly equally good
solutions and the solver picks wrong a fair fraction of the time. combine()
out-votes a flipped tag when there are others; with one, it can do nothing.

VisionPoseFilter gained rule 6, TOO_FEW_TAGS, MIN_TAGS = 2.

### Detection was crippled, and it was my doing

Tag Pose Test could only hold ONE tag at a time. It was built on AUTO exposure
and the default decimation on the reasoning that a parked robot has no motion
blur -- but exposure and gain also decide whether a tag DECODES, and
TurretCamera's 6 ms / gain 250 were tuned on these tags. The bring-up tool was
detecting worse than the match camera.

Fixed: 6 ms, gain 250, and DECIMATION 1 (not 2) to buy range on a 3.25 in tag
at 30-60 in. A bench test can afford the CPU.

### So the five-spot accuracy run is SUSPECT

Those five spots were measured with the bad camera settings, so some were
probably single-tag fixes -- which means they may have measured pose ambiguity
rather than any calibration error. That would explain why every theory built on
the "9.8% radial" figure has since failed: the row-offset sign, the encoder
scale, PIVOT_Y, and the camera resolution were all chased and all cleared.

DO NOT reason further from those five numbers. Redo the run.

### Tomorrow, in order

1. Redeploy. Check **how many tags** Tag Pose Test can now hold. If 3-4 is
   reliable, decimation was the blocker.
2. Verify **DriveTeleOp camera aiming** still tracks, after the pitch change.
3. **Redo the five-spot accuracy run** with 3+ tags and steady (non-BOUNCING)
   readings at every spot. Record tag count with each error.
4. The most trustworthy single reading from today, for comparison: at (59, 9)
   with 3 tags, z ~ 0.5, error 7 in. If the redone run agrees, ~7 in is real and
   worth chasing. If it comes in much lower, the error was ambiguity all along.

Still unmeasured on the robot, and the remaining candidates if ~7 in survives:
CAMERA_LEFT and CAMERA_YAW_DEG, both assumed 0. The steady -2 to -3 deg heading
error is what a small yaw looks like.

---

# 2026-11-13 (from home): wall re-anchor, and the blue-side test

Theory session, no robot. Two additions to Tag Pose Test, both read-only.

## X now cycles all four wall starts

Red audience -> red scoring -> blue scoring -> blue audience. The screen says
which preset is loaded ("none (hand-set)" once the triggers touch it).

**The mirrored-pair test.** Red audience and blue scoring are exact 180 deg
rotations of each other (so are the other two), so the camera sees the same
picture from each. An error that lives on the ROBOT (camera yaw/left/pitch,
turret zero, lens) FLIPS SIGN in field coordinates between the pair. An error in
the FIELD MODEL (HiveTags, the blue hive placed by symmetry) does not. Before
running it:
- tip the hives into mirrored states (red audience cell up <-> blue scoring up);
- write down the prediction that ~2 in of x difference is our FIELD (58 vs 60
  in wall-to-hive), not the code;
- do it AFTER the 3+ tag redo on red, or it compares ambiguity noise.

## The WALL line: re-anchoring on a wall, measured before it is built

Shane's idea: the drivers hit walls constantly, so let a button press say "we
are square on this wall" and let the camera supply the rest. `WallFix` does the
math: odometry heading rounded to the nearest 90 (refused past 10 deg), the wall
fixes one coordinate, and the camera's horizontal BEARING to each tag gives the
position along the wall. No range, no camera height, no tag orientation -- so
none of pitch, edge-of-frame heading bias, or single-tag pose ambiguity.
One tag is enough. `WallFixTest` covers the math (10 tests).

On screen, under ERROR:

    WALL  <-- and this   AUDIENCE x 33.4, err +0.3..+0.6in
      wall q             RIGHT face, 4 tags, spread 0.4 in, 1.1 in/deg, snap +0.0 deg

- `err` is the along-wall error only -- the one number the camera supplies. The
  other coordinate and the heading come from the wall, so against a preset they
  are right by construction.
- `in/deg` is how many inches one degree of bearing error costs here. ~1 at the
  wall starts. It blows up as the sight line approaches parallel to the wall,
  and sightings under 20 deg are refused.
- Each preset knows its touching face (audience wall = RIGHT, scoring = LEFT,
  both alliances). Hand-set truth uses `WALL_FACE`, default FRONT (collector).

**What to compare at each preset:** ERROR line's along-wall component (dx on
the audience/scoring walls) vs the WALL err. If WALL is consistently within
1-2 in while the full solve is not, the re-anchor is worth wiring into
DriveTeleOp. A steady WALL bias that flips sign across a mirrored pair is a
bearing bias -- CAMERA_YAW_DEG or TURRET_ZERO_DEG -- worth ~1 in per degree.

**Unverified assumption:** WallFix reads `ftcPose` as CAMERA-relative. Tag Pose
Test's processor has a camera pose set, and nobody has confirmed that leaves
ftcPose camera-relative (it should; robotPose is the robot-relative one). If WALL
is wildly off while the full solve is sane, suspect this first.

**For the match version (not built):** collector-first is the only workable
convention, and it needs the turret flipped -- back-first puts the camera facing
the wall. Guards to add: robot stopped + turret settled before accepting, refuse
an answer on a flower segment or off the field, rumble accept/refuse, and on
accept reset poseTrusted and clear the trim (TurretDirector.resetPhase).

## Next thread, proposed 2026-11-13: distance -> RPM, and the flywheel offset

Not built. Waiting on answers to the questions below.

- **Localization is not the limit.** Range goes roughly as v^2 at a fixed
  launch angle, so a distance error of 2 in at 60 in (3%) needs about 1.7%
  RPM, roughly 40 RPM at 2400. Flywheel consistency (PIDF untuned, dip and
  recovery between shots) is likely worse than that.
  `shooting/ShotTable.java` already does distance -> RPM interpolation and
  is EMPTY.
- **The flywheel offset depends on TURRET ANGLE, not heading or bearing**,
  IF the flywheels are fixed to the chassis and fire up through the turret
  hole onto the deflector (confirm this). A ~0.75 in offset to the robot's
  left lands sideways on the deflector at turret 0 (left/right aim bias), and
  fore/aft at +/-90 (long one way, short the other). So the model is: a
  ShotTable over distance, plus small corrections of the form
  a*cos(turret) + b*sin(turret) for both aim and RPM. That is two or three
  numbers each, NOT a 2-D grid.
- **Proposed "Shot Map" test:** live distance and turret angle on screen,
  outcome buttons (HIT/SHORT/LONG/LEFT/RIGHT), and every shot logged to CSV
  with pose, distance, turret angle, RPM of both wheels at release, deflector
  angle and shot number in the volley. Find the EDGES of the hit band at each
  distance, not one RPM that happened to work; the middle of the band is the
  table value, and its width is the tolerance.
- **Questions for Shane:** is the layout above right? Is the bad flywheel
  encoder fixed (the velocity loop depends on it)? RPM only with the deflector
  fixed, or both? A new branch?

## BUILT 2026-11-13: Shot Map (branch `shane/shot-map`, off deflector-autohome)

Shane's answers: layout confirmed (chassis flywheels fire up through the turret
hole onto the deflector), RPM is the ONLY knob (deflector fixed at
DEFAULT_SHOT_DEG), new branch. The flywheel-encoder question went unanswered;
the screen shows L/R RPM and every log row carries both, so a bad encoder will
show up as WHEELS DISAGREE or a skewed column.

**Op mode:** `Shot Map`, group Tune (`ShotMapTune.java`). Not run on a robot yet.
- INIT: pick the wall start you are parked on (the four WallStarts, now shared
  in `field/WallStart.java` by Tag Pose Test too), and the aim source.
- One ball, one press: A hit, D-pad down SHORT / up LONG / left LEFT / right
  RIGHT. The RPM jumps to the next value worth trying; the "band" line says
  DONE when both edges are found, then B saves the spot.
- RB/LB nudge RPM; LT+RB/LB nudge the aim trim (applied via
  TurretAim.AIM_BIAS_DEG, restored on stop). X undoes. BACK on gamepad 1
  re-syncs the pose when parked back on the start spot.
- Log: `/sdcard/FIRST/shotmap/shotmap-<robot>-<date>.csv`, a "shot" row per
  press and a "spot" row per B. `adb pull /sdcard/FIRST/shotmap`.

**The search** (`shooting/ShotMap.java`, 14 laptop tests): it finds both EDGES
of the hit band, not one lucky RPM. Centre -> ShotTable, width -> tolerance.
About 16-20 balls a spot at 10 RPM resolution; RPM_RESOLUTION defaults to 25.
Set it from the band widths you actually see.

**Session plan:**
1. Distance table, turret near 0: 4-6 spots across where you actually shoot.
2. Offset sweep: at 1-2 distances, rotate in place so the turret sits at about
   -90 / -45 / 0 / +45 / +90, and map each. The FIT line shows
   `rpm a cos + b sin` and `aim c + a cos + b sin` once there is enough data.
   On 6494 expect something; on 5618 (turret centred) expect ~0.
3. Copy the spot rows into `RobotProfile` (`ShotTable.DISTANCES/RPMS`, PER
   ROBOT, empty on both now). A rebuild that moves the flywheels relative to
   the turret voids that robot's table and fit.

**Not built yet:** using the table and fit in DriveTeleOp (spinUp(lookup(d)) +
the turret-angle corrections). Do it after a table exists.

### Revised the same day: the LADDER (Shane asked how it copes with random balls)

It didn't. The first version kept only extremes (highest short, lowest long),
so one bad ball between two hits permanently wrecked the band. At ~1200-1300
RPM with 25 RPM steps, ball-to-ball scatter is likely bigger than a step, so
that would have been the normal case. Replaced before any data was taken.

**Now:** rungs every `RPM_RUNG` (25), `BALLS_PER_RUNG` (4) balls on each,
short/hit/long tallied per rung. The ladder only moves once a rung is full:
up if mostly short, down if mostly long, outward if mostly hits, then fills
any gap. The answer is the BALANCE POINT where longs and shorts cancel,
interpolated between rungs, using every ball. The screen also shows:
- **best hit rate**: low on every rung = the balls/mechanism are the limit,
  not the RPM, and a finer table buys nothing;
- **transition width** (score -0.5 to +0.5): how much RPM or distance error the
  shot forgives.

Laptop simulation (scatter about as wide as the hit band): within 15 RPM of
the true centre in 191/200 runs, median 16 balls a spot, 20 at the 90th
percentile. Aim uses the same ladder but only moves when a rung is mostly
left or right. The offset fit is now weighted by each spot's certainty and
prints a +- on every term: a term inside ~2x its +- is noise (the expected
answer on 5618).

Steps, per Shane: Shot Map nudge 10 RPM, rungs 25. Pollen Shot Tune's
flywheel step went 50 -> 25, fine stays 10.
