# AprilTag tracking & accuracy notes (2026-10-08)

## What one detection gives you

| Field | What it is | Needs tag size in library? |
|---|---|---|
| `id` | Which tag | No |
| `center`, `corners` | Pixel positions in the image | No |
| `decisionMargin`, `hamming` | Detection confidence (higher margin = better; 0 hamming = best) | No |
| `ftcPose.x / y / z` | Tag position relative to the camera, inches (right, forward, up) | Yes |
| `ftcPose.range` | Straight-line distance | Yes |
| `ftcPose.bearing` | How far left/right of camera center the tag is (deg) | Yes |
| `ftcPose.elevation` | How far up/down of center (deg) | Yes |
| `ftcPose.yaw / pitch / roll` | Which way the tag itself is turned/tilted relative to the camera | Yes |
| `robotPose` | Robot's position on the field | Yes, plus `setCameraPose(...)` and field tag positions |

- **Bearing** = *where* the tag is ("10° to my left").
- **Yaw** = *which way the tag faces* ("turned 10° away from me").
- For following, use bearing. Yaw only matters for squaring up to the tag.

## Following a tag left/right — yes, plenty of info

1. **`bearing`** + proportional control: `turnPower = kP * bearing`. Needs tag in library with correct size.
2. **`center.x`** vs. image center: works without pose or calibration — most robust if you just want the tag centered.

Real limits are speed, not accuracy:
- **Frame rate / lag:** ~20–30 detections/sec at 640x480 on a Control Hub, each ~50–100 ms old. Too much gain → oscillation.
- **Motion blur:** auto exposure smears moving tags. Fix: manual exposure (~5–6 ms) with gain turned up.
- **Lost frames:** decide what to do when nothing is detected (hold last command briefly, then stop).

## How accurate are the angles?

Measure it — these are general patterns, not guarantees:
- **Bearing / x:** most accurate, usually within ~1–2°.
- **Range:** within a few % *if the lens calibration is right*.
- **Yaw / pitch / roll:** noisiest. Ambiguous near face-on (can jump or flip sign around 0°). Small 3.25" tags at distance → several degrees of jitter. Watch `decisionMargin`.

**Calibration caveat:** repo calibrations (`teamwebcamcalibrations.xml`) cover only Logitech C310 and C920. Other webcams → SDK guesses lens values → range/angles can be way off. **Which webcam do we have?**

## Measurement plan (use AprilTagIdTest)

1. Tag on a wall at camera height; robot at 12", 24", 48", 72". Compare range to a tape measure; note jitter while still.
2. At one distance, slide robot sideways — bearing should change smoothly.
3. Rotate tag (or robot) ~0°, 15°, 30°, 45°; compare yaw to a protractor. Watch near 0°.
4. Wave tag side to side at increasing speed; note when detections drop out → tracking speed limit.

## Open to-dos

- [ ] Identify webcam model (calibration).
- [ ] Decide: write a "tag-follow" test OpMode (turn to keep tag centered, show bearing error)?
- [ ] Fix camera name mismatch: `"Webcam 1"` (AprilTagIdTest, RobotHardware) vs `"webcam"` (CameraTest, vision/AprilTagAuto) — must match the robot config.
- [ ] Uncommitted `AprilTagIdTest.java` on `connor/red-startup` differs from the committed one on `connor/apriltag-test` — move, merge, or discard?
- [ ] Delete merged `evan/field-constants` branch? Delete stray `grep.exe.stackdump`?
