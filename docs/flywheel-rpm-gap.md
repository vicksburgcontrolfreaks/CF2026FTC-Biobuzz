# Flywheel RPM gap: right reads about half of left

**Status:** open. **The right wheel spins at full speed. Only its reading is low**
(slow-motion video, 2026-11-04). Next: find whether the cable, the right motor's
encoder, or CH port 2 is at fault (see "Isolating the bad part").

The **low** side is **`shooterRight`, Control Hub motor port 2.** The first write-up
had the sides backwards.

## What we saw

**2026-11-02 (after the rebuild):** about 1200 vs 400 (low side about 1/3 of
high). Mode unknown, possibly VELOCITY, which would change the gap. Always note
the mode.

**2026-11-04 (POWER mode):**
- Left ~1300 RPM, right ~600-650 at full trigger. Low side about **1/2** of high.
- The **percentage gap stays the same at every trigger position.**
- Amps L ~2.6, R ~2.2 at full.
- Both wheels feel the same spun by hand. Hand-spun RPM was too short to read.
- Both wheels "look the same speed," but 10 vs 22 rev/s are both a blur, so
  this doesn't prove anything.

## What that rules out

- **Intermittent dropped counts (a loose cable)** usually gets worse with speed.
  A gap that's a constant percentage acts like a fixed **multiplier**, so this is less likely.
- **Hub-to-hub PIDF differences:** POWER mode is open loop, and a ~2x gap is far
  too big anyway.
- **The `SHOOTER_GEAR_RATIO` setting:** both sides use the same 3.0, and the
  encoder sits on the motor, before the gearbox, so this setting can't skew one side.

## Still possible

- **Real:** the right motor really spins about half as fast (weak or damaged
  motor, binding in the right gearbox cartridge, or a different motor type or
  cartridge put in during the rebuild).
- **Reading:** the right wheel spins at full speed, but its encoder (or CH port 2)
  reports a consistently scaled-down count.

## Next test: slow-motion video (decides real vs. reading)

1. Robot off. Put bright tape on the edge of each flywheel.
2. Film each wheel in slow motion (240 fps if possible) at full trigger, POWER mode.
   Keep hands and the phone clear.
3. Count frames for one full turn. At 240 fps: 1300 RPM is about **11 frames/turn**,
   600 RPM about **24**.

- **Frames about equal** → reading problem. Check or swap the right encoder cable.
  Then swap encoder ports between sides to see if the low reading follows the
  motor (the encoder is bad) or stays with the port (CH port 2 is bad).
- **Right takes about 2x the frames** → the right wheel really is slow. Check the
  right cartridge ratio and motor type against the left, and look for gearbox
  binding. A motor swap may be needed (involve a mentor).

## Slow-motion result (2026-11-04)

Both wheels took about the same number of frames per turn. **Reading problem,
not a slow wheel.** The constant-percentage scale-down happens between the right
motor's encoder and CH port 2. The code treats both sides the same and the left
reads correctly, so it's hardware.

## Isolating the bad part (robot OFF before touching cables)

1. Reseat and inspect the right encoder cable at both ends. Retest.
2. Swap in a spare encoder cable. Fixed → the cable was bad.
3. Cross-swap encoder cables only: right motor's encoder → EH port 2, left's →
   CH port 2.
   - Low reading moves to "Actual L RPM" → the right motor's encoder is bad
     (replace the motor; involve a mentor).
   - Low reading stays on "Actual R RPM" → CH port 2 is bad. Move shooterRight
     to another CH port and update the config + `RobotHardware` comment.
   - **Put the cables back afterwards.**

## Related

- `docs/reassembly-checklist.md` step 11 (wheels spin inward) and step 15 (top
  speed never measured; VELOCITY mode can saturate and look broken). Do step 15
  after this is fixed.
