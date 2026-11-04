# Flywheel RPM gap: left reads ~400, right ~1200

**Status:** open. Diagnosis planned, not yet run (robot unavailable 2026-11-03).

## What we saw (2026-11-02, after the rebuild)

In `Shooter Flywheel Test`, `Actual R RPM` was about **1200** and `Actual L RPM`
about **400**. That's 3x apart. We didn't write down whether it was in POWER or
VELOCITY mode. Note that next time.

## Why it isn't the hubs, or a gear ratio

- Each hub (EH2 left, CH2 right) runs its own velocity PIDF, which can leave a
  *small* gap. A 3x gap is too big to come from that.
- Both sides use the same `RobotHardware.SHOOTER_GEAR_RATIO` (3.0). The encoder
  is on the **motor**, before the gearbox. So a mismatched cartridge would make
  the left *wheel* slow while both *readings* matched. It can't make the left
  reading 3x low. (Check the cartridges anyway; it takes ten seconds.)

So it's one of these two:

- **A. The left motor really is slower.** Something is rubbing or jammed.
  The pivot was just rebuilt, and it sets the gap between the flywheels, so
  look there first.
- **B. The left reading is wrong.** The left encoder cable (EH motor port 2)
  is loose, damaged, or in the wrong port, or the encoder itself is bad.

## Test plan (Shooter Flywheel Test, POWER mode, no code changes)

1. **Motors off (don't press A): spin each wheel by hand**, about the same speed.
   - Left feels stiffer or stops sooner → **A**.
   - Left RPM reads 0, jumps around, or reads much lower than right → **B**.
2. **Motors on: press A, hold the trigger about halfway.** Do both wheels look and
   sound the same speed? Left visibly slower → **A**.
3. **Compare `Amps L` and `Amps R`** at that same trigger position.
   - Left amps much higher → **A**: the motor is fighting something.
   - Amps about equal → **B**: the motor is fine and the reading is wrong.

Record: mode, trigger position, L/R RPM, L/R amps, and how the hand-spin felt.

Keep fingers clear whenever the shooter is ON. Do the hand-spin only with it off.

## Related

- `docs/reassembly-checklist.md` step 11 (wheels spin inward) and step 15
  (top speed has never been measured; VELOCITY mode can saturate and *look*
  broken). Step 15 can't explain a left/right gap, but do it once this is fixed.
