# Pollen shot tuning — 6494, 2026-11-09

Found with **Pollen Shot Tune** (Jade). Recorded before the numbers went into
code, because a session ran out of time and these were only in someone's head.

**Not yet applied to any constant.** See "Putting these in" below.

## What was found

| Knob | Started at | Goldilocks zone | Maps to |
|---|---|---|---|
| Pivot (flywheel gap) | 0.67 | **0.69** | `ShooterPivot.POLLEN_POSITION` |
| Deflector angle | 67.5° | **69.5°** — see caveat | `DeflectorLogic.POSITIONS` |
| Indexer feed | 25 RPM | **75 RPM** | `CollectionSystem.FEED_RPM` (6494's case) |
| Flywheel RPM | — | **not tested** | `Shooter` / `ShotTable` |

Every "started at" matches what is in the code for 6494 today, so these are real
measured changes and not a misremembered baseline:

- `ShooterPivot.POLLEN_POSITION = 0.67` (6494, measured 2026-10-30)
- `67.5` is the second entry in `DeflectorLogic.POSITIONS`
- `CollectionSystem.FEED_RPM = 25.0` for 6494, and 50.0 for 5618

## Flywheel RPM was NOT tested

Left wherever Pollen Shot Tune starts it (1300 RPM). So this set is "the pivot,
deflector and indexer that worked at whatever flywheel speed was already dialled
in" -- not a complete shot recipe. Finding the flywheel RPM is the next job, and
it is the one `ShotTable` needs.

Worth checking first, because it decides whether flywheel numbers can be trusted
at all: `docs/flywheel-rpm-gap.md` says **`shooterRight`'s encoder is bad** and
reads about half the real speed, "replace the motor, then retest (2026-11-04)".
No commit since then says it was replaced, and the doc never says which robot.
If that is 6494, every flywheel RPM read there is wrong.

## Putting these in

Each one needs a decision, which is why they are written down rather than pasted:

**Pivot 0.69** goes in `ShooterPivot`'s 6494 block -- not the bare
`POLLEN_POSITION` field, which the constructor overwrites per robot. 0.69 is
inside 6494's safe band.

**Deflector 69.5** is NOT one of the presets (`0, 67.5, 70, 72.5, 75, 80, 90`),
and it sits between 67.5 and 70. So either add 69.5 as a row, or move the 67.5
row -- and `POSITIONS` is shared by both robots, so changing it changes 5618 too.
A per-robot deflector table may be what this actually wants.

**Indexer 75 RPM** goes in `CollectionSystem`'s 6494 case, beside 5618's 50.
Three times the old 25, so worth a second confirmation run before trusting it in
a match: a feed that fast changes how the flywheels dip per shot, and Pollen
Shot Tune's shot log measures exactly that.

## The deflector number needs re-measuring after auto-homing (Jade, 2026-11-09)

**69.5° is measured against whatever zero was set that day**, by hand. The
physical angle that worked is real; the *number* is only as good as its
reference. Auto-homing will establish a different, repeatable zero, and the same
physical angle will then read as a different number.

So do not paste 69.5 into anything. Treat it as "about 2 degrees above the 67.5
preset" -- it says where to look, not what to set.

**Auto-homing is already written** (commit 47911bd, on this branch): every auto
homes the deflector in INIT -- down to the hard stop, up to level, power off --
and TeleOp homes at Start if it is not already zeroed. It stall-detects the stop
and sets zero from a per-robot constant.

**But it is inert**, because that constant is not measured:

    Deflector.LEVEL_ABOVE_STOP_DEG = NaN   // both robots

While it is NaN, homing refuses and nothing changes. So the order is:

1. Measure `LEVEL_ABOVE_STOP_DEG` for 6494 -- how many degrees above the hard
   stop is level. **Deflector Zero** shows the stop reading for exactly this.
2. Auto-homing wakes up on its own, no other change needed.
3. **Then** re-find the pollen deflector angle in Pollen Shot Tune. That number
   is worth writing into code, because the zero behind it is now repeatable
   across power cycles.

The pivot (0.69) and indexer (75 RPM) figures are unaffected -- neither depends
on the deflector's zero.
