# Measuring the hive opening (so we can aim at the hive)

**Picked up here:** 2026-11-10, end of Shane's turret-aiming session.
**Status:** not measured. `Turret Odometry Aim Test` says
`CAN'T AIM: hive opening not measured yet` when you press Y for the hive target.

## What blocks it

Three numbers in `field/HiveGeometry.java`:

```java
public static double RED_AUDIENCE_OPENING_X = Double.NaN;
public static double RED_AUDIENCE_OPENING_Y = Double.NaN;
public static double RED_AUDIENCE_OPENING_Z = Double.NaN;   // height above the tiles
```

`isMeasured()` is false while any one is NaN, and that is the message.

**Measure ONE cell and get all four free.** `HiveGeometry.openingOf()` mirrors
across y = 72 for our other cell and spins 180 deg for blue.

## What the manual already gives us

From `docs/BIOBUZZ_Competition_Manual_V1.pdf` section 9.6 (text extracted
2026-11-10, so these are quotes, not guesses):

- pivot axis **43.95 in.** above the tiles -- already `FieldConstants.HIVE_PIVOT_HEIGHT`
- the two cells of a hive are **~18.8 in.** apart
- frame **49.46 in.** wide x **38.95 in.** deep, centred on the field
- cell opening **~20 in. wide x 14 in. tall x 12 in. deep**

So **Y is nearly free**: pivot at y = 72 (Fig. 9-8), cells +/-9.4 in., which puts
the audience cell near **y = 62.6**.

> Treat 62.6 as an estimate, not a measurement. 18.8 in. is the spacing along the
> hive's arm; when the hive is tipped, the HORIZONTAL distance shrinks by
> cos(tilt), so the real number is somewhat closer to 72.

**X and Z are not in the manual text.** They are only in Figures 9-8 through
9-11, which are raster images. Claude cannot render them (this machine has
`pdftotext` but not `pdftoppm`/`pdfimages`).

### Try this first: open the PDF yourself

Pages **70-72**, Figures 9-8 / 9-9 / 9-10 / 9-11. Look for:

- the **x spacing between the red hive and the blue hive** inside the frame
- the **height of the cell opening** above the tiles

That may be the whole job, with no tape measure needed.

## If the figures do not have it: measure on a real field

Tip the hive so the **red audience cell faces up**, then:

1. Hang a plumb line (weighted string) from the **centre of the opening** and
   mark where it touches the tiles.
2. Tape from the **red alliance wall** to that mark -> **X**
3. Tape from the **audience wall** to that mark -> **Y**
4. Tape from the tiles straight up to the centre of the opening -> **Z**

Four pulls, no trig. The plumb line is the part that keeps us honest: the
opening is tilted and high up, so eyeballing its floor position is exactly
where the error creeps in.

## Two things worth knowing before bothering

**Z does nothing for turret aiming.** `TurretGeometry.aimAngle()` and
`distanceTo()` are flat x/y only. Z feeds `LaunchReach` (can we physically reach
the hive, and eventually the ballistics). It blocks hive aiming here only
because `isMeasured()` insists on all three -- over-strict for this op mode.

**For proving the turret geometry, the tape mark is the better target.** We are
trying to confirm `TURRET_OFFSET_X/Y = -4.5, 0`. Aiming at the hive tests two
unknowns at once: if the aim sits off the opening, we cannot tell whether the
turret offset is wrong or the hive numbers are. The tape mark's position we
KNOW, because we put it there. So the `CAN'T AIM` gate is protecting the
experiment -- prove the geometry on the mark first, then switch to the hive and
any error left over is the hive numbers.

## Bonus: one measurement, two payoffs

`RED_AUDIENCE_OPENING_X` should come out equal to `HiveTags.RED_HIVE_X`, which
is a **MADE-UP 60.0** today and feeds the AprilTag localization library
(`FieldAprilTags.getLocalizationLibrary()`). Measuring the hive once fixes both.

## Open offer (Shane had not answered yet)

Make `HiveGeometry` `@Configurable` and `refreshClass` it in
`TurretOdometryAimTest`, so estimates can be typed into Panels on the field and
the aim watched live, instead of a redeploy per guess.

Small catch: `HiveGeometry` is a `public final class`, and nothing `final` in
this repo is `@Configurable` yet, so the `final` would come off and we would
confirm Panels actually picks the fields up.
