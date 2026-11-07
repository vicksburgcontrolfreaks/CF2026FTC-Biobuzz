# Branches: what each one is, and which to use

_Last updated 2026-11-05. Nothing listed here has been deleted. Every branch is
still on GitHub exactly as its owner left it._

## The short version

- **Deploy from `main`.** It has the drive code that was tested on both robots
  (5618 and 6494) on 2026-11-05.
- **Start new work from `main`:** `git checkout main`, `git pull`,
  `git checkout -b yourname/feature`.
- **Check the Build line at INIT.** DriveTeleOp, Select Auto, Drive
  Motor Direction Test and Pivot Jog Test all show:

      Build   main @ c904ce9  (built 11/05 19:42)
      Robot   6494

  If the branch is wrong, or it says `+UNCOMMITTED`, stop and redeploy.

## Main line

| Branch | What it is |
|---|---|
| `main` | **Deploy from here.** 2026 hardware, per-robot drive directions (`WhichRobot`), turret subsystem, collection system, pivot, indexer ball classification. |
| `shane/consolidate` | Everything else that's worth keeping, collected in one place: non-blocking Shooter, Select Auto + routes, turret aiming math, deflector servo tests, build info. Compiles and passes laptop tests. **Not on `main` yet**, and most of it isn't robot-tested. |
| `shane/drive-2026` | Same as `main` right now. Kept for history. |

## Already included in `shane/consolidate` (nothing lost if you ignore these)

Every commit on these branches is already in `shane/consolidate`, and most are
in `main` too. They're still here; you just don't need them to get their work.

| Branch | Owner | What it was |
|---|---|---|
| `elliott/hardware-config-2026` | Elliott | 2026 hardware config, the base everything is built on |
| `theo/turret-autoaim` | Theo | Turret AprilTag aiming |
| `connor/turret-autoaim`, `jade/turret-subsystem` | Connor, Jade | Turret subsystem, encoder homing, soft limits |
| `shane/collector-rpm`, `shane/indexer-load-sensing` | Shane | Collection system, pivot band, ball classification from indexer current, flywheel encoder diagnosis |
| `shane/shooter` | Shane | Non-blocking Shooter + laptop unit tests |
| `evan/auto-selector` | Evan | Select Auto: pick alliance, start, wait, park during INIT |
| `shane/auton-cleanup` | Shane | Moves unused autos to `legacy/`, tools to `test/` (includes `evan/auto-selector`) |
| `connor/red-startup` | Connor | Red Start Up auto + PathSafety checker |
| `shane/turret-aim` | Shane | Odometry turret aim, ShotTable, LaunchReach |
| `evan/turret-geometry` | Evan | Hive/tag measurements from the game manual |
| `shane/turret-geometry`, `shane/turret-geometry-solution` | Shane | Turret math scaffold + solutions |
| `connor/servo-test` | Connor | Servo Position Test + deflector Servo Feedback Test |
| `evan/field-constants` | Evan | BIOBUZZ field constants (on `main` since 9/29) |

## Not merged anywhere: earlier experiments (kept, not deleted)

Most of these were built for the 2025 robot or before the 2026 hardware config.
They're kept as-is. Ask the owner before building on one.

| Branch | Owner | Unmerged commits | What it is |
|---|---|---|---|
| `connor/apriltag-follow` | Connor | 10 | AprilTag Field Follow (built on the 2025 robot; it backed into walls, cause not found) |
| `connor/field-follow-24in` | Connor | 10 | Same commits as `connor/apriltag-follow` |
| `connor/no-cam-auton` | Connor | 3 | Camera-free copy of 2025's AutoLongShot (2025 field poses) |
| `elliott/shooter-velocity-test` | Elliott | 3 | Two-motor PIDF velocity test using FTC Dashboard. This repo uses Panels now; replaced by Shooter Flywheel Test |
| `elliott/distance-sensor` | Elliott | 1 | REV 2m distance sensor for game-piece detection |
| `elliott/touch-sensor` | Elliott | 1 | REV touch sensor as a limit switch |
| `emmett/led-test` | Emmett | 1 | Blinkin LED test cycling red/white/blue (the Blinkin is on 5618's config) |
| `shane/apriltag-notes-parked` | Shane | 2 | Oct 8 AprilTag notes + a Pedro visualizer how-to |
| `shane/startup-positions` | Shane | 1 | The Pedro visualizer how-to (also in `shane/apriltag-notes-parked`) |

## Duplicates: the same work is already on `main`

| Branch | Why |
|---|---|
| `connor/apriltag-test` | Its one commit (BIOBUZZ tag library) was redone on the main line |
| `connor/drive-motor-directions` | Its field-centric fix was redone on the main line |

## Waiting on the owner

| Branch | Owner | Unmerged commits | Question |
|---|---|---|---|
| `jade/shooter-indexer-tuning` | Jade | 13 | Should this come into `shane/consolidate`? |
| `jade/color-sensing-subsystem` | Jade | 3 | Still needed? Ball type is now read from indexer current, and neither robot has a color sensor configured |
