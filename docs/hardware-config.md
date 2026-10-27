# Robot hardware configuration — 2026 BIOBUZZ

The authoritative list of what is plugged in where, and what name the code
expects for it. Last confirmed against the robot **2026-10-27**.

**Config names are exact string matches.** `Color Sensor` and `colorSensor` are
two different devices as far as the SDK is concerned, and the wrong one is a
crash on `init()`, not a warning. Copy the names out of the Name column
character for character.

---

## Control Hub

### Motors
| Port | Name | Notes |
|---|---|---|
| 0 | `lr` | left rear drive |
| 1 | `lf` | left front drive — **also carries the turret encoder** (see below) |
| 2 | `shooterRight` | flywheel; reversed in code, not in the config |
| 3 | `collector` | collection intake, 20:1 |

### Servos
| Port | Name | Type | Notes |
|---|---|---|---|
| 0 | `shooterPivot` | Servo | sets the gap between the two flywheels |
| 1 | `blinkin` | Servo | REV Blinkin LED driver |
| 2 | `turret` | **Continuous Rotation** Servo | CR, not standard — see `docs/turret-bringup.md` |

### Digital
| Port | Name | Notes |
|---|---|---|
| 0 or 1 | `turretHome` | REV magnetic limit switch. Reports on either pin |

### I2C
| Bus | Name | Notes |
|---|---|---|
| 0 | `pinpoint` | goBILDA Pinpoint odometry. **Not** `odometry` — `Constants.java` expects `pinpoint` |

---

## Expansion Hub

### Motors
| Port | Name | Notes |
|---|---|---|
| 0 | `rr` | right rear drive |
| 1 | `rf` | right front drive |
| 2 | `shooterLeft` | flywheel |
| 3 | `indexer` | feeds pieces to the shooter, 20:1 |

### Servos
_None._

### Digital
| Port | Name | Notes |
|---|---|---|
| 0 | `pivotHome` | IR beam break the shooter pivot calibrates against |

### I2C
| Bus | Name | Notes |
|---|---|---|
| 0 | `imu` | the hub's own IMU. Nothing in the code reads it today |
| 1 | `colorSensor` | reads the piece at the indexer wheel |

---

## USB

| Name | Notes |
|---|---|
| `Webcam 1` | AprilTag camera, **turret-mounted** — it swings with the turret |

---

## Things that bite

**There is no `turretEncoder` device, and there cannot be.** The turret's REV
Through Bore encoder plugs into `lf`'s spare encoder connector, so the SDK reads
it through the `lf` *motor*. Nothing to add to the config. Two rules keep sharing
that port safe:

1. `lf` stays in `RUN_WITHOUT_ENCODER` (the default). In `RUN_USING_ENCODER` or
   `RUN_TO_POSITION` the hub would regulate a drive wheel from the *turret's*
   position.
2. Nobody calls `STOP_AND_RESET_ENCODER` on `lf`, which would zero the turret
   mid-match.

Moved from `rf` to `lf` on 2026-10-27 so encoder reads stop crossing RS485 to the
Expansion Hub. Turret direction and homing need re-verifying after that move.

**The two shooter motors are on different hubs** (`shooterLeft` on EH2,
`shooterRight` on CH2). Each hub runs its own velocity PIDF locally so each wheel
regulates itself fine, and the split spreads current draw. But if the two ever
report speeds that disagree, cross-hub command skew is the first suspect.

**REV digital sensors can land on the odd pin.** A switch wired into digital pair
0-1 sometimes configures as port 1 rather than 0. `turretHome` reports on either,
so it does not matter there — but check it if a digital device reads nothing.

**Servo direction and motor direction live in code, not in the config.**
`shooterRight`, `collector`, `lf` and `rr` are all reversed in
`RobotHardware.init()`. Do not also reverse them in the config or they cancel out.

---

## Retired 2026-10-27 — do not add these back

| Old name | Why it is gone |
|---|---|
| `sensor1` `sensor2` `sensor3` | the three magazine beam breaks, off the robot. `isMagazineFull()` went with them and nothing replaced it |
| `flipper` | launch servo, replaced by the `indexer` motor |
| `shooter` | single shooter motor, now `shooterLeft` + `shooterRight` |
| `limitSwitch` | never tied to a mechanism |
| `indexer` as a **Servo** | that servo is now `shooterPivot`. `indexer` is a **motor**. Op modes asking for a Servo named `indexer` are stale |

---

## Copying the config to the robot instead of thumb-typing it

The Driver Station stores each robot configuration as an XML file on the **Robot
Controller**, under `/sdcard/FIRST/settings/`. So this is a solved problem — but
it has to be built **once** on the Driver Station first.

`TeamCode/src/main/res/xml/hardware2025.xml` is **not** a usable config. Its
format (`<RobotConfiguration>`, `MotorType.REV_HD_20_1`) is not what the FTC SDK
reads, and its device names (`exampleServo`, `leftOdo`) match nothing in the code.
It was never loadable, which is probably why last year still got typed in by hand.
Do not use it as a template.

The path that actually works:

1. **Build the config once** on the Driver Station, from the tables above.
2. **Connect to the Robot Controller** — USB-C to the Control Hub, or over the
   robot's wifi:
   ```
   adb connect 192.168.43.1:5555
   ```
3. **Confirm the directory and find the file name** (it is named whatever you
   called the config):
   ```
   adb shell ls /sdcard/FIRST/settings/
   ```
4. **Pull it into the repo** so it is version controlled:
   ```
   adb pull /sdcard/FIRST/settings/<YourConfigName>.xml docs/robot-config/
   ```
5. **Push it to any robot** from then on — this is the step that replaces all the
   thumb-typing:
   ```
   adb push docs/robot-config/<YourConfigName>.xml /sdcard/FIRST/settings/
   ```
   Then on the Driver Station: Configure Robot → the config appears in the list →
   Activate.

Once step 4 is done, a rebuilt Control Hub is one `adb push` away from a working
config, and any change to the wiring shows up as a reviewable diff instead of a
rumour.

`adb` lives at
`C:\Users\<you>\AppData\Local\Android\Sdk\platform-tools\adb.exe`.
