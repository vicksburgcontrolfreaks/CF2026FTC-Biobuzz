package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

/**
 * What is actually in the active hardware config, versus what the code expects.
 *
 * WHY THIS EXISTS
 * "Unable to find a hardware device with name 'rr'" tells you about ONE name --
 * whichever the failing op mode happened to ask for first -- and nothing about
 * the rest. On 2026-11-12 that error came from the Expansion Hub, and because
 * `rr` is EH port 0 and the first device RobotHardware.init() asks for, it hid
 * the fact that rf, shooterLeft, indexer, shooterPivot, pivotHome and
 * colorSensor were all equally unreachable.
 *
 * Same week, two more config mysteries cost real bench time: the shooter pivot
 * servo was programmed continuous-rotation while the config said positional, and
 * nobody could say whether `pivotPos` was still configured. All three are one
 * screen of output.
 *
 * IT TOUCHES NO HARDWARE AND BUILDS NO SUBSYSTEM. That is the whole point: it
 * has to run when RobotHardware.init() cannot. It never constructs
 * RobotHardware, never calls Constants.createFollower, never opens the camera,
 * and writes to nothing. Only safe to add to this op mode are more LOOKUPS.
 *
 * WHAT IT CHECKS
 *   1. HUBS. How many Lynx modules the app can see. TWO is normal. ONE means a
 *      whole hub is not answering -- RS-485 cable between the hubs, hub power,
 *      or the hub address -- and every device on it will read as missing no
 *      matter what the config says. Check this line FIRST; if a hub is gone,
 *      the missing-device list below is a symptom, not the cause.
 *   2. MISSING. Expected names the config does not have, required ones first.
 *   3. WRONG TYPE. Name is there but configured as something the code cannot
 *      fetch. The repo has been bitten twice: `blinkin` must be type Blinkin
 *      (not Servo) and `turret` must be Continuous Rotation (not Servo).
 *   4. EXTRA. Configured devices the code never asks for. Usually harmless;
 *      sometimes a rename that left the old entry behind.
 *
 * WHAT IT CANNOT TELL YOU
 * Which hub or port a device is on -- the SDK does not expose that here, so the
 * hub/port notes below come from docs/hardware-config.md and are only as current
 * as that file. It also cannot see INSIDE a servo: an Axon programmed
 * continuous-rotation while the config says positional looks perfectly fine
 * here. See the axon note in Pivot Jog Test for how that one presents.
 *
 * Press A to list every configured device with its type.
 */
@TeleOp(name = "Config Scan", group = "Test")
public class ConfigScan extends OpMode {

    /** One name the code looks up, and what happens if it is not there. */
    private static final class Expect {
        final String name;
        final Class<? extends HardwareDevice> type;
        final boolean required;   // true = a hard get(); INIT dies without it
        final String note;

        Expect(String name, Class<? extends HardwareDevice> type, boolean required, String note) {
            this.name = name;
            this.type = type;
            this.required = required;
            this.note = note;
        }
    }

    /**
     * Every name the code asks for, from RobotHardware, Turret, Deflector,
     * pedroPathing/Constants and the bring-up op modes. Hub and port notes are
     * from docs/hardware-config.md -- if the robot gets rewired, fix them there
     * and here together.
     *
     * `required` means the code does a hard hardwareMap.get(): absent, it throws
     * at INIT. The rest are tryGet() and degrade quietly, so a missing one is
     * only a problem for whatever reads it.
     */
    private static final Expect[] EXPECTED = {
            // ---- Drive. All four are hard gets in RobotHardware AND named again
            // in Constants for Pedro, so a missing one breaks everything.
            new Expect("rr", DcMotorEx.class, true, "EH0  right rear drive"),
            new Expect("rf", DcMotorEx.class, true, "EH1  right front drive"),
            new Expect("lr", DcMotorEx.class, true, "CH0  left rear drive"),
            new Expect("lf", DcMotorEx.class, true, "CH1  left front drive + TURRET ENCODER"),

            // ---- Odometry. Pedro requires this exact name, not "odometry".
            new Expect("pinpoint", HardwareDevice.class, true, "CH I2C0  goBILDA Pinpoint"),

            // ---- Collection
            new Expect("collector", DcMotorEx.class, true, "CH3  intake, 20:1"),
            new Expect("indexer", DcMotorEx.class, true, "EH3  feeds the shooter, 20:1"),
            new Expect("colorSensor", NormalizedColorSensor.class, false,
                    "EH I2C1  optional, nothing reads it yet"),

            // ---- Shooter
            new Expect("shooterLeft", DcMotorEx.class, true, "EH2  flywheel"),
            new Expect("shooterRight", DcMotorEx.class, true, "CH2  flywheel"),
            new Expect("shooterPivot", Servo.class, true, "EH servo 0  flywheel gap"),
            new Expect("pivotHome", DigitalChannel.class, false,
                    "EH digital 0  optional, not fitted as of 11-05"),
            new Expect("pivotPos", AnalogInput.class, false,
                    "EH analog 3  shooter pivot feedback. NOTHING READS IT YET"),

            // ---- Turret. Hard gets in Turret.init().
            new Expect("turret", CRServo.class, true,
                    "CH servo 0  MUST be type Continuous Rotation, not Servo"),
            new Expect("turretHome", TouchSensor.class, true,
                    "CH digital  magnetic limit switch"),

            // ---- Deflector. Both tryGet; either missing -> isPresent() false
            // and the deflector is inert rather than broken.
            new Expect("deflector", CRServo.class, false,
                    "CH servo  CR ON PURPOSE. Removed from the robot 2026-11-12"),
            new Expect("deflectorPos", AnalogInput.class, false,
                    "CH analog 1  deflector feedback (5618 may differ)"),

            // ---- Other
            new Expect("blinkin", RevBlinkinLedDriver.class, false,
                    "CH servo 1  MUST be type Blinkin, not Servo"),
            new Expect("Webcam 1", WebcamName.class, true,
                    "USB  the turret camera. Note the SPACE in the name"),
    };

    private boolean showAll = false;

    @Override
    public void init() {
        // Deliberately empty. Everything happens in init_loop so the answers are
        // on the screen at INIT, without anyone pressing play on a robot whose
        // config is already known to be broken.
    }

    @Override
    public void init_loop() {
        scan();
    }

    @Override
    public void loop() {
        scan();
    }

    private void scan() {
        if (gamepad1.aWasPressed()) showAll = !showAll;

        telemetry.addLine("CONFIG SCAN -- looks things up, touches nothing");
        BuildInfo.show(telemetry);

        // The config NAME, and whether WhichRobot can read a robot out of it.
        // Guarded: WhichRobot throws when the name doesn't start with a number,
        // and this op mode has to survive that -- diagnosing it is its job.
        String configName;
        try {
            configName = WhichRobot.configName();
        } catch (RuntimeException e) {
            configName = "<could not read: " + e.getMessage() + ">";
        }
        telemetry.addData("Active config", "\"%s\"", configName);
        try {
            telemetry.addData("WhichRobot says", WhichRobot.number());
        } catch (RuntimeException e) {
            telemetry.addLine(">>> WhichRobot CANNOT TELL which robot this is.");
            telemetry.addLine("    Rename the config so it starts with 5618 or 6494,");
            telemetry.addLine("    or every per-robot number will be the wrong one.");
        }

        // ---- 1. HUBS. Read this before anything below. ----------------------
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        telemetry.addLine();
        telemetry.addData("HUBS SEEN", "%d   %s", hubs.size(),
                hubs.size() >= 2 ? "(normal)" : ">>> A HUB IS MISSING <<<");
        if (hubs.size() < 2) {
            telemetry.addLine("    Only one Lynx module answered. Everything on the other");
            telemetry.addLine("    hub will read as missing whatever the config says.");
            telemetry.addLine("    Check: RS-485 cable between the hubs, hub power, hub address.");
            telemetry.addLine("    A deploy restarts the app and re-scans the hubs, which is");
            telemetry.addLine("    why this can appear right after a deployment.");
        }
        for (LynxModule hub : hubs) {
            telemetry.addLine("    " + namesOf(hub) + (hub.isParent() ? "  (parent)" : ""));
        }

        // ---- 2 and 3. Missing, and present-but-wrong-type --------------------
        List<String> missingRequired = new ArrayList<>();
        List<String> missingOptional = new ArrayList<>();
        List<String> wrongType = new ArrayList<>();

        for (Expect e : EXPECTED) {
            boolean anyDevice = hardwareMap.tryGet(HardwareDevice.class, e.name) != null;
            boolean asRightType = e.type == HardwareDevice.class
                    ? anyDevice
                    : hardwareMap.tryGet(e.type, e.name) != null;

            if (!anyDevice) {
                (e.required ? missingRequired : missingOptional)
                        .add(String.format("%-14s %s", e.name, e.note));
            } else if (!asRightType) {
                wrongType.add(String.format("%-14s wanted %s -- %s",
                        e.name, e.type.getSimpleName(), e.note));
            }
        }

        telemetry.addLine();
        if (missingRequired.isEmpty()) {
            telemetry.addLine("MISSING (required): none -- INIT should get past the hardware.");
        } else {
            telemetry.addLine(">>> MISSING, REQUIRED -- these STOP op modes from INIT-ing:");
            for (String s : missingRequired) telemetry.addLine("    " + s);
        }

        if (!wrongType.isEmpty()) {
            telemetry.addLine();
            telemetry.addLine(">>> WRONG TYPE -- the name is there, the code can't fetch it:");
            for (String s : wrongType) telemetry.addLine("    " + s);
        }

        if (!missingOptional.isEmpty()) {
            telemetry.addLine();
            telemetry.addLine("Missing, optional (these degrade quietly, no crash):");
            for (String s : missingOptional) telemetry.addLine("    " + s);
        }

        // ---- 4. Everything configured, and anything unexpected --------------
        Set<String> present = allConfiguredNames();
        Set<String> expectedNames = new TreeSet<>();
        for (Expect e : EXPECTED) expectedNames.add(e.name);
        List<String> extra = new ArrayList<>();
        for (String name : present) {
            if (!expectedNames.contains(name)) extra.add(name);
        }
        if (!extra.isEmpty()) {
            telemetry.addLine();
            telemetry.addLine("Configured but the code never asks for it:");
            telemetry.addLine("    " + join(extra));
            telemetry.addLine("    (often a rename that left the old entry behind)");
        }

        telemetry.addLine();
        telemetry.addData("Devices configured", "%d   (A %s the full list)",
                present.size(), showAll ? "hides" : "shows");
        if (showAll) {
            for (HardwareDevice d : hardwareMap.getAll(HardwareDevice.class)) {
                telemetry.addLine(String.format("    %-16s %s",
                        namesOf(d), d.getClass().getSimpleName()));
            }
        }
        telemetry.update();
    }

    /** Every name in the config, sorted, with duplicates from aliases collapsed. */
    private Set<String> allConfiguredNames() {
        Set<String> names = new TreeSet<>();
        for (HardwareDevice d : hardwareMap.getAll(HardwareDevice.class)) {
            names.addAll(hardwareMap.getNamesOf(d));
        }
        return names;
    }

    /** A device can carry more than one configured name. Show them all. */
    private String namesOf(HardwareDevice device) {
        Set<String> names = new TreeSet<>(hardwareMap.getNamesOf(device));
        return names.isEmpty() ? "<unnamed>" : join(new ArrayList<>(names));
    }

    private static String join(List<String> items) {
        Collections.sort(items);
        StringBuilder out = new StringBuilder();
        for (String s : items) {
            if (out.length() > 0) out.append(", ");
            out.append(s);
        }
        return out.toString();
    }
}
