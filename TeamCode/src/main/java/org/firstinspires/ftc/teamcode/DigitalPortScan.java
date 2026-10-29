package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Finds out which digital port a sensor is actually wired to.
 *
 * WHY THIS EXISTS
 * A REV digital cable breaks out TWO pins, and a sensor wired into the 0-1 pair
 * often reports on port 1 rather than port 0. Read the wrong one and you get an
 * unconnected pin, which floats HIGH and reads "true" forever -- the exact
 * symptom of a sensor that looks dead but is fine.
 *
 * Rather than edit the config, redeploy, and guess again, this reads every
 * digital name it can find at once and tells you which one CHANGED when you
 * blocked the beam or pressed the switch.
 *
 * SETUP (one time, on the Driver Station)
 * This can only read ports that exist in the config -- hardwareMap cannot see a
 * port nothing is configured on. So add Digital Devices on the ports you want to
 * check, named from the list below, then come back and run this:
 *
 *   dig0 dig1 dig2 dig3   Expansion Hub digital 0-3
 *   chd0 chd1 chd2 chd3   Control Hub digital 0-3
 *
 * Any you skip are simply reported as "not in config" -- nothing crashes, so
 * add only the ones you care about. For the shooter pivot beam break, dig0 and
 * dig1 are the two that matter.
 *
 * Your existing "pivotHome" and "turretHome" are probed too, if present.
 *
 * HOW TO USE IT
 * 1. Start the op mode and leave the sensor ALONE. Every port shows its
 *    resting state.
 * 2. Block the beam (or press the switch) and hold it.
 * 3. Look for the port whose line says CHANGED. That is your sensor.
 * 4. Rename THAT port to "pivotHome" in the config, and delete the dig0-3
 *    scaffolding when you are finished.
 *
 * The CHANGED flag latches, so you can block the beam and then look at the
 * screen -- you do not have to watch and poke at the same time.
 *
 * IF NOTHING EVER CHANGES, the pin is not the problem. In that case:
 *   - Point a phone camera at the emitter. IR is invisible to your eyes but
 *     shows as a faint purple dot on a phone screen. No dot means no power
 *   - Reseat the cable at both the hub end and the sensor end
 *   - Try the sensor in a known-good digital port to rule out a dead sensor
 *
 * Read-only. This op mode never commands a motor or a servo.
 */
@TeleOp(name = "Digital Port Scan", group = "Test")
public class DigitalPortScan extends LinearOpMode {

    private static final String[] CANDIDATES = {
            "dig0", "dig1", "dig2", "dig3",
            "chd0", "chd1", "chd2", "chd3",
            "pivotHome", "turretHome",
    };

    @Override
    public void runOpMode() {
        int count = CANDIDATES.length;
        DigitalChannel[] channels = new DigitalChannel[count];
        boolean[] initial = new boolean[count];
        boolean[] changed = new boolean[count];
        int found = 0;

        for (int i = 0; i < count; i++) {
            // tryGet returns null instead of throwing, so missing names are fine.
            channels[i] = hardwareMap.tryGet(DigitalChannel.class, CANDIDATES[i]);
            if (channels[i] != null) {
                channels[i].setMode(DigitalChannel.Mode.INPUT);
                initial[i] = channels[i].getState();
                found++;
            }
        }

        telemetry.addLine("DIGITAL PORT SCAN");
        telemetry.addData("Ports found in config", found);
        if (found == 0) {
            telemetry.addLine("");
            telemetry.addLine("Nothing to read. Add Digital Devices named dig0/dig1/...");
            telemetry.addLine("on the Driver Station first -- see this op mode's notes.");
        } else {
            telemetry.addLine("");
            telemetry.addLine("Leave the sensor alone, press START, THEN block the beam.");
            telemetry.addLine("Watch for the port that says CHANGED.");
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Re-baseline with A, so a second sensor can be found without a restart.
            if (gamepad1.a) {
                for (int i = 0; i < count; i++) {
                    if (channels[i] != null) {
                        initial[i] = channels[i].getState();
                        changed[i] = false;
                    }
                }
            }

            telemetry.addLine("Block the beam / press the switch. A = re-baseline.");
            telemetry.addLine();

            for (int i = 0; i < count; i++) {
                if (channels[i] == null) {
                    telemetry.addData(CANDIDATES[i], "not in config");
                    continue;
                }
                boolean now = channels[i].getState();
                if (now != initial[i]) {
                    changed[i] = true;
                }
                telemetry.addData(CANDIDATES[i],
                        "now=" + now + "  rest=" + initial[i]
                                + (changed[i] ? "   <<< CHANGED - THIS IS YOUR SENSOR" : ""));
            }

            telemetry.update();
        }
    }
}
