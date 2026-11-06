package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.shooting.ShooterLogic;

/**
 * Shooter subsystem: two facing flywheels plus the indexer that feeds them.
 * NON-BLOCKING -- every method returns immediately. Call update() every loop.
 *
 *     shooter.spinUp(rpm);      // every loop if the distance changes
 *     shooter.fire(3);          // queue three pieces; they go when the wheels are ready
 *     shooter.update();         // every loop, or nothing happens
 *     if (!shooter.isBusy())    // auto: volley finished, drive on
 *
 * All the deciding happens in shooting/ShooterLogic (tested on a laptop). This
 * class only reads the motors, hands the numbers over, and writes the answer.
 *
 * THE INDEXER IS SHARED with CollectionSystem (the driver's unjam button uses it).
 * This class only writes the indexer while a piece is actually being fed, then
 * stops it once and lets go. The rest of the time, collection owns it.
 *
 * NOT HANDLED HERE (yet):
 *   - The pivot (pollen / nectar gap). Set it with ShooterPivot before firing.
 *   - Which RPM. Feed it ShotTable.lookup(distance) once that's on this branch.
 */
@Configurable
public class Shooter {

    // TODO(gear ratio): ShootAprilTag says 1:1, Elliott's ShooterVelocityTest says
    // a 3:1 cartridge. Count the cartridges on the motor. Until then this matches
    // ShooterVelocityTest so RPMs read the same as on the bench. WHATEVER IT IS,
    // don't change it after ShotTable data is collected -- the table's RPMs are
    // only meaningful in the units they were recorded in.
    public static double SHOOTER_GEAR_RATIO = 3.0;

    // Velocity PIDF for both wheels. NaN = leave the hub's firmware defaults.
    // Tune with ShooterVelocityTest, then copy the numbers here.
    public static double P = Double.NaN;
    public static double I = Double.NaN;
    public static double D = Double.NaN;
    public static double F = Double.NaN;

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final CollectionSystem collection;
    private final ShooterLogic logic = new ShooterLogic();
    private final ElapsedTime clock = new ElapsedTime();

    private boolean hadIndexer = false;
    private double leftRpm, rightRpm;

    /** Directions are set by RobotHardware; this class doesn't touch them. */
    public Shooter(DcMotorEx shooterLeft, DcMotorEx shooterRight, CollectionSystem collection) {
        this.left = shooterLeft;
        this.right = shooterRight;
        this.collection = collection;
        // setVelocity() needs the hub's velocity loop running. Safe here: neither
        // shooter motor's encoder port is shared (the turret's is on lf).
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Coast, don't brake: braking a 2000+ RPM flywheel is hard on the gearbox.
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // ---- Commands (see ShooterLogic for exactly what each does) -------------

    public void spinUp(double rpm) { logic.spinUp(rpm); }
    public void fire(int pieces)   { logic.fire(pieces); }
    public void stop()             { logic.stop(); }

    /** Run every loop. The only place the motors are written. */
    public void update() {
        if (!Double.isNaN(P) && !Double.isNaN(I) && !Double.isNaN(D) && !Double.isNaN(F)) {
            left.setVelocityPIDFCoefficients(P, I, D, F);
            right.setVelocityPIDFCoefficients(P, I, D, F);
        }

        leftRpm = toRpm(left.getVelocity());
        rightRpm = toRpm(right.getVelocity());
        logic.update(clock.seconds(), leftRpm, rightRpm);

        double target = logic.getTargetRpm();
        if (target <= 0) {
            left.setPower(0);    // coast down, not velocity 0 (that brakes)
            right.setPower(0);
        } else {
            left.setVelocity(toTicksPerSec(target));
            right.setVelocity(toTicksPerSec(target));
        }

        if (logic.ownsIndexer()) {
            collection.setIndexerPower(logic.getIndexerPower());
            hadIndexer = true;
        } else if (hadIndexer) {
            collection.stopIndexer();   // once, on letting go
            hadIndexer = false;
        }
    }

    private static double toRpm(double ticksPerSec) {
        return ticksPerSec / (RobotHardware.TICKS_PER_MOTOR_REV * SHOOTER_GEAR_RATIO) * 60.0;
    }

    private static double toTicksPerSec(double rpm) {
        return rpm * RobotHardware.TICKS_PER_MOTOR_REV * SHOOTER_GEAR_RATIO / 60.0;
    }

    // ---- Readouts -----------------------------------------------------------

    public boolean isReady()            { return logic.isReady(); }
    public boolean isBusy()             { return logic.isBusy(); }
    public boolean ownsIndexer()        { return logic.ownsIndexer(); }
    public ShooterLogic.State getState() { return logic.getState(); }
    public String getNote()             { return logic.getNote(); }
    public double getTargetRpm()        { return logic.getTargetRpm(); }
    public double getLeftRpm()          { return leftRpm; }
    public double getRightRpm()         { return rightRpm; }
    public int getShotsQueued()         { return logic.getShotsQueued(); }
    public int getShotsFired()          { return logic.getShotsFired(); }
    public double getLastDipRpm()       { return logic.getLastDipRpm(); }
}
