package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Collection system: the collector motor that pulls game pieces into the robot,
 * the indexer motor that feeds them up to the shooter, and the color sensor that
 * reads what is sitting at the indexer wheel.
 *
 * Driver Station config names this expects:
 *   "collector"   - DcMotorEx, 20:1
 *   "indexer"     - DcMotorEx, 20:1
 *   "colorSensor" - NormalizedColorSensor, aimed at the indexer wheel
 *
 * Named CollectionSystem, not Collection, because java.util.Collection already
 * owns that name and the clash produces genuinely baffling import errors.
 *
 * WHAT THIS CLASS DOES NOT DO
 * It does not classify game pieces. Jade's ColorSensing already converts this
 * sensor into POLLEN_YELLOW / NECTAR_RED / NECTAR_BLUE, with hue ranges and
 * distance cutoffs measured on the real robot. Copying that logic here would
 * guarantee the two drift apart, so this class just holds the sensor and hands
 * it over:
 *
 *     ColorSensing pieces = new ColorSensing(collection.getColorSensor());
 *
 * That line does not compile until jade/color-sensing-subsystem merges, which
 * is the whole point -- one classifier, on her branch, not two.
 *
 * It also does not decide when to stop collecting. The three magazine beam
 * breaks were removed 2026-10-27 and nothing replaced them, so there is no
 * "magazine full" signal on the robot any more. The driver decides.
 */
@Configurable
public class CollectionSystem {

    // Collector speeds, closed loop, in collector RPM (after the 20:1; ~300 max).
    // Tunable live in Panels. 200 is Collection System Test's starting speed; the
    // old 0.5 power (~150 RPM unloaded, less under load) felt slow on 6494, 2026-11-05.
    public static double COLLECT_RPM = 200.0;
    public static double EJECT_RPM = 200.0;   // run in reverse by eject()

    // Indexer speeds, closed loop, in indexer RPM (after the 20:1). 50 RPM is
    // what Collection System Test, Load Recovery and the ball classifier use;
    // full power (~300 RPM) was far too fast (seen on 6494, 2026-11-05).
    // 25 on 6494, 2026-11-06: at 75 the second ball followed the first with no
    // visible pause. If you change it, scale ShooterLogic.NO_BALL_SEC to match.
    public static double FEED_RPM = 25.0;   // forward = up into the flywheels
    public static double UNJAM_RPM = 50.0;  // run in reverse by unjam()

    private static final double TICKS_PER_REV = RobotHardware.TICKS_PER_MOTOR_REV;
    private static final double COLLECTOR_RATIO = RobotHardware.COLLECTOR_GEAR_RATIO;
    private static final double INDEXER_RATIO = RobotHardware.INDEXER_GEAR_RATIO;

    private final DcMotorEx collector;
    private final DcMotorEx indexer;
    private final NormalizedColorSensor colorSensor;

    public CollectionSystem(DcMotorEx collector, DcMotorEx indexer,
                            NormalizedColorSensor colorSensor) {
        this.collector = collector;
        this.indexer = indexer;
        this.colorSensor = colorSensor;
    }

    // ---- Collector ----------------------------------------------------------

    /** Pulls pieces in at COLLECT_RPM. */
    public void collect() {
        setCollectorRPM(COLLECT_RPM);
    }

    /** Spits a piece back out -- the wrong-alliance nectar escape hatch. */
    public void eject() {
        setCollectorRPM(-EJECT_RPM);
    }

    public void stopCollector() {
        collector.setPower(0.0);
    }

    public void setCollectorPower(double power) {
        collector.setPower(power);
    }

    /** Closed-loop collector speed, for when a repeatable intake rate matters. */
    public void setCollectorRPM(double rpm) {
        collector.setVelocity(rpmToTicksPerSec(rpm, COLLECTOR_RATIO));
    }

    public double getCollectorRPM() {
        return ticksPerSecToRPM(collector.getVelocity(), COLLECTOR_RATIO);
    }

    // ---- Indexer ------------------------------------------------------------

    /** Feeds a piece up into the flywheels at FEED_RPM. */
    public void feed() {
        setIndexerRPM(FEED_RPM);
    }

    /** Backs the indexer up (AWAY from the shooter) at UNJAM_RPM to clear a stuck piece. */
    public void unjam() {
        setIndexerRPM(-UNJAM_RPM);
    }

    public void stopIndexer() {
        indexer.setPower(0.0);
    }

    public void setIndexerPower(double power) {
        indexer.setPower(power);
    }

    public void setIndexerRPM(double rpm) {
        indexer.setVelocity(rpmToTicksPerSec(rpm, INDEXER_RATIO));
    }

    public double getIndexerRPM() {
        return ticksPerSecToRPM(indexer.getVelocity(), INDEXER_RATIO);
    }

    /**
     * Indexer current draw. A jammed motor pulls far more current than a loaded
     * one, so this is the cheapest jam signal available now that the beam breaks
     * are gone -- watch it on the dashboard with a piece deliberately stuck to
     * find the threshold before writing any auto-unjam logic against it.
     */
    public double getIndexerCurrentAmps() {
        return indexer.getCurrent(CurrentUnit.AMPS);
    }

    // ---- Color sensor -------------------------------------------------------

    /**
     * Wrap this in Jade's ColorSensing rather than reading hue here.
     *
     * MAY BE NULL. The color sensor is not fitted to the robot yet, so a caller
     * that passed null to the constructor gets null back. Check before wrapping.
     */
    public NormalizedColorSensor getColorSensor() {
        return colorSensor;
    }

    // ---- Everything ---------------------------------------------------------

    public void stopAll() {
        stopCollector();
        stopIndexer();
    }

    /**
     * Lets both motors coast instead of braking. Worth knowing the difference:
     * BRAKE shorts the motor so it resists turning, which is what you want on a
     * drivetrain but will crush a game piece caught in the indexer.
     */
    public void setFloatOnZeroPower() {
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private double rpmToTicksPerSec(double rpm, double gearRatio) {
        return rpm * TICKS_PER_REV * gearRatio / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec, double gearRatio) {
        return (ticksPerSec / (TICKS_PER_REV * gearRatio)) * 60.0;
    }
}
