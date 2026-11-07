package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.WhichRobot;
import org.firstinspires.ftc.teamcode.shooting.NectarGuard;
import org.firstinspires.ftc.teamcode.shooting.NectarGuard.Gap;
import org.firstinspires.ftc.teamcode.shooting.ShooterLogic;

/**
 * Launcher: everything a ball touches on its way out -- collector, indexer,
 * flywheels, pivot -- run as ONE thing, with NectarGuard in the middle.
 *
 * WHY ONE CLASS: the nectar guard only protects anything if EVERY forward run of
 * the indexer goes past it -- the fire button, the "everything forward" trigger,
 * the Shooter's own feed, and later auto. Spread those across op modes and one
 * of them will eventually skip it. So nothing outside this class writes the
 * collector, indexer, flywheels or pivot.
 *
 *     launcher.start();                 // in start(): pivot to the pollen gap
 *     launcher.toggleFlywheel();        // Y
 *     controls.fire = gamepad2.right_bumper; ...
 *     launcher.update(controls);        // every loop
 *
 * The forward indexer always runs at CollectionSystem.FEED_RPM -- the one speed
 * the guard is calibrated for (current scales with speed).
 *
 * Pivot: only robot 6494 has measured pollen/nectar positions. On any other
 * robot the pivot is never written and isPivotBlocked() says so -- the guard
 * still watches, but it can't move anything.
 */
public class Launcher {

    /** Fixed for now; ShotTable (distance -> RPM) comes with odometry aiming. */
    public static double FLYWHEEL_RPM = 1300;

    /** What the operator is holding this loop. */
    public static class Controls {
        /** Fire while held: starts the flywheel; one ball at a time, only when the wheels are at speed. */
        public boolean fire;
        /** Indexer backward. */
        public boolean indexerBack;
        public boolean collectIn;
        public boolean collectOut;
        /** Collector + indexer forward: starts the flywheel and fires like {@link #fire}, collector helping. */
        public boolean allForward;
        /** Collector + indexer backward. */
        public boolean allBack;
    }

    private enum Motion { STOP, FORWARD, BACK }

    private final CollectionSystem collection;
    private final Shooter shooter;
    private final ShooterPivot pivot;   // null = not measured on this robot
    private final VoltageSensor battery;
    private final NectarGuard guard = new NectarGuard();
    private final ElapsedTime clock = new ElapsedTime();

    private boolean flywheelOn = false;
    private Gap appliedGap = null;
    private Motion indexerCmd = null;    // null = unknown, forces the next write
    private Motion collectorCmd = null;
    private boolean shooterHadIndexer = false;
    private int lastShotsFired = 0;
    private boolean indexerForward = false;
    private double indexerAmps = 0;

    public Launcher(RobotHardware hw, HardwareMap hardwareMap) {
        collection = new CollectionSystem(hw.collector, hw.indexer, hw.colorSensor);
        shooter = new Shooter(hw.shooterLeft, hw.shooterRight, collection);
        // Same rule as Shooter Test: 5618's pivot positions aren't measured, and a
        // guessed position can strain the servo or crush a ball.
        pivot = WhichRobot.get() == WhichRobot.Robot.R6494 ? new ShooterPivot(hw.shooterPivot) : null;
        battery = hardwareMap.voltageSensor.iterator().next();
    }

    /** Call from start() (servos shouldn't move during INIT). Puts the pivot at the guard's gap. */
    public void start() {
        appliedGap = null;
        applyGap(guard.gap());
    }

    /** Y. On/off (B/RT also turn it on). After a FAULT, the first press clears it and spins back up. */
    public void toggleFlywheel() {
        if (guard.isFault()) {
            guard.reset();
            flywheelOn = true;
        } else {
            flywheelOn = !flywheelOn;
        }
    }

    /** Auto: flywheel on/off directly (pre-spin before a launch spot, off after the last one). */
    public void setFlywheel(boolean on) {
        if (!guard.isFault()) flywheelOn = on;
    }

    /** Operator knows a nectar is next: nectar gap until the next shot. */
    public void requestNectar() {
        guard.requestNectar();
    }

    /** Run every loop. The only place any of these motors or the pivot are written. */
    public void update(Controls c) {
        double now = clock.seconds();

        if (guard.isFault()) {
            allStop();
            updateCollector(c);   // the driver keeps the collector even during a jam fault
            return;
        }

        // ---- Flywheels + firing --------------------------------------------
        // B or RT starts the flywheel (Shane, 2026-11-06; B was RB); it stays on until Y.
        // Backward controls never start it and never stop it. The Shooter still
        // holds the first ball until the wheels are at speed.
        if (c.fire || c.allForward) flywheelOn = true;
        if (flywheelOn) {
            shooter.spinUp(FLYWHEEL_RPM);
        } else {
            shooter.stop();
        }
        boolean firing = flywheelOn && (c.fire || c.allForward);
        if (firing) {
            // Keep exactly one shot queued while held. ShooterLogic feeds it when
            // the wheels are at speed, and pauses for recovery after each dip.
            if (shooter.getShotsQueued() == 0) shooter.fire(1);
        } else if (shooter.getShotsQueued() > 0 || shooter.ownsIndexer()) {
            shooter.cancelShots();
        }
        shooter.update();   // writes the wheels, and the indexer while it's feeding

        // ---- Indexer when the Shooter isn't feeding ------------------------
        boolean shooterOwns = shooter.ownsIndexer();
        if (shooterOwns) {
            shooterHadIndexer = true;
        } else {
            if (shooterHadIndexer) {
                indexerCmd = null;   // Shooter just let go (and stopped it); re-assert ours
                shooterHadIndexer = false;
            }
            // Forward only ever goes through the Shooter now (B/RT start the
            // flywheel), so the at-speed gate and the nectar guard see every ball.
            Motion want = (c.allBack || c.indexerBack) ? Motion.BACK : Motion.STOP;
            if (want != indexerCmd) {
                if (want == Motion.FORWARD) collection.feed();
                else if (want == Motion.BACK) collection.unjam();
                else collection.stopIndexer();
                indexerCmd = want;
            }
        }
        indexerForward = shooterOwns || indexerCmd == Motion.FORWARD;

        updateCollector(c);

        // ---- The guard -----------------------------------------------------
        // Current is an extra hub read, so only while it matters.
        indexerAmps = indexerForward
                ? collection.getIndexerCurrentAmps() * (12.0 / Math.max(battery.getVoltage(), 1.0))
                : 0;
        int fired = shooter.getShotsFired();
        boolean shotSeen = fired > lastShotsFired;
        lastShotsFired = fired;
        double drop = flywheelOn
                ? shooter.getTargetRpm() - Math.min(shooter.getLeftRpm(), shooter.getRightRpm())
                : Double.NaN;

        guard.update(now, indexerForward, indexerAmps, shotSeen, drop);
        if (guard.isFault()) {
            allStop();
            return;
        }
        applyGap(guard.gap());
    }

    /** Collector: triggers win over buttons. Runs even during a jam fault. */
    private void updateCollector(Controls c) {
        Motion collectorWant;
        if (c.allForward) collectorWant = Motion.FORWARD;
        else if (c.allBack) collectorWant = Motion.BACK;
        else if (c.collectIn) collectorWant = Motion.FORWARD;
        else if (c.collectOut) collectorWant = Motion.BACK;
        else collectorWant = Motion.STOP;
        if (collectorWant != collectorCmd) {
            if (collectorWant == Motion.FORWARD) collection.collect();
            else if (collectorWant == Motion.BACK) collection.eject();
            else collection.stopCollector();
            collectorCmd = collectorWant;
        }
    }

    // Jam fault: flywheel + indexer only. The collector isn't what gets hurt,
    // and cutting it mid-drive cost us (6494, 2026-11-06).
    private void allStop() {
        flywheelOn = false;
        shooter.stop();
        shooter.update();
        collection.stopIndexer();
        indexerCmd = Motion.STOP;
        indexerForward = false;
        applyGap(Gap.OPEN);   // release whatever is pinched
    }

    private void applyGap(Gap gap) {
        if (pivot == null || gap == appliedGap) return;
        switch (gap) {
            case POLLEN: pivot.goToPollen(); break;
            case NECTAR: pivot.goToNectar(); break;
            case OPEN:
                if (!pivot.goToOpen()) pivot.goToNectar();   // widest we have
                break;
        }
        appliedGap = gap;
    }

    // ---- Readouts -----------------------------------------------------------

    public boolean isFlywheelOn()   { return flywheelOn; }
    /** Wheels at speed, nothing in flight: a shot now would be a good one. */
    public boolean isReady()        { return flywheelOn && shooter.getState() == ShooterLogic.State.READY; }
    public Shooter getShooter()     { return shooter; }
    public NectarGuard getGuard()   { return guard; }
    public boolean isPivotBlocked() { return pivot == null; }
    public Gap getGap()             { return appliedGap; }
    public double getIndexerAmps()  { return indexerAmps; }
    public boolean isIndexerForward() { return indexerForward; }
}
