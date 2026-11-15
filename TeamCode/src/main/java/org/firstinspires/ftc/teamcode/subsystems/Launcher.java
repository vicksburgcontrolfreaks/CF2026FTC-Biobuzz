package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
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
 * Pivot: both robots have measured positions (ShooterPivot.loadRobotNumbers).
 * isPivotBlocked() stays for a robot that ever lacks them -- the guard
 * still watches, but it can't move anything.
 */
public class Launcher {

    /**
     * Fixed for now; ShotTable (distance -> RPM) comes with odometry aiming.
     * Per robot -- see loadRobotNumbers(). This bare value is 6494's / the fallback.
     */
    public static double FLYWHEEL_RPM = 1280;

    /**
     * The AUTO speed, per robot. RouteAuto calls useAutoRpm() after building
     * the Launcher, which copies this into FLYWHEEL_RPM for that op mode only --
     * every new Launcher reloads the TeleOp number, so TeleOp can't inherit it.
     * Same as FLYWHEEL_RPM unless a robot needs a different auto shot.
     */
    public static double AUTO_FLYWHEEL_RPM = 1280;

    /** Autos: shoot at this robot's AUTO_FLYWHEEL_RPM instead of the TeleOp speed. */
    public void useAutoRpm() {
        FLYWHEEL_RPM = AUTO_FLYWHEEL_RPM;
    }

    /**
     * This robot's flywheel speed. Every op mode that shoots starts from this
     * (DriveTeleOp and the autos through Launcher, Pollen Shot Tune and Shooter
     * Test as their starting value), so there is one place to change it.
     */
    public static double defaultFlywheelRpm() {
        loadRobotNumbers();
        return FLYWHEEL_RPM;
    }

    private static void loadRobotNumbers() {
        try {
            switch (org.firstinspires.ftc.teamcode.WhichRobot.get()) {
                case R5618:
                    // GOLDILOCKS after the full 5618 rebuild (Jade, 2026-11-14):
                    // TeleOp 1225, auto 1275. FLYWHEEL_RPM is DriveTeleOp's speed and
                    // Pollen Shot Tune's starting value; the autos switch to
                    // AUTO_FLYWHEEL_RPM (see useAutoRpm()).
                    // (Was 1100 for both: Shane, 2026-11-11, before the rebuild.)
                    FLYWHEEL_RPM = 1225;
                    AUTO_FLYWHEEL_RPM = 1275;
                    break;
                case R6494:
                    // 1300 -> 1280: Shane, 2026-11-14. This is the SHOT MAP's answer
                    // rather than a guess -- 263 balls over 8 spots put the balance
                    // point near 1287 with a mean band width of ~101 RPM, so one RPM
                    // covers 20-52 in and there is no table to build yet. See
                    // docs/shot-map-2026-11-13.md. 1280 sits inside every band
                    // measured.
                    FLYWHEEL_RPM = 1280;
                    AUTO_FLYWHEEL_RPM = 1280;   // same as TeleOp (unchanged behaviour)
                    break;
            }
        } catch (IllegalStateException e) {
            // Unknown robot: keep the default (6494's).
        }
    }

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

    /**
     * DIRECT FEED -- the simple mode, and the default since 2026-11-13.
     *
     * Shane, during a driver session: "our current state is really more of a do
     * what we say kind of mode where we hold a button and it does exactly that
     * for as long as we want it to." The old path does not do that, and five
     * specific things in it stop the indexer WHILE THE BUTTON IS STILL HELD:
     *
     *   1. NO_BALL_SEC -- no 60 RPM dip within 6 s and it zeroes the queue and
     *      quits, saying "no ball?";
     *   2. RECOVERING waits for the wheels to be at speed AGAIN after every ball,
     *      so a flywheel that does not recover parks the indexer indefinitely;
     *   3. one ball per queue slot with a pause between, so it stutters by design;
     *   4. DIP_RPM doubles as the ball COUNTER, so a dip that does not match
     *      reality miscounts shots and feeds back into 1 and 2;
     *   5. NectarGuard's jam backstop can latch FAULT and allStop() both motors
     *      until someone presses Y.
     *
     * 1-4 are last year's flipper sequence translated onto the indexer motor and
     * never validated against this mechanism. 5 is real protection but a false
     * trip costs the rest of the match.
     *
     * Also: WE ARE NOT HANDLING NECTAR at the next competition (Shane), so the
     * nectar detection and gap switching have nothing to do.
     *
     * WHAT DIRECT MODE DOES: while fire/allForward is held, run the indexer
     * forward, continuously, until it is let go. No queue, no dip detection, no
     * recovery pause, no no-ball timeout. The ONE piece of cleverness kept is a
     * one-time spin-up gate -- see spunUpThisHold.
     *
     * WHY THIS IS A FLAG AND NOT A DELETION: ShooterLogicTest (22 tests) and
     * NectarGuardTest (24) encode the old behaviour, nectar comes back later in
     * the season, and this is the match-critical path with a competition close --
     * so it reverts in one line at the event. Set false for the old behaviour.
     */
    public static boolean DIRECT_FEED = true;

    /**
     * Direct mode only. The flywheel stops when the fire button is released
     * (Shane's call, 2026-11-13: "stop when released", chosen over the latch).
     *
     * Y still works, and is now a PRE-SPIN rather than an on/off latch: spin up
     * early and the next press feeds with no wait, since the gate below is
     * already satisfied. That buys back most of what stop-on-release gives up.
     */
    private boolean flywheelPreSpin = false;

    /**
     * Direct mode only. True once the wheels have reached speed during THIS hold.
     *
     * The gate is one-time on purpose. Holding the first ball until the wheels
     * are up stops a weak first shot; re-checking per ball is item 2 above, which
     * is the behaviour we are removing. So once it latches, the indexer keeps
     * running until the button is released no matter what the wheels do.
     */
    private boolean spunUpThisHold = false;

    /**
     * AUTO ONLY (Shane, 2026-11-14): no at-speed gate. The indexer runs the
     * moment fire/allForward is held, whatever the wheels are doing. RouteAuto
     * runs on the clock instead -- a fixed spin-up, then a fixed feed time -- so
     * a flywheel that never quite reaches tolerance can no longer silently
     * cancel a whole shot. TeleOp keeps the gate. Direct mode only.
     */
    private boolean feedUngated = false;

    /** Autos: feed on command, no wait for flywheel speed. See feedUngated. */
    public void useUngatedFeed() {
        feedUngated = true;
    }

    private boolean flywheelOn = false;
    private Gap appliedGap = null;
    private Motion indexerCmd = null;    // null = unknown, forces the next write
    private Motion collectorCmd = null;
    private boolean shooterHadIndexer = false;
    private int lastShotsFired = 0;
    private boolean indexerForward = false;
    private double indexerAmps = 0;

    public Launcher(RobotHardware hw, HardwareMap hardwareMap) {
        loadRobotNumbers();
        collection = new CollectionSystem(hw.collector, hw.indexer, hw.colorSensor);
        shooter = new Shooter(hw.shooterLeft, hw.shooterRight, collection);
        pivot = new ShooterPivot(hw.shooterPivot);   // positions per robot (ShooterPivot.loadRobotNumbers)
        battery = hardwareMap.voltageSensor.iterator().next();
    }

    /** Call from start() (servos shouldn't move during INIT). Puts the pivot at the guard's gap. */
    public void start() {
        appliedGap = null;
        applyGap(guard.gap());
    }

    /**
     * Y. In DIRECT_FEED mode this is a PRE-SPIN toggle, not an on/off latch: the
     * flywheel runs while fire is held regardless, and this holds it on in
     * between so the next press feeds with no spin-up wait. Pressing it while
     * pre-spun turns the pre-spin back off.
     *
     * In the old mode it is the on/off latch it always was, and after a FAULT the
     * first press clears the fault and spins back up.
     */
    public void toggleFlywheel() {
        if (DIRECT_FEED) {
            flywheelPreSpin = !flywheelPreSpin;
            return;
        }
        if (guard.isFault()) {
            guard.reset();
            flywheelOn = true;
        } else {
            flywheelOn = !flywheelOn;
        }
    }

    /** Auto: flywheel on/off directly (pre-spin before a launch spot, off after the last one). */
    public void setFlywheel(boolean on) {
        if (DIRECT_FEED) {
            flywheelPreSpin = on;
            return;
        }
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

        if (DIRECT_FEED) {
            updateDirect(c, now);
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

    /**
     * DIRECT MODE. Held means running; released means stopped. That is the whole
     * contract, and it is the point -- see DIRECT_FEED.
     *
     * The Shooter is demoted to a flywheel controller here. We never queue a shot,
     * so ShooterLogic never enters FEEDING, so ownsIndexer() stays false and it
     * never touches the indexer. This method is the only thing driving it, which
     * is why there is no handover dance like the old path has.
     *
     * Reading at-speed off getState() == READY is deliberate rather than lazy:
     * spinUp() walks IDLE -> SPINNING_UP -> READY on exactly the tolerance and
     * settle time the old path used, so the gate keeps that tuning instead of
     * inventing a second definition of "at speed".
     */
    private void updateDirect(Controls c, double now) {
        boolean want = c.fire || c.allForward;

        // ---- Flywheel: on while held, or while pre-spun with Y ---------------
        flywheelOn = want || flywheelPreSpin;
        if (flywheelOn) {
            shooter.spinUp(FLYWHEEL_RPM);
        } else {
            shooter.stop();
        }
        shooter.update();   // writes the wheels only; it never owns the indexer here

        // ---- The one-time spin-up gate --------------------------------------
        if (!want) {
            spunUpThisHold = false;   // next hold re-gates
        } else if (shooter.getState() == ShooterLogic.State.READY) {
            spunUpThisHold = true;
        }

        // ---- Indexer ---------------------------------------------------------
        // Backward wins: it is the jam-clearing control and the driver reaching
        // for it is already unhappy. Forward needs the gate; nothing else can
        // stop it while held.
        Motion wantIndexer;
        if (c.allBack || c.indexerBack) {
            wantIndexer = Motion.BACK;
        } else if (want && (spunUpThisHold || feedUngated)) {
            wantIndexer = Motion.FORWARD;
        } else {
            wantIndexer = Motion.STOP;
        }
        if (wantIndexer != indexerCmd) {
            if (wantIndexer == Motion.FORWARD) collection.feed();
            else if (wantIndexer == Motion.BACK) collection.unjam();
            else collection.stopIndexer();
            indexerCmd = wantIndexer;
        }
        indexerForward = wantIndexer == Motion.FORWARD;

        updateCollector(c);

        // ---- Diagnostics only ------------------------------------------------
        // The amps still go on the Driver Station because they are how the nectar
        // threshold was measured and how a jam would be spotted by eye. Nothing
        // ACTS on them in this mode: no latching fault can take the shooter away
        // mid-match. The guard is not consulted and not updated, and the pivot
        // stays wherever start() put it, which is the pollen gap.
        indexerAmps = indexerForward
                ? collection.getIndexerCurrentAmps() * (12.0 / Math.max(battery.getVoltage(), 1.0))
                : 0;
        lastShotsFired = shooter.getShotsFired();
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
