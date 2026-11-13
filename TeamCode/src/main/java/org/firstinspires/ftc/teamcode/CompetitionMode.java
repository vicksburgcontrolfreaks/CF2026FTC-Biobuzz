package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

/**
 * ONE switch that hides the bench-only options before a competition.
 *
 * WHY THIS EXISTS
 * Development knobs keep ending up in the same INIT menu the drive team uses
 * thirty seconds before a match. Each one is a row to scroll past and a way to
 * start a match in a configuration nobody meant to pick. Deleting them before
 * an event and adding them back after is worse -- that is a code change under
 * time pressure, on competition day, to the op mode that runs the match.
 *
 * So instead: every bench-only option is written ONCE with
 * {@code showWhen(() -> !CompetitionMode.ON)}, and this flag turns the whole
 * set off together.
 *
 * HOW TO USE IT AT AN EVENT
 * Set ON = true and deploy, or flip it in Panels at the field. The rows below
 * disappear from the menu and the op modes behave as if nobody had ever picked
 * anything unusual. Set it back to false to get them back.
 *
 * WHAT IT HIDES (keep this list current -- it is the point of the class)
 *   DriveTeleOp "Gamepads"   one-gamepad solo testing. Forced back to TWO.
 *   DriveTeleOp "Aim test"   the aim diagnostic presets. Forced back to NORMAL.
 *
 * ADDING TO THE LIST
 *   1. Give the menu row {@code .showWhen(() -> !CompetitionMode.ON)}.
 *   2. In the op mode's init(), force the row to its safe value when ON, so a
 *      hidden row cannot still be holding a bench setting from an earlier run.
 *      Hiding a row does NOT reset it -- that is the trap this step closes.
 *   3. Add a line to the list above.
 *
 * WHAT THIS IS NOT
 * Not a safety interlock and not a rules check. It only hides OPTIONS. It does
 * not touch the per-robot numbers in RobotProfile, Turret or ShooterPivot, and
 * it will not make an untested configuration legal or safe.
 */
@Configurable
public final class CompetitionMode {

    private CompetitionMode() {}

    /**
     * true = hide the bench-only menu rows listed in this class's comment.
     *
     * FALSE during the season's build and test work, which is most of the time.
     * Turn it on for an event.
     */
    public static boolean ON = false;
}
