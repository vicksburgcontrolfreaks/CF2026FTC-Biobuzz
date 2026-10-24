package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

/*
 * A menu on the Driver Station for picking auto settings during INIT.
 *
 *   D-pad up/down     pick a row
 *   D-pad left/right  change that row
 *   A                 lock it in (the gamepad can't change anything after this)
 *   B                 unlock, if you need to fix something
 *
 * Adding a setting is one line -- the first choice is the default:
 *
 *     Choice<Integer> wait = menu.add("Wait (sec)", 0, 3, 5, 10);
 *
 * then use wait.get() wherever you need the value.
 *
 * Optional extras, chained on the end of add():
 *
 *     .summary("Wait %ds")          how it reads in the one-line summary
 *                                   (%d / %s = the value). Default: just the value.
 *     .showWhen(() -> someCheck())  hide the row unless someCheck() is true,
 *                                   for settings only some routes use.
 */
public final class AutoMenu {

    // One row of the menu: a name and the values you can pick from.
    // <T> means "any type" -- Alliance, a number, anything.
    public static final class Choice<T> {
        private final String name;
        private final List<T> options;
        private int index = 0;
        private String summaryFormat = "%s";
        // A BooleanSupplier is a tiny function that answers true/false when
        // asked. () -> true means "always show".
        private BooleanSupplier shown = () -> true;

        private Choice(String name, List<T> options) {
            this.name = name;
            this.options = options;
        }

        public T get() {
            return options.get(index);
        }

        public Choice<T> summary(String format) {
            summaryFormat = format;
            return this;
        }

        public Choice<T> showWhen(BooleanSupplier condition) {
            shown = condition;
            return this;
        }

        private boolean isShown() {
            return shown.getAsBoolean();
        }

        private String summaryText() {
            return String.format(summaryFormat, get());
        }

        // Step left (-1) or right (+1), wrapping around at the ends.
        private void step(int direction) {
            index = (index + direction + options.size()) % options.size();
        }
    }

    private final List<Choice<?>> rows = new ArrayList<>();
    private int selectedRow = 0;
    private boolean locked = false;

    @SafeVarargs
    public final <T> Choice<T> add(String name, T... options) {
        Choice<T> choice = new Choice<>(name, Arrays.asList(options));
        rows.add(choice);
        return choice;
    }

    public boolean isLocked() {
        return locked;
    }

    // Call every init_loop. Returns true if any setting changed, so the caller
    // knows to rebuild its paths.
    public boolean update(Gamepad gamepad) {
        // The "WasPressed" checks are true only once per press, so holding a
        // button doesn't scroll through everything.
        if (locked) {
            if (gamepad.bWasPressed()) locked = false;
            return false;
        }
        if (gamepad.aWasPressed()) {
            locked = true;
            return false;
        }
        // If the row we were on just got hidden, hop up to one that's showing.
        if (!rows.get(selectedRow).isShown()) moveSelection(-1);
        if (gamepad.dpadUpWasPressed())   moveSelection(-1);
        if (gamepad.dpadDownWasPressed()) moveSelection(+1);

        Choice<?> row = rows.get(selectedRow);
        if (gamepad.dpadLeftWasPressed())  { row.step(-1); return true; }
        if (gamepad.dpadRightWasPressed()) { row.step(+1); return true; }
        return false;
    }

    // Move up (-1) or down (+1) to the next row that's showing, skipping
    // hidden ones. Stays put if there isn't one.
    private void moveSelection(int direction) {
        for (int i = selectedRow + direction; i >= 0 && i < rows.size(); i += direction) {
            if (rows.get(i).isShown()) {
                selectedRow = i;
                return;
            }
        }
    }

    public void show(Telemetry telemetry) {
        // One line the drive team can check at a glance before the match.
        StringBuilder summary = new StringBuilder();
        for (Choice<?> row : rows) {
            if (!row.isShown()) continue;
            if (summary.length() > 0) summary.append(" · ");
            summary.append(row.summaryText());
        }
        telemetry.addLine(">>> " + summary + " <<<");

        if (locked) {
            telemetry.addLine("LOCKED  (B to change)");
        } else {
            telemetry.addLine("!! NOT LOCKED -- press A when it's right !!");
        }
        telemetry.addLine("");

        for (int i = 0; i < rows.size(); i++) {
            Choice<?> row = rows.get(i);
            if (!row.isShown()) continue;
            String pointer = (!locked && i == selectedRow) ? "> " : "   ";
            telemetry.addLine(pointer + row.name + ":  < " + row.get() + " >");
        }
        if (!locked) {
            telemetry.addLine("");
            telemetry.addLine("D-pad up/down: pick   left/right: change   A: lock");
        }
    }
}
