package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

/**
 * TelemetryWrapper - Makes the telemetry readouts a bit more readable and manageable.
 */

public class TelemetryWrapper {
    private final Telemetry telemetry;
    private final String[] lines;
    private final Object[] args;

    public TelemetryWrapper(Telemetry telemetry, int lines) {
        this.telemetry = telemetry;
        this.lines = new String[lines];
        args = new Object[lines];
        telemetry.clear();
        reset();
    }

    /**
     * Sets the given line to the given message.
     *
     * @param line    the line number
     * @param message the message string
     */
    public void setLine(int line, String message) {
        if (line < 0 || line >= lines.length) return;
        lines[line] = message;
    }

    /**
     * Sets the given line to the given format and arguments.
     *
     * @param line   the line number
     * @param format the format string
     * @param args   the arguments to be formatted
     */
    public void setLine(int line, String format, Object... args) {
        setLine(line, format);
        this.args[line] = args;
    }

    /**
     * Sets the given line to the given message and update the telemetry.
     * Use {@link #setLine(int, String)} and {@link #render()} if updating multiple lines of telemetry.
     *
     * @param line    the line number
     * @param message the message string
     */
    public void setLineAndRender(int line, String message) {
        setLine(line, message);
        render();
    }

    /**
     * Updates the telemetry, with arguments if not null.
     */
    public void render() {
        for (int i = 0; i < lines.length; i++) {
            if (lines[i].isEmpty()) continue;
            if (args[i] != null) {
                telemetry.addData("" + i, lines[i], args[i]);
            } else {
                telemetry.addData("" + i, lines[i]);
            }
        }
        telemetry.update();
    }

    /**
     * Resets all lines and arguments.
     */
    public void reset() {
        Arrays.fill(lines, "");
        Arrays.fill(args, null);
    }
}
