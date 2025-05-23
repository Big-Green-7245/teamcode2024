package org.firstinspires.ftc.teamcode.modules.util;

/**
 * Implement this interface if a module/component needs to be updated every cycle in the main loop.
 * See <a href="https://minecraft.wiki/w/Tick">Tick</a>
 */
@FunctionalInterface
public interface Tickable {
    /**
     * Called after initialization is complete and before start is pressed.
     */
    default void tickBeforeStart() {
        tick();
    }

    /**
     * Called when the OpMode is running.
     */
    void tick();
}
