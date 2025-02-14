package org.firstinspires.ftc.teamcode.modules.motor;

import androidx.core.math.MathUtils;
import org.firstinspires.ftc.teamcode.modules.util.Modulable;
import org.firstinspires.ftc.teamcode.modules.util.Tickable;
import org.firstinspires.ftc.teamcode.util.FinishCondition;

public abstract class RunToPosition implements Modulable, Tickable, FinishCondition {
    protected final String name;
    protected final double power;
    protected final int min;
    protected final int max;

    public RunToPosition(String name, double power) {
        this(name, power, Integer.MIN_VALUE, Integer.MAX_VALUE);
    }

    protected RunToPosition(String name, double power, int min, int max) {
        this.name = name;
        this.power = power;
        this.min = min;
        this.max = max;
    }

    public abstract void moveUsingEncoder(double power);

    public abstract void startMoveToPos(int position);

    public void startMoveToRelativePos(int relativePosition) {
        startMoveToPos(MathUtils.clamp(getCurrentPosition() + relativePosition, min, max));
    }

    public void startRetraction() {
        startMoveToPos(0);
    }

    public void stop() {
        startMoveToRelativePos(0);
    }

    /**
     * This class only implements {@link Tickable} for convenience,
     * since this is the only common super class between {@link LinearSlide} and {@link DoubleLinearSlides}.
     */
    @Override
    public void tick() {}

    public abstract double getCurrent();

    public abstract double getPower();

    public abstract int getTargetPosition();

    public abstract int getCurrentPosition();
}
