package org.firstinspires.ftc.teamcode.modules;

import androidx.core.math.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.util.Modulable;
import org.firstinspires.ftc.teamcode.modules.util.Tickable;
import org.firstinspires.ftc.teamcode.util.FinishCondition;

public class RunToPositionMotor implements Modulable, Tickable, FinishCondition {
    protected final String name;
    protected final double power;
    private final DcMotorSimple.Direction direction;
    protected final int min;
    protected final int max;
    protected DcMotorEx motor;

    public RunToPositionMotor(String name, double power, DcMotorSimple.Direction direction) {
        this(name, power, direction, Integer.MIN_VALUE, Integer.MAX_VALUE);
    }

    public RunToPositionMotor(String name, double power, DcMotorSimple.Direction direction, int min, int max) {
        this.name = name;
        this.power = power;
        this.direction = direction;
        this.min = min;
        this.max = max;
    }

    @Override
    public void init(HardwareMap map) {
        motor = (DcMotorEx) map.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Warning: DO NOT use this if motor is currently running to position. Undefined behavior.
     */
    public void moveUsingEncoder(double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
    }

    public void startMoveToRelativePos(int relativePosition) {
        startMoveToPos(MathUtils.clamp(getCurrentPosition() + relativePosition, min, max));
    }

    public void startMoveToPos(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public void startRetraction() {
        startMoveToPos(0);
    }

    /**
     * This class only implements {@link Tickable} for convenience,
     * since this is the only common super class between {@link LinearSlide} and {@link DoubleLinearSlides}.
     */
    @Override
    public void tick() {}

    /**
     * @return true if the motor is at the target position
     */
    @Override
    public boolean isFinished() {
        return !motor.isBusy();
    }

    public void stop() {
        startMoveToRelativePos(0);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public double getPower() {
        return motor.getPower();
    }

    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    /**
     * Encoders may not work when the wire is not compatible with the motor.
     *
     * @return the current reading of the encoder for this motor
     * @see DcMotor#getCurrentPosition()
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
}
