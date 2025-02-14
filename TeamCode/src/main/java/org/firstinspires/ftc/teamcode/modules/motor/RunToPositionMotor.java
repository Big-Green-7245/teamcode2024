package org.firstinspires.ftc.teamcode.modules.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RunToPositionMotor extends RunToPosition {
    private final DcMotorSimple.Direction direction;
    protected DcMotorEx motor;

    public RunToPositionMotor(String name, double power, DcMotorSimple.Direction direction) {
        this(name, power, direction, Integer.MIN_VALUE, Integer.MAX_VALUE);
    }

    public RunToPositionMotor(String name, double power, DcMotorSimple.Direction direction, int min, int max) {
        super(name, power, min, max);
        this.direction = direction;
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
    @Override
    public void moveUsingEncoder(double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
    }

    @Override
    public void startMoveToPos(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    /**
     * @return true if the motor is at the target position
     */
    @Override
    public boolean isFinished() {
        return !motor.isBusy();
    }

    @Override
    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    /**
     * Encoders may not work when the wire is not compatible with the motor.
     *
     * @return the current reading of the encoder for this motor
     * @see DcMotor#getCurrentPosition()
     */
    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
}
