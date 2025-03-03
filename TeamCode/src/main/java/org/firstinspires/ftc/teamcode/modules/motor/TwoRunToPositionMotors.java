package org.firstinspires.ftc.teamcode.modules.motor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

/**
 * Two motors instead of one.
 */
public class TwoRunToPositionMotors extends RunToPosition {
    // Allow a collection of motors for each of the left and right motors.
    protected final List<MotorInfo> motorInfoLeft;
    protected final List<MotorInfo> motorInfoRight;
    protected DcMotorEx motorLeft;
    protected DcMotorEx motorRight;

    public TwoRunToPositionMotors(String name, double power, DcMotorSimple.Direction directionLeft, DcMotorSimple.Direction directionRight) {
        this(name, power, directionLeft, directionRight, Integer.MIN_VALUE, Integer.MAX_VALUE);
    }

    public TwoRunToPositionMotors(String name, double power, DcMotorSimple.Direction directionLeft, DcMotorSimple.Direction directionRight, int min, int max) {
        this(List.of(new MotorInfo(name + "Left", directionLeft)), List.of(new MotorInfo(name + "Right", directionRight)), power, min, max);
    }

    public TwoRunToPositionMotors(List<MotorInfo> motorInfoLeft, List<MotorInfo> motorInfoRight, double power, int min, int max) {
        super(power, min, max);
        this.motorInfoLeft = motorInfoLeft;
        this.motorInfoRight = motorInfoRight;
    }

    @Override
    public void init(HardwareMap map) {
        motorLeft = DcMotorsEx.of(map, motorInfoLeft);
        motorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorRight = DcMotorsEx.of(map, motorInfoRight);
        motorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Warning: DO NOT use this if motor is currently running to position. Undefined behavior.
     */
    @Override
    public void moveUsingEncoder(double power) {
        motorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    @Override
    public void startMoveToPos(int position) {
        motorLeft.setTargetPosition(position);
        motorRight.setTargetPosition(position);
        motorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    /**
     * @return true if the motor is at the target position
     */
    @Override
    public boolean isFinished() {
        return !motorLeft.isBusy() || !motorRight.isBusy();
    }

    @Override
    public double getCurrent() {
        return motorLeft.getCurrent(CurrentUnit.AMPS) + motorRight.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public double getPower() {
        return motorLeft.getPower() + motorRight.getPower();
    }

    @Override
    public int getTargetPosition() {
        return (motorLeft.getTargetPosition() + motorRight.getTargetPosition()) / 2;
    }

    public int[] getTargetPositions() {
        return new int[]{motorLeft.getTargetPosition(), motorRight.getTargetPosition()};
    }

    @Override
    public int getCurrentPosition() {
        return (motorLeft.getCurrentPosition() + motorRight.getCurrentPosition()) / 2;
    }

    public int[] getCurrentPositions() {
        return new int[]{motorLeft.getCurrentPosition(), motorRight.getCurrentPosition()};
    }
}
