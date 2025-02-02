package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.util.Modulable;

/**
 * Two motors instead of one.
 */
public class TwoRunToPositionMotors extends RunToPositionMotor implements Modulable {
    private final DcMotorSimple.Direction directionLeft;
    private final DcMotorSimple.Direction directionRight;
    protected DcMotorEx motorLeft;
    protected DcMotorEx motorRight;

    public TwoRunToPositionMotors(String name, double power, DcMotorSimple.Direction directionLeft, DcMotorSimple.Direction directionRight) {
        super(name, power, null);
        this.directionLeft = directionLeft;
        this.directionRight = directionRight;
    }

    public TwoRunToPositionMotors(String name, double power, DcMotorSimple.Direction directionLeft, DcMotorSimple.Direction directionRight, int min, int max) {
        super(name, power, null, min, max);
        this.directionLeft = directionLeft;
        this.directionRight = directionRight;
    }

    @Override
    public void init(HardwareMap map) {
        motorLeft = (DcMotorEx) map.get(DcMotor.class, name + "Left");
        motorLeft.setDirection(directionLeft);
        motorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorRight = (DcMotorEx) map.get(DcMotor.class, name + "Right");
        motorRight.setDirection(directionRight);
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
