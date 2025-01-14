package org.firstinspires.ftc.teamcode.modules.output;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DoubleLinearSlides extends LinearSlide {
    private final DcMotorSimple.Direction directionLeft;
    private final DcMotorSimple.Direction directionRight;
    private DcMotorEx elevatorLeft;
    private DcMotorEx elevatorRight;
    private TouchSensor elevatorBtnLeft;
    private TouchSensor elevatorBtnRight;

    public DoubleLinearSlides(String name, double power, DcMotorSimple.Direction directionLeft, DcMotorSimple.Direction directionRight) {
        super(name, power, null);
        this.directionLeft = directionLeft;
        this.directionRight = directionRight;
    }

    public DoubleLinearSlides(String name, double power, DcMotorSimple.Direction directionLeft, DcMotorSimple.Direction directionRight, int limit) {
        super(name, power, null, limit);
        this.directionLeft = directionLeft;
        this.directionRight = directionRight;
    }

    @Override
    public boolean isElevatorBtnPressed() {
        return elevatorBtnLeft.isPressed() && elevatorBtnRight.isPressed();
    }

    public boolean[] areElevatorButtonsPressed() {
        return new boolean[]{elevatorBtnLeft.isPressed(), elevatorBtnRight.isPressed()};
    }

    @Override
    public void init(HardwareMap map) {
        elevatorBtnLeft = map.get(RevTouchSensor.class, name + "LeftBtn");
        elevatorBtnRight = map.get(RevTouchSensor.class, name + "RightBtn");
        elevatorLeft = (DcMotorEx) map.get(DcMotor.class, name + "Left");
        elevatorLeft.setDirection(directionLeft);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorRight = (DcMotorEx) map.get(DcMotor.class, name + "Right");
        elevatorRight.setDirection(directionRight);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Warning: DO NOT use this if motor is currently running to position. Undefined behavior.
     */
    @Override
    public void moveUsingEncoder(double power) {
        elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorLeft.setPower(power);
        elevatorRight.setPower(power);
    }

    @Override
    public void startMoveToPos(int position) {
        elevatorLeft.setTargetPosition(position);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(power);
        elevatorRight.setTargetPosition(position);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setPower(power);
    }

    /**
     * Checks if the slider is retracting and pressing the button. If it is, reset the encoder.
     */
    @Override
    public void tick() {
        if ((elevatorBtnLeft.isPressed() && elevatorLeft.isBusy()) || (elevatorBtnRight.isPressed() && elevatorRight.isBusy())) {
            int targetPosLeft = elevatorLeft.getTargetPosition();
            int targetPosRight = elevatorRight.getTargetPosition();
            double powerLeft = elevatorLeft.getPower();
            double powerRight = elevatorRight.getPower();
            elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorLeft.setTargetPosition(Math.max(targetPosLeft, 10));
            elevatorRight.setTargetPosition(Math.max(targetPosRight, 10));
            elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorLeft.setPower(powerLeft);
            elevatorRight.setPower(powerRight);
        }
    }

    /**
     * @return true if the elevator is at the target position
     * @implNote manually check the elevator position and the button because {@link DcMotor#isBusy()} has a lot of delay.
     */
    @Override
    public boolean isFinished() {
        return !elevatorLeft.isBusy() && !elevatorRight.isBusy();
    }

    @Override
    public double getCurrent() {
        return elevatorLeft.getCurrent(CurrentUnit.AMPS) + elevatorRight.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public double getPower() {
        return elevatorLeft.getPower() + elevatorRight.getPower();
    }

    @Override
    public int getTargetPosition() {
        return (elevatorLeft.getTargetPosition() + elevatorRight.getTargetPosition()) / 2;
    }

    public int[] getTargetPositions() {
        return new int[]{elevatorLeft.getTargetPosition(), elevatorRight.getTargetPosition()};
    }

    @Override
    public int getCurrentPosition() {
        return (elevatorLeft.getCurrentPosition() + elevatorRight.getCurrentPosition()) / 2;
    }

    public int[] getCurrentPositions() {
        return new int[]{elevatorLeft.getCurrentPosition(), elevatorRight.getCurrentPosition()};
    }
}
