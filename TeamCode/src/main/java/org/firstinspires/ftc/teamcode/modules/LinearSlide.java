package org.firstinspires.ftc.teamcode.modules;

import androidx.core.math.MathUtils;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.util.Modulable;
import org.firstinspires.ftc.teamcode.modules.util.Tickable;
import org.firstinspires.ftc.teamcode.util.FinishCondition;

public class LinearSlide implements Modulable, Tickable, FinishCondition {
    protected final String name;
    protected final double power;
    private final DcMotorSimple.Direction direction;
    protected final int limit;
    private DcMotorEx elevator;
    private TouchSensor elevatorButton;

    public LinearSlide(String name, double power) {
        this(name, power, DcMotorSimple.Direction.FORWARD);
    }

    public LinearSlide(String name, double power, DcMotorSimple.Direction direction) {
        this(name, power, direction, Integer.MAX_VALUE);
    }

    public LinearSlide(String name, double power, DcMotorSimple.Direction direction, int limit) {
        this.name = name;
        this.power = power;
        this.direction = direction;
        this.limit = limit;
    }

    public boolean isElevatorBtnPressed() {
        return elevatorButton.isPressed();
    }

    @Override
    public void init(HardwareMap map) {
        elevatorButton = map.get(RevTouchSensor.class, name + "Btn");
        elevator = (DcMotorEx) map.get(DcMotor.class, name);
        elevator.setDirection(direction);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Warning: DO NOT use this if motor is currently running to position. Undefined behavior.
     */
    public void moveUsingEncoder(double power) {
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setPower(power);
    }

    public void startMoveToRelativePos(int relativePosition) {
        startMoveToPos(MathUtils.clamp(getCurrentPosition() + relativePosition, 10, limit));
    }

    public void startMoveToPos(int position) {
        elevator.setTargetPosition(position);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(power);
    }

    /**
     * Starts to move the intakeSlide to the ground position.
     * ONLY call this function once for every move to ground!
     * YOU MUST call {@link #tick()} in a loop to stop the intakeSlide when it reaches the ground.
     */
    public void startRetraction() {
        startMoveToPos(-1000);
    }

    /**
     * Checks if the slide is retracting and pressing the button. If it is, reset the encoder.
     */
    @Override
    public void tick() {
        if (elevatorButton.isPressed() && elevator.isBusy()) {
            int targetPosLeft = elevator.getTargetPosition();
            double powerLeft = elevator.getPower();
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.setTargetPosition(Math.max(targetPosLeft, 10));
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(powerLeft);
        }
    }

    /**
     * @return true if the elevator is at the target position
     * @implNote manually check the elevator position and the button because {@link DcMotor#isBusy()} has a lot of delay.
     */
    @Override
    public boolean isFinished() {
        return !elevator.isBusy();
    }

    public void stop() {
        startMoveToRelativePos(0);
    }

    public double getCurrent() {
        return elevator.getCurrent(CurrentUnit.AMPS);
    }

    public double getPower() {
        return elevator.getPower();
    }

    public int getTargetPosition() {
        return elevator.getTargetPosition();
    }

    /**
     * Encoders may not work when the wire is not compatible with the motor.
     *
     * @return the current reading of the encoder for this motor
     * @see DcMotor#getCurrentPosition()
     */
    public int getCurrentPosition() {
        return elevator.getCurrentPosition();
    }
}
