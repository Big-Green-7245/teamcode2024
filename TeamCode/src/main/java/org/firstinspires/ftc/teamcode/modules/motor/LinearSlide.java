package org.firstinspires.ftc.teamcode.modules.motor;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This class adds a limit switch (rev touch sensor) at position 0 on top of a run to position motor.
 */
public class LinearSlide extends RunToPositionMotor {
    private TouchSensor elevatorButton;

    public LinearSlide(String name, double power) {
        this(name, power, DcMotorSimple.Direction.FORWARD);
    }

    public LinearSlide(String name, double power, DcMotorSimple.Direction direction) {
        super(name, power, direction);
    }

    public LinearSlide(String name, double power, DcMotorSimple.Direction direction, int min, int max) {
        super(name, power, direction, min, max);
    }

    public boolean isElevatorBtnPressed() {
        return elevatorButton.isPressed();
    }

    @Override
    public void init(HardwareMap map) {
        super.init(map);
        elevatorButton = map.get(RevTouchSensor.class, motorInfo.get(0).name() + "Btn");
    }

    /**
     * Starts to move the intakeSlide to the ground position.
     * ONLY call this function once for every move to ground!
     * YOU MUST call {@link #tick()} in a loop to stop the intakeSlide when it reaches the ground.
     */
    @Override
    public void startRetraction() {
        startMoveToPos(-2000);
    }

    /**
     * Checks if the slide is retracting and pressing the button. If it is, reset the encoder.
     */
    @Override
    public void tick() {
        if (elevatorButton.isPressed() && motor.isBusy()) {
            int targetPosLeft = motor.getTargetPosition();
            double powerLeft = motor.getPower();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(Math.max(targetPosLeft, min));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(powerLeft);
        }
    }
}
