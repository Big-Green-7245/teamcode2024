package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.modules.util.Modulable;
import org.firstinspires.ftc.teamcode.modules.util.Tickable;
import org.firstinspires.ftc.teamcode.util.FinishCondition;

/**
 * This class adds a limit switch (rev touch sensor) at position 0 on top of two run to position motors.
 * Otherwise, it is the same as {@link TwoRunToPositionMotors}.
 * The only difference between this and {@link LinearSlide} is that this class has two motors instead of one.
 */
public class DoubleLinearSlides extends TwoRunToPositionMotors implements Modulable, Tickable, FinishCondition {
    private TouchSensor elevatorBtnLeft;
    private TouchSensor elevatorBtnRight;

    public DoubleLinearSlides(String name, double power, DcMotorSimple.Direction directionLeft, DcMotorSimple.Direction directionRight) {
        super(name, power, directionLeft, directionRight);
    }

    public DoubleLinearSlides(String name, double power, DcMotorSimple.Direction directionLeft, DcMotorSimple.Direction directionRight, int min, int max) {
        super(name, power, directionLeft, directionRight, min, max);
    }

    public boolean[] areElevatorButtonsPressed() {
        return new boolean[]{elevatorBtnLeft.isPressed(), elevatorBtnRight.isPressed()};
    }

    @Override
    public void init(HardwareMap map) {
        super.init(map);
        elevatorBtnLeft = map.get(RevTouchSensor.class, name + "LeftBtn");
        elevatorBtnRight = map.get(RevTouchSensor.class, name + "RightBtn");
    }

    /**
     * Starts to move the intakeSlide to the ground position.
     * ONLY call this function once for every move to ground!
     * YOU MUST call {@link #tick()} in a loop to stop the intakeSlide when it reaches the ground.
     */
    @Override
    public void startRetraction() {
        startMoveToPos(-1000);
    }

    /**
     * Checks if either slide is retracting and pressing the button. If it is, reset the encoder.
     */
    @Override
    public void tick() {
        if ((elevatorBtnLeft.isPressed() && motorLeft.isBusy()) || (elevatorBtnRight.isPressed() && motorRight.isBusy())) {
            int targetPosLeft = motorLeft.getTargetPosition();
            int targetPosRight = motorRight.getTargetPosition();
            double powerLeft = motorLeft.getPower();
            double powerRight = motorRight.getPower();
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setTargetPosition(Math.max(targetPosLeft, 10));
            motorRight.setTargetPosition(Math.max(targetPosRight, 10));
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeft.setPower(powerLeft);
            motorRight.setPower(powerRight);
        }
    }
}
