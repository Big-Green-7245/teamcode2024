package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.modules.util.Modulable;
import org.firstinspires.ftc.teamcode.modules.util.Tickable;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoOutputPivot implements Modulable, Tickable {
    public static final double CLOSED_POS = 0;
    private static final double OPENED_POS = 0.5;
    private static final double TARGET_TIME = 1.4;
    private final String name;
    private CRServo pivot;
    private ServoController controller;
    private final ElapsedTime runtime;
    private boolean output = false;
    private boolean isRunning;

    public TouchSensor intakeButton;

    public ServoOutputPivot(String name) {
        this.name = name;
        this.runtime = new ElapsedTime();
    }

    public double getPosition() {
        return controller.getServoPosition(0);
    }

    public void setPosition(double position) {
        controller.setServoPosition(0, position);
    }

    @Override
    public void init(HardwareMap map) {
        pivot = map.get(CRServo.class, name);
        pivot.setDirection(CRServo.Direction.REVERSE);
        controller = pivot.getController();
        intakeButton = map.get(TouchSensor.class, "intakeButton");
    }

    /**
     * Makes the claw start to move towards the specified position.
     *
     * @param output open or close the claw
     */
    public void setClawOutput(boolean output) {
        this.output = output;
        startMovePivot();
    }

    /**
     * Start to move the claw opposite to the current state.
     */
    public void togglePivot() {
        if (!isRunning) {
            output = !output;
            startMovePivot();
        }
    }

    private void startMovePivot() {
        if (!isRunning) {
            runtime.reset();
            setPower(output ? 1.0 : -1.0);
            isRunning = true;
        }
    }

    public void setPower(double power) {
        isRunning = false;
        pivot.setPower(power);
    }

    public boolean isFinished() {
        return !isRunning;
    }

    @Override
    public void tick() {
        if (isRunning) {
            if (intakeButton.isPressed() && !output) {
                isRunning = false;
                setPower(0);
            }
            if (runtime.seconds() >= TARGET_TIME) {
                isRunning = false;
                setPower(0);
            }
        }
    }
}

