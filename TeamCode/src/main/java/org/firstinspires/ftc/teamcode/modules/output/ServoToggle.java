package org.firstinspires.ftc.teamcode.modules.output;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.Modulable;

public class ServoToggle implements Modulable {
    private double actionPos = 0;
    private double idlePos = 0.3;
    private String name;
    protected Servo servo;
    private boolean action = false;

    public void setIdlePos(double pos) {
        actionPos = pos;
    }

    public void setActionPos(double pos) {
        idlePos = pos;
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public void init(HardwareMap map, String servoName, double idlePos, double actionPos, boolean isReversed) {
        name = servoName;
        servo = map.get(Servo.class, name);
        if (isReversed) {
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }
        this.idlePos = idlePos;
        this.actionPos = actionPos;
        setAction(action);
    }

    @Override
    public void init(HardwareMap map) {
        name = "outputClaw";
        servo = map.get(Servo.class, name);
        servo.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Makes the claw start to move towards the specified position.
     *
     * @param isAction open or close the claw
     */
    public void setAction(boolean isAction) {
        this.action = isAction;
        if (action) {
            servo.setPosition(actionPos);
        } else {
            servo.setPosition(idlePos);
        }
    }

    /**
     * Start to move the claw opposite to the current state.
     */
    public void toggleAction() {
        action = !action;
        setAction(action);
    }
}

