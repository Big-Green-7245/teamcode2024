package org.firstinspires.ftc.teamcode.modules.output;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.Modulable;

public class ServoToggle implements Modulable {
    private String name;
    protected Servo servo;
    private boolean action = false;

    public void init(HardwareMap map, String servoName, double idlePos, double actionPos, boolean isReversed) {
        name = servoName;
        servo = map.get(Servo.class, name);
        if (isReversed) {
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }
        servo.scaleRange(idlePos, actionPos);
        setAction(action);
    }

    @Override
    public void init(HardwareMap map) {
        name = "outputClaw";
        servo = map.get(Servo.class, name);
        servo.setDirection(Servo.Direction.REVERSE);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    /**
     * Starts to move to claw to the given position where 0 is the idlePos passed in init and 1 is the actionPos passed in init.
     *
     * @param position the position between 0 and 1
     */
    public void setPosition(double position) {
        servo.setPosition(position);
    }

    /**
     * Makes the claw start to move towards the specified position.
     *
     * @param isAction open or close the claw
     */
    public void setAction(boolean isAction) {
        this.action = isAction;
        if (action) {
            servo.setPosition(1);
        } else {
            servo.setPosition(0);
        }
    }

    /**
     * Start to move the claw opposite to the current state.
     */
    public void toggleAction() {
        setAction(!action);
    }
}

