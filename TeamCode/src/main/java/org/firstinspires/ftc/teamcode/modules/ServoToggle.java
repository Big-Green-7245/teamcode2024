package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.modules.util.Modulable;

public class ServoToggle implements Modulable {
    protected final String name;
    protected final double idlePos;
    protected final double actionPos;
    private final Servo.Direction direction;
    private Servo servo;
    protected boolean action = false;

    public ServoToggle(String name, double idlePos, double actionPos, boolean isReversed) {
        this(name, idlePos, actionPos, isReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    public ServoToggle(String name, double idlePos, double actionPos, Servo.Direction direction) {
        this.name = name;
        this.idlePos = idlePos;
        this.actionPos = actionPos;
        this.direction = direction;
    }

    @Override
    public void init(HardwareMap map) {
        servo = map.get(Servo.class, name);
        servo.setDirection(direction);
        servo.scaleRange(idlePos, actionPos);
        setAction(action);
    }

    public double getPosition() {
        return getPosition(servo);
    }

    public double getPosition(Servo servo) {
        return getPosition(servo, idlePos, actionPos);
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
     * @param action open or close the claw
     */
    public void setAction(boolean action) {
        this.action = action;
        setPosition(action ? 1 : 0);
    }

    /**
     * Start to move the claw opposite to the current state.
     */
    public void toggleAction() {
        setAction(!action);
    }

    /**
     * Gets the position of the servo, scaled to the given min and max positions.
     * <p>
     * This gets around the incorrect reverse in {@link com.qualcomm.robotcore.hardware.ServoImpl#getPosition() ServoImpl#getPosition()},
     * where {@link com.qualcomm.robotcore.hardware.ServoImpl#reverse(double) ServoImpl#reverse(double)} is called before {@link Range#scale(double, double, double, double, double)}.
     */
    @SuppressWarnings("JavadocReference")
    public static double getPosition(Servo servo, double posMin, double posMax) {
        double position = servo.getController().getServoPosition(servo.getPortNumber());
        double scaled = Range.scale(position, posMin, posMax, Servo.MIN_POSITION, Servo.MAX_POSITION);
        if (servo.getDirection() == Servo.Direction.REVERSE) scaled = Servo.MAX_POSITION - scaled + Servo.MIN_POSITION;
        return Range.clip(scaled, Servo.MIN_POSITION, Servo.MAX_POSITION);
    }
}
