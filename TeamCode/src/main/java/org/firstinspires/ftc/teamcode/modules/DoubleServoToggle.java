package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DoubleServoToggle extends ServoToggle {
    private final Servo.Direction directionLeft;
    private final Servo.Direction directionRight;
    private Servo leftServo;
    private Servo rightServo;

    public DoubleServoToggle(String name, double idlePos, double actionPos, Servo.Direction directionLeft, Servo.Direction directionRight) {
        super(name, idlePos, actionPos, null);
        this.directionLeft = directionLeft;
        this.directionRight = directionRight;
    }

    @Override
    public void init(HardwareMap map) {
        leftServo = map.get(Servo.class, name + "Left");
        leftServo.setDirection(directionLeft);
        leftServo.scaleRange(idlePos, actionPos);

        rightServo = map.get(Servo.class, name + "Right");
        rightServo.setDirection(directionRight);
        rightServo.scaleRange(idlePos, actionPos);

        setAction(action);
    }

    @Override
    public double getPosition() {
        return (getPosition(leftServo) + getPosition(rightServo)) / 2;
    }

    @Override
    public void setPosition(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
}
