package org.firstinspires.ftc.teamcode.modules.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorInfo {
    private final String name;
    private final DcMotorSimple.Direction direction;

    public MotorInfo(String name) {
        this(name, DcMotorSimple.Direction.FORWARD);
    }

    public MotorInfo(String name, DcMotorSimple.Direction direction) {
        this.name = name;
        this.direction = direction;
    }

    public String name() {
        return name;
    }

    public DcMotorEx motor(HardwareMap map) {
        DcMotorEx motor = ((DcMotorEx) map.get(DcMotor.class, name));
        motor.setDirection(direction);
        return motor;
    }
}
