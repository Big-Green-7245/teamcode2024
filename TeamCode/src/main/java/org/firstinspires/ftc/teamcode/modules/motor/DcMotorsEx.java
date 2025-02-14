package org.firstinspires.ftc.teamcode.modules.motor;

import android.util.Pair;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;
import java.util.stream.Collectors;

/**
 * A group of motors that implements the {@link DcMotorEx} interface.
 * Get methods will delegate to the first motor in the list.
 * Set methods will call the same method on all motors in the list.
 */
public class DcMotorsEx implements DcMotorEx {
    private final List<DcMotorEx> motors;

    public DcMotorsEx(List<DcMotorEx> motors) {
        if (motors.isEmpty()) {
            throw new IllegalArgumentException("List of motors cannot be empty");
        }
        this.motors = motors;
    }

    public static DcMotorEx of(HardwareMap map, List<Pair<String, Direction>> motorInfos) {
        if (motorInfos.isEmpty()) {
            throw new IllegalArgumentException("List of motors cannot be empty");
        }
        if (motorInfos.size() > 1) {
            return new DcMotorsEx(motorInfos.stream().map(motorInfo -> {
                DcMotorEx motor = (DcMotorEx) map.get(DcMotor.class, motorInfo.first);
                motor.setDirection(motorInfo.second);
                return motor;
            }).collect(Collectors.toList()));
        } else {
            DcMotorEx motor = (DcMotorEx) map.get(DcMotor.class, motorInfos.get(0).first);
            motor.setDirection(motorInfos.get(0).second);
            return motor;
        }
    }

    private DcMotorEx motor() {
        return motors.get(0);
    }

    // region HardwareDevice
    @Override
    public Manufacturer getManufacturer() {
        return motor().getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor().getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor().getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor().getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        for (DcMotorEx motor : motors) {
            motor.resetDeviceConfigurationForOpMode();
        }
    }

    @Override
    public void close() {
        for (DcMotorEx motor : motors) {
            motor.close();
        }
    }
    // endregion

    // region DcMotorSimple
    @Override
    public double getPower() {
        return motor().getPower();
    }

    @Override
    public void setPower(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    @Override
    public Direction getDirection() {
        return motor().getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        for (DcMotorEx motor : motors) {
            motor.setDirection(direction);
        }
    }
    // endregion

    // region DcMotor
    @Override
    public RunMode getMode() {
        return motor().getMode();
    }

    @Override
    public void setMode(RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    @Override
    public int getCurrentPosition() {
        return motor().getCurrentPosition();
    }

    @Override
    public boolean isBusy() {
        return motor().isBusy();
    }

    @Override
    public int getTargetPosition() {
        return motor().getTargetPosition();
    }

    @Override
    public void setTargetPosition(int position) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(position);
        }
    }

    @Override
    public boolean getPowerFloat() {
        return motor().getPowerFloat();
    }

    @SuppressWarnings("deprecation")
    @Override
    public void setPowerFloat() {
        for (DcMotorEx motor : motors) {
            motor.setPowerFloat();
        }
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor().getZeroPowerBehavior();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public int getPortNumber() {
        return motor().getPortNumber();
    }

    @Override
    public DcMotorController getController() {
        return motor().getController();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        for (DcMotorEx motor : motors) {
            motor.setMotorType(motorType);
        }
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor().getMotorType();
    }
    // endregion

    // region DcMotorEx
    @Override
    public void setMotorEnable() {
        for (DcMotorEx motor : motors) {
            motor.setMotorEnable();
        }
    }

    @Override
    public void setMotorDisable() {
        for (DcMotorEx motor : motors) {
            motor.setMotorDisable();
        }
    }

    @Override
    public boolean isMotorEnabled() {
        return motor().isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(angularRate);
        }
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(angularRate, unit);
        }
    }

    @Override
    public double getVelocity() {
        return motor().getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return motor().getVelocity(unit);
    }

    @SuppressWarnings("deprecation")
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDCoefficients(mode, pidCoefficients);
        }
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        for (DcMotorEx motor : motors) {
            motor.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        for (DcMotorEx motor : motors) {
            motor.setPositionPIDFCoefficients(p);
        }
    }

    @SuppressWarnings("deprecation")
    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motor().getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motor().getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPositionTolerance(tolerance);
        }
    }

    @Override
    public int getTargetPositionTolerance() {
        return motor().getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return motor().getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor().getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        for (DcMotorEx motor : motors) {
            motor.setCurrentAlert(current, unit);
        }
    }

    @Override
    public boolean isOverCurrent() {
        return motor().isOverCurrent();
    }
    // endregion
}
