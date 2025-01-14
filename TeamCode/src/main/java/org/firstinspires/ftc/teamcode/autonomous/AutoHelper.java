package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.output.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;

public class AutoHelper {
    static Action moveSlideToPos(LinearSlide slide, int pos) {
        return new SequentialAction(
                telemetryPacket -> {
                    slide.startMoveToPos(pos);
                    return false;
                },
                telemetryPacket -> {
                    slide.tick();
                    return !slide.isFinished();
                }
        );
    }

    static Action retractSlide(LinearSlide slide) {
        return new SequentialAction(
                telemetryPacket -> {
                    slide.startRetraction();
                    return false;
                },
                telemetryPacket -> {
                    slide.tick();
                    return !slide.isFinished();
                }
        );
    }

    static Action intakeSample(ServoToggle intakeSlide1, ServoToggle intakeSlide2, ServoToggle intakePivot, Servo activeIntake) {
        return new SequentialAction(
                telemetryPacket -> {
                    intakePivot.setAction(true);
                    activeIntake.setPosition(1);
                    return false;
                },
                new SleepAction(0.5),
                telemetryPacket -> {
                    intakeSlide1.interpolateAction(0.5);
                    intakeSlide2.interpolateAction(0.5);
                    return false;
                },
                new SleepAction(0.2),
                telemetryPacket -> {
                    activeIntake.setPosition(0.5);
                    intakePivot.setAction(false);
                    intakeSlide1.setAction(false);
                    intakeSlide2.setAction(false);
                    return false;
                },
                new SleepAction(0.5)
        );
    }

    static Action transferSample(Servo activeIntake) {
        return new SequentialAction(
                telemetryPacket -> {
                    activeIntake.setPosition(0);
                    return false;
                },
                new SleepAction(0.5),
                telemetryPacket -> {
                    activeIntake.setPosition(0.5);
                    return false;
                }
        );
    }
}
