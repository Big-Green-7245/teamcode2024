package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.output.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;

public class AutoHelper {
    static final Pose2d BASKET_INITIAL_POSE = new Pose2d(36, 61, 3 * Math.PI / 2);
    static final Pose2d BASKET_POSE = new Pose2d(54, 54, 5 * Math.PI / 4);
    static final Pose2d SAMPLE_1_POSE = new Pose2d(36, 26, 0);
    static final Pose2d SAMPLE_2_POSE = new Pose2d(46, 26, 0);
    static final Pose2d SAMPLE_3_POSE = new Pose2d(56, 26, 0);
    public static final int BASKET_SLIDE_HIGH = 2700;

    static final Pose2d SPECIMEN_INITIAL_POSE = new Pose2d(-12, 61, Math.PI / 2);
    static final Pose2d INITIAL_SUBMERSIBLE_POSE = new Pose2d(-6, 36, Math.PI / 2);
    static final int OBSERVATION_ZONE_Y = 48;
    static final Pose2d OBSERVATION_ZONE_POSE = new Pose2d(-36, 60, 3 * Math.PI / 2);
    static final Pose2d SUBMERSIBLE_POSE = new Pose2d(-8, 36, Math.PI / 2);
    static final Pose2d SUBMERSIBLE_SIDE_POSE = new Pose2d(-4, 36, Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_1_POSE = new Pose2d(-41, 14, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_1_DEPOSIT_POSE = new Pose2d(-42, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_2_POSE = new Pose2d(-51, 14, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_2_DEPOSIT_POSE = new Pose2d(-52, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_3_POSE = new Pose2d(-61, 14, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_3_DEPOSIT_POSE = new Pose2d(-61, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    static final int SPECIMEN_SLIDE_HIGH = 1900;

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
                    telemetryPacket.put("outputSlideCurrentPos", slide.getCurrentPosition());
                    telemetryPacket.put("outputSlideTargetPos", slide.getTargetPosition());
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
                    intakeSlide1.setAction(true);
                    intakeSlide2.setAction(true);
                    return false;
                },
                new SleepAction(0.5),
                telemetryPacket -> {
                    intakePivot.setAction(false);
                    intakeSlide1.setAction(false);
                    intakeSlide2.setAction(false);
                    return false;
                },
                new SleepAction(0.5),
                telemetryPacket -> {
                    activeIntake.setPosition(0.5);
                    return false;
                }
        );
    }

    static Action transferSample(Servo activeIntake) {
        return new SequentialAction(
                new SleepAction(0.5),
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
