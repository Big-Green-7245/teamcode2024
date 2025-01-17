package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.output.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;

import java.lang.Math;
import java.util.List;

public class AutoHelper {
    static final Pose2d BASKET_INITIAL_POSE = new Pose2d(36, 61, 3 * Math.PI / 2);
    static final Pose2d BASKET_POSE = new Pose2d(54, 54, 5 * Math.PI / 4);
    static final Pose2d SAMPLE_1_POSE = new Pose2d(34, 26, 0);
    static final Pose2d SAMPLE_2_POSE = new Pose2d(44, 26, 0);
    static final Pose2d SAMPLE_3_POSE = new Pose2d(54, 26, 0);
    static final List<Pose2d> SAMPLE_POSES = List.of(
            SAMPLE_1_POSE,
            SAMPLE_2_POSE,
            SAMPLE_3_POSE
    );
    static final Pose2d ASCENT_ZONE_POSE = new Pose2d(24, 12, Math.PI);
    public static final int BASKET_SLIDE_HIGH = 2800;

    static final Pose2d SPECIMEN_INITIAL_POSE = new Pose2d(-12, 63, Math.PI / 2);
    static final Pose2d INITIAL_SUBMERSIBLE_POSE = new Pose2d(-6, 34.5, Math.PI / 2);
    static final int OBSERVATION_ZONE_Y = 48;
    static final Pose2d OBSERVATION_ZONE_POSE = new Pose2d(-36, 60, 3 * Math.PI / 2);
    static final Pose2d SUBMERSIBLE_POSE = new Pose2d(-8, 34.5, Math.PI / 2);
    static final Pose2d SUBMERSIBLE_SIDE_POSE = new Pose2d(-4, 34.5, Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_1_POSE = new Pose2d(-42, 14, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_1_DEPOSIT_POSE = new Pose2d(-44, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_2_POSE = new Pose2d(-52, 14, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_2_DEPOSIT_POSE = new Pose2d(-54, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_3_POSE = new Pose2d(-62, 14, 3 * Math.PI / 2);
    static final Pose2d SPECIMEN_SAMPLE_3_DEPOSIT_POSE = new Pose2d(-62, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    static final int SPECIMEN_SLIDE_HIGH = 1500;

    static Action moveSlideToPos(LinearSlide slide, int pos) {
        return new SequentialAction(
                new InstantAction(() -> slide.startMoveToPos(pos)),
                telemetryPacket -> {
                    slide.tick();
                    return !slide.isFinished();
                }
        );
    }

    static Action retractSlide(LinearSlide slide) {
        return new SequentialAction(
                new InstantAction(slide::startRetraction),
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
                new InstantAction(() -> {
                    intakeSlide1.setPosition(0.5);
                    intakeSlide2.setPosition(0.5);
                }),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    intakePivot.setAction(false);
                    intakeSlide1.setAction(false);
                    intakeSlide2.setAction(false);
                }),
                new SleepAction(0.5),
                new InstantAction(() -> activeIntake.setPosition(0.5))
        );
    }

    static Action transferSample(Servo activeIntake) {
        return new SequentialAction(
                new SleepAction(0.5),
                new InstantAction(() -> activeIntake.setPosition(0)),
                new SleepAction(0.5),
                new InstantAction(() -> activeIntake.setPosition(0.5))
        );
    }
}
