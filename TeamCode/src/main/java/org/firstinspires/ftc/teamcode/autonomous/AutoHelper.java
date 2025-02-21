package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.ServoToggle;
import org.firstinspires.ftc.teamcode.modules.motor.RunToPosition;
import org.firstinspires.ftc.teamcode.util.EncoderConstants;

import java.lang.Math;
import java.util.List;

public class AutoHelper {
    public static final EncoderConstants OUTPUT_SLIDE = EncoderConstants.YELLOW_JACKET_435;

    static final Pose2d BASKET_INITIAL_POSE = new Pose2d(36, 61, 3 * Math.PI / 2);
    static final Pose2d BASKET_POSE = new Pose2d(55, 55, 5 * Math.PI / 4);
    static final Pose2d SAMPLE_1_POSE = new Pose2d(35, 26, 0);
    static final Pose2d SAMPLE_2_POSE = new Pose2d(45, 26, 0);
    static final Pose2d SAMPLE_3_POSE = new Pose2d(55, 26, 0);
    static final List<Pose2d> SAMPLE_POSES = List.of(SAMPLE_1_POSE, SAMPLE_2_POSE, SAMPLE_3_POSE);
    public static final Pose2d ASCENT_ZONE_POSE = new Pose2d(24, 10, Math.PI);
    public static final Pose2d ASCENT_ZONE_POSE_2 = new Pose2d(22, 8, 0);
    public static final int BASKET_SLIDE_HIGH = (int) (7.3 * OUTPUT_SLIDE.getPulsesPerRevolution());

    static final Pose2d SPECIMEN_INITIAL_POSE = new Pose2d(-12, 63, Math.PI / 2);
    static final double SUBMERSIBLE_Y = 34;
    static final Pose2d INITIAL_SUBMERSIBLE_POSE = new Pose2d(-4, SUBMERSIBLE_Y, Math.PI / 2);
    static final int SPECIMEN_SAMPLE_PICKUP_Y = 42;
    static final Pose2d SPECIMEN_SAMPLE_1_POSE = new Pose2d(-32, SPECIMEN_SAMPLE_PICKUP_Y, 5 * Math.PI / 4);
    static final Pose2d SPECIMEN_SAMPLE_2_POSE = new Pose2d(-42, SPECIMEN_SAMPLE_PICKUP_Y, 5 * Math.PI / 4);
    static final Pose2d SPECIMEN_SAMPLE_3_POSE = new Pose2d(-52, SPECIMEN_SAMPLE_PICKUP_Y, 5 * Math.PI / 4);
    static final double SPECIMEN_INTAKE_SLIDE_PREPARE = 0.6;
    static final double SPECIMEN_INTAKE_SLIDE_EXTEND = 0.9;
    static final List<Pose2d> SPECIMEN_SAMPLE_POSES = List.of(SPECIMEN_SAMPLE_1_POSE, SPECIMEN_SAMPLE_2_POSE, SPECIMEN_SAMPLE_3_POSE);
    static final Pose2d SPECIMEN_SAMPLE_3_DEPOSIT_POSE = new Pose2d(-40, 52, 3 * Math.PI / 4);
    public static final Pose2d OBSERVATION_ZONE_POSE = new Pose2d(-36, 61, 3 * Math.PI / 2);
    static final Pose2d SUBMERSIBLE_1_POSE = new Pose2d(-6, SUBMERSIBLE_Y, Math.PI / 2);
    static final Pose2d SUBMERSIBLE_2_POSE = new Pose2d(-8, SUBMERSIBLE_Y, Math.PI / 2);
    static final Pose2d SUBMERSIBLE_3_POSE = new Pose2d(-10, SUBMERSIBLE_Y, Math.PI / 2);
    static final Pose2d SUBMERSIBLE_4_POSE = new Pose2d(-12, SUBMERSIBLE_Y, Math.PI / 2);
    static final List<Pose2d> SUBMERSIBLE_POSES = List.of(SUBMERSIBLE_1_POSE, SUBMERSIBLE_2_POSE, SUBMERSIBLE_3_POSE, SUBMERSIBLE_4_POSE);
    public static final int SPECIMEN_SLIDE_HIGH = (int) (3.64 * OUTPUT_SLIDE.getPulsesPerRevolution());

    /**
     * @param returnTolerance This action will return when the motor is within this given tolerance in encoder ticks.
     *                        The motor will still continue to run to position, just that this action will return early.
     *                        Passing 0 will wait for the motor to finish using the default firmware tolerance.
     */
    static Action moveSlideToPos(RunToPosition slide, int pos, int returnTolerance) {
        return new SequentialAction(
                new InstantAction(() -> slide.startMoveToPos(pos)),
                telemetryPacket -> {
                    slide.tick();
                    if (returnTolerance <= 0) return !slide.isFinished();
                    return Math.abs(slide.getTargetPosition() - slide.getCurrentPosition()) > returnTolerance;
                }
        );
    }

    static Action retractSlide(RunToPosition slide) {
        return new SequentialAction(
                new InstantAction(slide::startRetraction),
                telemetryPacket -> {
                    slide.tick();
                    return !slide.isFinished();
                }
        );
    }

    static Action startIntake(ServoToggle intakePivot, Servo activeIntake) {
        return new SequentialAction(
                new InstantAction(() -> {
                    intakePivot.setAction(true);
                    activeIntake.setPosition(1);
                }),
                new SleepAction(0.7)
        );
    }

    static Action retractIntake(ServoToggle intakeSlide, ServoToggle intakePivot, Servo activeIntake) {
        return new SequentialAction(
                new InstantAction(() -> {
                    intakePivot.setAction(false);
                    intakeSlide.setAction(false);
                }),
                new SleepAction(0.5),
                new InstantAction(() -> activeIntake.setPosition(0.5))
        );
    }

    static Action transferSample(Servo activeIntake) {
        return new SequentialAction(
                new InstantAction(() -> activeIntake.setPosition(0)),
                new SleepAction(0.5),
                new InstantAction(() -> activeIntake.setPosition(0.5))
        );
    }
}
