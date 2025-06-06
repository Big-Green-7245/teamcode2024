package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.ServoToggle;
import org.firstinspires.ftc.teamcode.modules.motor.RunToPosition;
import org.firstinspires.ftc.teamcode.util.EncoderConstants;

import java.lang.Math;
import java.util.List;
import java.util.function.BooleanSupplier;

public class AutoHelper {
    public static final double OUTPUT_SLIDE_ENCODER = EncoderConstants.YELLOW_JACKET_435.getPulsesPerRevolution() / 1.5;

    static final Pose2d BASKET_INITIAL_POSE = new Pose2d(35.5, 61.5, 3 * Math.PI / 2);
    public static final Pose2d BASKET_POSE = new Pose2d(55, 55, 5 * Math.PI / 4);
    private static final Pose2d SAMPLE_1_POSE = new Pose2d(48, 39, 3 * Math.PI / 2); // 48, 25.5
    private static final Pose2d SAMPLE_2_POSE = new Pose2d(51, 37.5, 5 * Math.PI / 3); // 58, 25.5
    private static final Pose2d SAMPLE_3_POSE = new Pose2d(58, 35.5, 7 * Math.PI / 4); // 68, 25.5
    static final Pose2d SAMPLE_25650_POSE = new Pose2d(6, 58.5, 11 * Math.PI / 12); // -18, 65
    static final List<Pose2d> SAMPLE_POSES = List.of(SAMPLE_1_POSE, SAMPLE_2_POSE, SAMPLE_3_POSE);
    static final Pose2d SAMPLE_SUBMERSIBLE_POSE_1 = new Pose2d(24, 10, Math.PI);
    private static final Pose2d SAMPLE_SUBMERSIBLE_POSE_2 = new Pose2d(24, 8, Math.PI);
    static final List<Pose2d> SAMPLE_SUBMERSIBLE_POSES = List.of(SAMPLE_SUBMERSIBLE_POSE_1, SAMPLE_SUBMERSIBLE_POSE_2);
    public static final Pose2d ASCENT_ZONE_POSE_PARKING = new Pose2d(22, 8, 0);
    public static final int BASKET_SLIDE_HIGH = (int) (7.3 * OUTPUT_SLIDE_ENCODER);
    public static final int BASKET_SLIDE_TOLERANCE = (int) (5 * OUTPUT_SLIDE_ENCODER);
    public static final double BASKET_DEPOSIT_TIME = 0.75;

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
    private static final Pose2d SPECIMEN_SUBMERSIBLE_1_POSE = new Pose2d(-6, SUBMERSIBLE_Y, Math.PI / 2);
    private static final Pose2d SPECIMEN_SUBMERSIBLE_2_POSE = new Pose2d(-8, SUBMERSIBLE_Y, Math.PI / 2);
    private static final Pose2d SPECIMEN_SUBMERSIBLE_3_POSE = new Pose2d(-10, SUBMERSIBLE_Y, Math.PI / 2);
    private static final Pose2d SPECIMEN_SUBMERSIBLE_4_POSE = new Pose2d(-12, SUBMERSIBLE_Y, Math.PI / 2);
    static final List<Pose2d> SPECIMEN_SUBMERSIBLE_POSES = List.of(SPECIMEN_SUBMERSIBLE_1_POSE, SPECIMEN_SUBMERSIBLE_2_POSE, SPECIMEN_SUBMERSIBLE_3_POSE, SPECIMEN_SUBMERSIBLE_4_POSE);
    public static final int SPECIMEN_INITIAL_SLIDE_HIGH = (int) (3.52 * OUTPUT_SLIDE_ENCODER);
    public static final int SPECIMEN_SLIDE_HIGH = (int) (3.64 * OUTPUT_SLIDE_ENCODER);

    /**
     * @param returnTolerance This action will return when the motor is within this given tolerance in encoder ticks.
     *                        The motor will still continue to run to position, just that this action will return early.
     *                        Passing 0 will wait for the motor to finish using the default firmware tolerance.
     */
    public static Action moveSlideToPos(RunToPosition slide, int pos, int returnTolerance) {
        return new SequentialAction(
                new InstantAction(() -> slide.startMoveToPos(pos)),
                telemetryPacket -> {
                    slide.tick();
                    if (returnTolerance <= 0) return !slide.isFinished();
                    return Math.abs(slide.getTargetPosition() - slide.getCurrentPosition()) > returnTolerance;
                }
        );
    }

    public static Action retractSlide(RunToPosition slide) {
        return new SequentialAction(
                new InstantAction(slide::startRetraction),
                telemetryPacket -> {
                    slide.tick();
                    return !slide.isFinished();
                }
        );
    }

    public static Action startIntake(ServoToggle intakePivot, Servo activeIntake) {
        return new SequentialAction(
                new InstantAction(() -> {
                    intakePivot.setAction(true);
                    activeIntake.setPosition(1);
                }),
                new SleepAction(0.7)
        );
    }

    public static Action retractIntake(ServoToggle intakeSlide, ServoToggle intakePivot, Servo activeIntake) {
        return new SequentialAction(
                new InstantAction(() -> {
                    intakePivot.setAction(false);
                    intakeSlide.setAction(false);
                }),
                new SleepAction(0.5),
                new InstantAction(() -> activeIntake.setPosition(0.5))
        );
    }

    public static Action transferSample(Servo activeIntake) {
        return new SequentialAction(
                new InstantAction(() -> activeIntake.setPosition(0)),
                new SleepAction(0.5),
                new InstantAction(() -> activeIntake.setPosition(0.5))
        );
    }

    public static Action depositSample(BooleanSupplier outputSlideExtended, ServoToggle outputBox) {
        return new SequentialAction(
                // Wait for the slide to be within a tolerance to the basket height
                telemetryPacket -> !outputSlideExtended.getAsBoolean(),
                // Deposit the sample
                new InstantAction(() -> outputBox.setAction(true)),
                new SleepAction(AutoHelper.BASKET_DEPOSIT_TIME)
        );
    }
}
