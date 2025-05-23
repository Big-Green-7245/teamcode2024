package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.modules.DoubleServoToggle;
import org.firstinspires.ftc.teamcode.modules.ServoToggle;
import org.firstinspires.ftc.teamcode.modules.motor.DoubleLinearSlides;
import org.firstinspires.ftc.teamcode.modules.motor.MotorInfo;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.lang.Math;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@SuppressWarnings("FieldCanBeLocal")
@Autonomous(name = "AutoBasket", group = "Big Green", preselectTeleOp = "TeleOpBasket")
public class AutoBasket extends LinearOpMode {
    private final boolean team25650;
    // Declare modules
    private TelemetryWrapper telemetryWrapper;
    private MecanumDrive drive;
    private ServoToggle intakeSlide;
    private ServoToggle intakePivot;
    private Servo activeIntake;
    private ServoToggle intakeSweeper;
    private DoubleLinearSlides outputSlide;
    private ServoToggle outputBox;
    private ServoToggle specimenClaw;

    public AutoBasket() {
        this(false);
    }

    public AutoBasket(boolean team25650) {
        this.team25650 = team25650;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "Basket Auto \tInitializing");

        // Initialize robot modules
        drive = new PinpointDrive(hardwareMap, AutoHelper.BASKET_INITIAL_POSE);
        intakeSlide = new DoubleServoToggle("intakeSlide", 0, 0.3, Servo.Direction.REVERSE, Servo.Direction.FORWARD);
        intakePivot = new DoubleServoToggle("intakePivot", 0, 0.66, Servo.Direction.FORWARD, Servo.Direction.REVERSE);
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        intakeSweeper = new ServoToggle("intakeSweeper", 0, 0.5, false);
        outputSlide = new DoubleLinearSlides(
                List.of(new MotorInfo("outputSlideLeft", DcMotorSimple.Direction.REVERSE), new MotorInfo("outputSlideLeft2", DcMotorSimple.Direction.FORWARD)),
                List.of(new MotorInfo("outputSlideRight", DcMotorSimple.Direction.FORWARD), new MotorInfo("outputSlideRight2", DcMotorSimple.Direction.REVERSE)),
                1, (int) (0.1 * AutoHelper.OUTPUT_SLIDE_ENCODER), Integer.MAX_VALUE
        );
        outputBox = new ServoToggle("outputBox", 0, 0.4, true);
        specimenClaw = new ServoToggle("specimenClaw", 0, 0.2, false);

        intakeSlide.init(hardwareMap);
        intakePivot.init(hardwareMap);
        intakeSweeper.init(hardwareMap);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap);
        specimenClaw.init(hardwareMap);

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "Basket Auto \tPress start to start >");
        while (opModeInInit()) {
            outputSlide.tickBeforeStart();
        }

        // Begin autonomous program
        // See AutoBasketPathTest.java for a visualization

        // Move to basket and deposit the preload sample
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_INITIAL_POSE)
                        .setTangent(3 * Math.PI / 2)
                        .beforeEndDisp(1, AutoHelper.depositSample(() -> true, outputBox))
                        .splineToLinearHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build(),
                AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH, AutoHelper.BASKET_SLIDE_TOLERANCE)
        ));

        MecanumDrive.PARAMS.maxWheelVel = 50;
        MecanumDrive.PARAMS.minProfileAccel = -30;
        MecanumDrive.PARAMS.maxProfileAccel = 50;
        for (Pose2d samplePose : team25650 ? Stream.concat(AutoHelper.SAMPLE_POSES.stream(), Stream.of(AutoHelper.SAMPLE_25650_POSE)).collect(Collectors.toList()) : AutoHelper.SAMPLE_POSES) {
            AtomicBoolean outputSlideExtended = new AtomicBoolean(false);
            Actions.runBlocking(drive.actionBuilder(AutoHelper.BASKET_POSE)
                    // Move to sample while resetting output box and retracting slides
                    .setTangent(5 * Math.PI / 4)
                    .afterTime(0, new ParallelAction(
                            new InstantAction(() -> outputBox.setAction(false)),
                            AutoHelper.retractSlide(outputSlide),
                            AutoHelper.startIntake(intakePivot, activeIntake),
                            new InstantAction(samplePose == AutoHelper.SAMPLE_25650_POSE ? () -> intakeSlide.setPosition(0.5) : () -> {})
                    ))
                    .beforeEndDisp(1, () -> intakeSlide.setAction(true))
                    .splineTo(samplePose.position, samplePose.heading)
                    // Intake the sample and move to basket
                    .setTangent(samplePose.heading.plus(Math.PI))
                    .setReversed(true)
                    .afterTime(0.2, new SequentialAction(
                            AutoHelper.retractIntake(intakeSlide, intakePivot, activeIntake),
                            new SleepAction(0.1),
                            AutoHelper.transferSample(activeIntake),
                            AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH, AutoHelper.BASKET_SLIDE_TOLERANCE),
                            new InstantAction(() -> outputSlideExtended.set(true))
                    ))
                    // When the robot is within one inch to the basket, deposit the sample
                    .beforeEndDisp(1, AutoHelper.depositSample(outputSlideExtended::get, outputBox))
                    .splineTo(AutoHelper.BASKET_POSE.position, Math.PI / 4)
                    .build()
            );
        }
        MecanumDrive.PARAMS.maxWheelVel = 70;
        MecanumDrive.PARAMS.minProfileAccel = -50;
        MecanumDrive.PARAMS.maxProfileAccel = 70;

        for (Pose2d submersiblePose : team25650 ? List.of(AutoHelper.SAMPLE_SUBMERSIBLE_POSE_1) : AutoHelper.SAMPLE_SUBMERSIBLE_POSES) {
            // Move to ascent zone while resetting output box and retracting slides
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(AutoHelper.BASKET_POSE)
                            .setTangent(5 * Math.PI / 4)
                            .beforeEndDisp(1, () -> intakeSweeper.setAction(true))
                            .splineTo(submersiblePose.position, Math.PI)
                            .build(),
                    new InstantAction(() -> outputBox.setAction(false)),
                    AutoHelper.retractSlide(outputSlide),
                    new SequentialAction(
                            new SleepAction(1),
                            new InstantAction(() -> intakeSlide.setPosition(0.3))
                    )
            ));

            // Intake a sample and move to basket
            AtomicBoolean outputSlideExtended = new AtomicBoolean(false);
            Actions.runBlocking(AutoHelper.startIntake(intakePivot, activeIntake));
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new SleepAction(0.5),
                            drive.actionBuilder(submersiblePose)
                                    .setTangent(0)
                                    .setReversed(true)
                                    .beforeEndDisp(1, AutoHelper.depositSample(outputSlideExtended::get, outputBox))
                                    .splineTo(AutoHelper.BASKET_POSE.position, Math.PI / 4)
                                    .build()
                    ),
                    new SequentialAction(
                            new InstantAction(() -> intakeSlide.setPosition(1)),
                            new SleepAction(0.5),
                            AutoHelper.retractIntake(intakeSlide, intakePivot, activeIntake),
                            new InstantAction(() -> intakeSweeper.setAction(false)),
                            new SleepAction(0.1),
                            AutoHelper.transferSample(activeIntake),
                            AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH, AutoHelper.BASKET_SLIDE_TOLERANCE),
                            new InstantAction(() -> outputSlideExtended.set(true))
                    )
            ));
        }

        // Move to ascent zone while resetting output box and retracting slides
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineToLinearHeading(AutoHelper.ASCENT_ZONE_POSE_PARKING, Math.PI)
                        .build(),
                new InstantAction(() -> outputBox.setAction(false)),
                AutoHelper.moveSlideToPos(outputSlide, (int) (3.5 * AutoHelper.OUTPUT_SLIDE_ENCODER), 0)
        ));
    }
}
