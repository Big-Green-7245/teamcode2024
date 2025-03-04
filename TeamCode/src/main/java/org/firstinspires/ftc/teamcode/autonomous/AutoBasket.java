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

@SuppressWarnings("FieldCanBeLocal")
@Autonomous(name = "AutoBasket", group = "Big Green", preselectTeleOp = "TeleOpBasket")
public class AutoBasket extends LinearOpMode {
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
                        .beforeEndDisp(0.5, AutoHelper.depositSample(outputSlide, outputBox))
                        .splineToLinearHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build(),
                AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH, AutoHelper.BASKET_SLIDE_TOLERANCE)
        ));

        MecanumDrive.PARAMS.maxWheelVel = 50;
        MecanumDrive.PARAMS.minProfileAccel = -30;
        MecanumDrive.PARAMS.maxProfileAccel = 50;
        for (Pose2d samplePose : AutoHelper.SAMPLE_POSES) {
            Actions.runBlocking(drive.actionBuilder(AutoHelper.BASKET_POSE)
                    // Move to sample while resetting output box and retracting slides
                    .setTangent(5 * Math.PI / 4)
                    .afterTime(0, new ParallelAction(
                            new InstantAction(() -> outputBox.setAction(false)),
                            AutoHelper.retractSlide(outputSlide),
                            AutoHelper.startIntake(intakePivot, activeIntake)
                    ))
                    .beforeEndDisp(0.5, new InstantAction(() -> intakeSlide.setAction(true)))
                    .splineToLinearHeading(samplePose, samplePose.heading)
                    // Intake the sample and move to basket
                    .setTangent(samplePose.heading.plus(Math.PI))
                    .afterTime(0.5, new SequentialAction(
                            AutoHelper.retractIntake(intakeSlide, intakePivot, activeIntake),
                            new SleepAction(0.1),
                            AutoHelper.transferSample(activeIntake),
                            AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH, AutoHelper.BASKET_SLIDE_TOLERANCE)
                    ))
                    // When the robot is within 0.5 inches to the basket, deposit the sample
                    .beforeEndDisp(0.5, AutoHelper.depositSample(outputSlide, outputBox))
                    .splineToLinearHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                    .build()
            );
        }
        MecanumDrive.PARAMS.maxWheelVel = 70;
        MecanumDrive.PARAMS.minProfileAccel = -50;
        MecanumDrive.PARAMS.maxProfileAccel = 70;

        for (Pose2d submersiblePose : AutoHelper.SAMPLE_SUBMERSIBLE_POSES) {
            // Move to ascent zone while resetting output box and retracting slides
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(AutoHelper.BASKET_POSE)
                            .setTangent(5 * Math.PI / 4)
                            .beforeEndDisp(0.5, new InstantAction(() -> intakeSweeper.setAction(true)))
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
            Actions.runBlocking(AutoHelper.startIntake(intakePivot, activeIntake));
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new SleepAction(0.5),
                            drive.actionBuilder(submersiblePose)
                                    .setTangent(0)
                                    .setReversed(true)
                                    .beforeEndDisp(0.5, AutoHelper.depositSample(outputSlide, outputBox))
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
                            AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH, AutoHelper.BASKET_SLIDE_TOLERANCE)
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
