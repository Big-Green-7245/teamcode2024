package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.modules.DoubleLinearSlides;
import org.firstinspires.ftc.teamcode.modules.DoubleServoToggle;
import org.firstinspires.ftc.teamcode.modules.ServoToggle;
import org.firstinspires.ftc.teamcode.modules.TwoRunToPositionMotors;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.lang.Math;

@Autonomous(name = "AutoBasket", group = "Big Green", preselectTeleOp = "TeleOpBasket")
public class AutoBasket extends LinearOpMode {
    // Declare modules
    private TelemetryWrapper telemetryWrapper;
    private MecanumDrive drive;
    private ServoToggle intakeSlide;
    private ServoToggle intakePivot;
    private Servo activeIntake;
    private DoubleLinearSlides outputSlide;
    private ServoToggle outputBox;
    private ServoToggle specimenClaw;
    private TwoRunToPositionMotors hanging;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "Basket Auto \tInitializing");

        // Initialize robot modules
        drive = new PinpointDrive(hardwareMap, AutoHelper.BASKET_INITIAL_POSE);
        intakeSlide = new DoubleServoToggle("intakeSlide", 0, 0.3, Servo.Direction.REVERSE, Servo.Direction.FORWARD);
        intakePivot = new DoubleServoToggle("intakePivot", 0, 0.66, Servo.Direction.FORWARD, Servo.Direction.REVERSE);
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        outputSlide = new DoubleLinearSlides("outputSlide", 1, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        outputBox = new ServoToggle("outputBox", 0, 0.4, true);
        specimenClaw = new ServoToggle("specimenClaw", 0, 0.2, false);
        hanging = new TwoRunToPositionMotors("hangingMotor", 1, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);

        intakeSlide.init(hardwareMap);
        intakePivot.init(hardwareMap);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap);
        specimenClaw.init(hardwareMap);
        hanging.init(hardwareMap);

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
                        .splineToLinearHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build(),
                AutoHelper.moveSlideToPos(outputSlide, hanging, AutoHelper.BASKET_SLIDE_HIGH)
        ));
        outputBox.setAction(true);
        sleep(500);

        for (Pose2d samplePose : AutoHelper.SAMPLE_POSES) {
            // Move to sample while resetting output box and retracting slides
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(AutoHelper.BASKET_POSE)
                            .setTangent(5 * Math.PI / 4)
                            .splineToSplineHeading(samplePose, 3 * Math.PI / 2)
                            .build(),
                    new InstantAction(() -> outputBox.setAction(false)),
                    AutoHelper.retractSlide(outputSlide, hanging),
                    new SequentialAction(
                            new SleepAction(1),
                            AutoHelper.startIntake(intakePivot, activeIntake)
                    )
            ));

            // Intake the sample and move to basket
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new SleepAction(0.5),
                            drive.actionBuilder(samplePose)
                                    .setTangent(Math.PI / 2)
                                    .splineToSplineHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                                    .build()
                    ),
                    new SequentialAction(
                            AutoHelper.intakeSample(intakeSlide, intakePivot, activeIntake),
                            new SleepAction(0.5),
                            AutoHelper.transferSample(activeIntake),
                            AutoHelper.moveSlideToPos(outputSlide, hanging, AutoHelper.BASKET_SLIDE_HIGH)
                    )
            ));

            // Deposit the sample
            outputBox.setAction(true);
            sleep(500);
        }

        // Move to ascent zone while resetting output box and retracting slides
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineTo(AutoHelper.ASCENT_ZONE_POSE.position, Math.PI)
                        .build(),
                new InstantAction(() -> outputBox.setAction(false)),
                AutoHelper.retractSlide(outputSlide, hanging),
                new SequentialAction(
                        new SleepAction(1),
                        new InstantAction(() -> intakeSlide.setPosition(0.4))
                )
        ));

        // Intake a sample and move to basket
        Actions.runBlocking(AutoHelper.startIntake(intakePivot, activeIntake));
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new SleepAction(0.5),
                        drive.actionBuilder(AutoHelper.ASCENT_ZONE_POSE)
                                .setTangent(0)
                                .splineToLinearHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                                .build()
                ),
                new SequentialAction(
                        AutoHelper.intakeSample(intakeSlide, intakePivot, activeIntake),
                        new SleepAction(0.5),
                        AutoHelper.transferSample(activeIntake),
                        AutoHelper.moveSlideToPos(outputSlide, hanging, AutoHelper.BASKET_SLIDE_HIGH)
                )
        ));

        // Deposit the sample
        outputBox.setAction(true);
        sleep(500);

        // Move to ascent zone while resetting output box and retracting slides
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineTo(AutoHelper.ASCENT_ZONE_POSE.position, Math.PI)
                        .build(),
                new InstantAction(() -> outputBox.setAction(false)),
                AutoHelper.retractSlide(outputSlide, hanging)
        ));
    }
}
