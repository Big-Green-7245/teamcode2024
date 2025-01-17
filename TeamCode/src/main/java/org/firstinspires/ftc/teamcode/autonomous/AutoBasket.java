package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.modules.output.DoubleLinearSlides;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@Autonomous(name = "AutoBasket", group = "Big Green", preselectTeleOp = "TeleOp")
public class AutoBasket extends LinearOpMode {
    // Define attributes
    private static final String PROGRAM_VERSION = "0.1.0";
    private static final double SPEED_MULTIPLIER = 0.9;

    // Declare modules
    private TelemetryWrapper telemetryWrapper;
    private MecanumDrive drive;
    private ServoToggle intakeSlide1, intakeSlide2;
    private ServoToggle intakePivot;
    private Servo activeIntake;
    private DoubleLinearSlides outputSlide;
    private ServoToggle outputBox;
    private ServoToggle specimenClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryWrapper = new TelemetryWrapper(telemetry, 20);
        telemetryWrapper.setLineAndRender(1, "Basket Auto v" + PROGRAM_VERSION + "\t Initializing");

        // Initialize robot modules
        drive = new PinpointDrive(hardwareMap, AutoHelper.BASKET_INITIAL_POSE);
        intakeSlide1 = new ServoToggle();
        intakeSlide2 = new ServoToggle();
        intakePivot = new ServoToggle();
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        outputSlide = new DoubleLinearSlides("outputSlide", 1, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        outputBox = new ServoToggle();
        specimenClaw = new ServoToggle();

        intakeSlide1.init(hardwareMap, "intakeSlide1", 0, 0.2, true);
        intakeSlide2.init(hardwareMap, "intakeSlide2", 0, 0.2, false);
        intakePivot.init(hardwareMap, "intakePivot", 0, 0.66, false);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap, "outputBox", 0, 0.4, true);
        specimenClaw.init(hardwareMap, "specimenClaw", 0, 0.2, false);

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "Basket Auto v" + PROGRAM_VERSION + "\t Press start to start >");
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
                AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH)
        ));
        outputBox.setAction(true);
        sleep(500);

        // Move to first sample while resetting output box and retracting slides
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineToSplineHeading(AutoHelper.SAMPLE_1_POSE, 3 * Math.PI / 2)
                        .build(),
                new InstantAction(() -> outputBox.setAction(false)),
                AutoHelper.retractSlide(outputSlide)
        ));

        // Intake the first sample
        Actions.runBlocking(AutoHelper.intakeSample(intakeSlide1, intakeSlide2, intakePivot, activeIntake));

        // Move to basket the second time and deposit
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.SAMPLE_1_POSE)
                        .setTangent(Math.PI / 2)
                        .splineToSplineHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build(),
                new SequentialAction(
                        AutoHelper.transferSample(activeIntake),
                        AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH)
                )
        ));
        outputBox.setAction(true);
        sleep(500);

        // Move to second sample while resetting output box and retracting slides
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineToSplineHeading(AutoHelper.SAMPLE_2_POSE, 3 * Math.PI / 2)
                        .build(),
                new InstantAction(() -> outputBox.setAction(false)),
                AutoHelper.retractSlide(outputSlide)
        ));

        // Intake the second sample
        Actions.runBlocking(AutoHelper.intakeSample(intakeSlide1, intakeSlide2, intakePivot, activeIntake));

        // Move to basket the third time and deposit
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.SAMPLE_2_POSE)
                        .setTangent(Math.PI / 2)
                        .splineToSplineHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build(),
                new SequentialAction(
                        AutoHelper.transferSample(activeIntake),
                        AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH)
                )
        ));
        outputBox.setAction(true);
        sleep(500);

        // Move to third sample while resetting output box and retracting slides
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineToSplineHeading(AutoHelper.SAMPLE_3_POSE, 3 * Math.PI / 2)
                        .build(),
                new InstantAction(() -> outputBox.setAction(false)),
                AutoHelper.retractSlide(outputSlide)
        ));

        // Intake the third sample
        Actions.runBlocking(AutoHelper.intakeSample(intakeSlide1, intakeSlide2, intakePivot, activeIntake));

        // Move to basket the fourth time and deposit
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.SAMPLE_3_POSE)
                        .setTangent(Math.PI / 2)
                        .splineToSplineHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build(),
                new SequentialAction(
                        AutoHelper.transferSample(activeIntake),
                        AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH)
                )
        ));
        outputBox.setAction(true);
        sleep(500);

        // Move to ascent zone while resetting output box and retracting slides
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineTo(AutoHelper.ASCENT_ZONE_POSE.position, Math.PI)
                        .build(),
                new InstantAction(() -> outputBox.setAction(false)),
                AutoHelper.retractSlide(outputSlide)
        ));
    }
}
