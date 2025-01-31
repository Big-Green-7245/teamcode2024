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
import org.firstinspires.ftc.teamcode.modules.ServoToggle;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.lang.Math;

@Autonomous(name = "AutoSpecimen", group = "Big Green", preselectTeleOp = "TeleOpSpecimen")
public class AutoSpecimen extends LinearOpMode {
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
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "Specimen Auto\t Initializing");

        // Initialize robot modules
        drive = new PinpointDrive(hardwareMap, AutoHelper.SPECIMEN_INITIAL_POSE);
        intakeSlide1 = new ServoToggle("intakeSlide1", 0, 0.25, true);
        intakeSlide2 = new ServoToggle("intakeSlide2", 0, 0.25, false);
        intakePivot = new ServoToggle("intakePivot", 0, 0.66, false);
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        outputSlide = new DoubleLinearSlides("outputSlide", 1, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        outputBox = new ServoToggle("outputBox", 0, 0.4, true);
        specimenClaw = new ServoToggle("specimenClaw", 0, 0.2, false);

        intakeSlide1.init(hardwareMap);
        intakeSlide2.init(hardwareMap);
        intakePivot.init(hardwareMap);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap);
        specimenClaw.init(hardwareMap);

        specimenClaw.setAction(true);

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "Specimen Auto\t Press start to start >");
        while (opModeInInit()) {
            outputSlide.tickBeforeStart();
        }

        // Begin autonomous program
        // See AutoSpecimenPathTest.java for a visualization

        // Move to submersible and deposit the preload specimen
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.SPECIMEN_INITIAL_POSE)
                        .setTangent(3 * Math.PI / 2)
                        .splineToConstantHeading(AutoHelper.INITIAL_SUBMERSIBLE_POSE.position, 3 * Math.PI / 2)
                        .build(),
                AutoHelper.moveSlideToPos(outputSlide, AutoHelper.SPECIMEN_SLIDE_HIGH)
        ));

        // Move to first sample while resetting specimen claw and retracting slides
        Actions.runBlocking(new ParallelAction(
                // This big drive spline takes much longer than the other actions in this parallel action, so don't get confused.
                new SequentialAction(
                        new SleepAction(0.1),
                        drive.actionBuilder(AutoHelper.INITIAL_SUBMERSIBLE_POSE)
                                // Move to first sample
                                .setTangent(3 * Math.PI / 4)
                                .splineToLinearHeading(AutoHelper.SPECIMEN_SAMPLE_1_POSE, Math.PI)
                                .build()
                ),

                // Start retracting the output slides while we start moving
                new SequentialAction(
                        AutoHelper.retractSlide(outputSlide),
                        AutoHelper.startIntake(intakePivot, activeIntake),
                        new InstantAction(() -> {
                            intakeSlide1.setPosition(AutoHelper.SPECIMEN_INTAKE_SLIDE_PREPARE);
                            intakeSlide2.setPosition(AutoHelper.SPECIMEN_INTAKE_SLIDE_PREPARE);
                        })
                ),

                // Open specimen claw after sleeping for a short time
                new SequentialAction(
                        new SleepAction(0.5),
                        new InstantAction(() -> specimenClaw.setAction(false))
                )
        ));

        for (int i = 1; i < AutoHelper.SPECIMEN_SAMPLE_POSES.size(); i++) {
            // Intake and turn to observation zone
            TrajectoryActionBuilder builder = drive.actionBuilder(AutoHelper.SPECIMEN_SAMPLE_POSES.get(i - 1))
                    .turnTo(3 * Math.PI / 4);
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new SleepAction(0.5),
                            builder.build()
                    ),
                    new InstantAction(() -> {
                        activeIntake.setPosition(1);
                        intakeSlide1.setPosition(AutoHelper.SPECIMEN_INTAKE_SLIDE_EXTEND);
                        intakeSlide2.setPosition(AutoHelper.SPECIMEN_INTAKE_SLIDE_EXTEND);
                    })
            ));

            // Spit out the sample and move to the next sample while starting intake
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new SleepAction(0.3),
                            builder.fresh()
                                    .setTangent(Math.PI)
                                    .splineToLinearHeading(AutoHelper.SPECIMEN_SAMPLE_POSES.get(i), Math.PI)
                                    .build()
                    ),
                    new SequentialAction(
                            AutoHelper.transferSample(activeIntake),
                            new InstantAction(() -> {
                                intakeSlide1.setPosition(AutoHelper.SPECIMEN_INTAKE_SLIDE_PREPARE);
                                intakeSlide2.setPosition(AutoHelper.SPECIMEN_INTAKE_SLIDE_PREPARE);
                            })
                    )
            ));
        }

        // Intake and turn to observation zone
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new SleepAction(0.5),
                        drive.actionBuilder(AutoHelper.SPECIMEN_SAMPLE_3_POSE)
                                .setTangent(Math.PI / 4)
                                .splineToLinearHeading(AutoHelper.SPECIMEN_SAMPLE_3_DEPOSIT_POSE, Math.PI / 4)
                                .build()
                ),
                new SequentialAction(
                        new InstantAction(() -> {
                            activeIntake.setPosition(1);
                            intakeSlide1.setPosition(AutoHelper.SPECIMEN_INTAKE_SLIDE_EXTEND);
                            intakeSlide2.setPosition(AutoHelper.SPECIMEN_INTAKE_SLIDE_EXTEND);
                        }),
                        new SleepAction(0.5),
                        new InstantAction(() -> {
                            intakeSlide1.setAction(false);
                            intakeSlide2.setAction(false);
                        })
                )
        ));

        // Move to observation zone to pick up specimen
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new SleepAction(0.3),
                        drive.actionBuilder(AutoHelper.SPECIMEN_SAMPLE_3_DEPOSIT_POSE)
                                .setTangent(Math.PI / 4)
                                .splineToLinearHeading(AutoHelper.OBSERVATION_ZONE_POSE, Math.PI / 2)
                                .build()
                ),
                new SequentialAction(
                        AutoHelper.transferSample(activeIntake),
                        new InstantAction(() -> intakePivot.setAction(false))
                )
        ));

        for (Pose2d submersiblePose : AutoHelper.SUBMERSIBLE_POSES) {
            // Pick up specimen and move to submersible
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new SleepAction(0.2),
                            drive.actionBuilder(AutoHelper.OBSERVATION_ZONE_POSE)
                                    .setTangent(7 * Math.PI / 4)
                                    .splineToLinearHeading(submersiblePose, 7 * Math.PI / 4)
                                    .build()
                    ),
                    new SequentialAction(
                            new InstantAction(() -> specimenClaw.setAction(true)),
                            new SleepAction(0.1),
                            AutoHelper.moveSlideToPos(outputSlide, AutoHelper.SPECIMEN_SLIDE_HIGH)
                    )
            ));

            // Deposit the specimen and move to observation zone
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new SleepAction(0.1),
                            drive.actionBuilder(submersiblePose)
                                    .setTangent(3 * Math.PI / 4)
                                    .splineToLinearHeading(AutoHelper.OBSERVATION_ZONE_POSE, Math.PI / 2)
                                    .build()
                    ),
                    AutoHelper.retractSlide(outputSlide),
                    new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> specimenClaw.setAction(false))
                    )
            ));
        }
    }
}
