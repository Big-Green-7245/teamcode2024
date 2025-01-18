package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.modules.output.DoubleLinearSlides;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.lang.Math;

@Autonomous(name = "AutoSpecimen", group = "Big Green", preselectTeleOp = "TeleOp")
public class AutoSpecimen extends LinearOpMode {
    private TelemetryWrapper telemetryWrapper;
    private MecanumDrive drive;
    private ServoToggle intakeSlide1, intakeSlide2;
    private ServoToggle intakePivot;
    private DoubleLinearSlides outputSlide;
    private ServoToggle outputBox;
    private ServoToggle specimenClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "Specimen Auto\t Initializing");

        // Initialize robot modules
        drive = new PinpointDrive(hardwareMap, AutoHelper.SPECIMEN_INITIAL_POSE);
        intakeSlide1 = new ServoToggle("intakeSlide1", 0, 0.2, true);
        intakeSlide2 = new ServoToggle("intakeSlide2", 0, 0.2, false);
        intakePivot = new ServoToggle("intakePivot", 0, 0.66, false);
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

        // Start moving while retracting the output slides; reset specimen claw after a short sleep
        Actions.runBlocking(new ParallelAction(
                // This big drive spline takes much longer than the other actions in this parallel action, so don't get confused.
                new SequentialAction(
                        new SleepAction(0.05),
                        drive.actionBuilder(AutoHelper.INITIAL_SUBMERSIBLE_POSE)
                                // Move to first sample
                                .setTangent(Math.PI / 2)
                                .splineTo(AutoHelper.SPECIMEN_SAMPLE_1_INTERMEDIATE_1.position, 3 * Math.PI / 2)
                                .splineToSplineHeading(AutoHelper.SPECIMEN_SAMPLE_1_INTERMEDIATE_2, 3 * Math.PI / 2)
                                .splineToConstantHeading(AutoHelper.SPECIMEN_SAMPLE_1_POSE.position, Math.PI / 2)
                                // Push first sample to observation zone
                                .splineToConstantHeading(AutoHelper.SPECIMEN_SAMPLE_1_DEPOSIT_POSE.position, Math.PI / 2)
                                // Move to second sample
                                .setTangent(3 * Math.PI / 2)
                                .splineToConstantHeading(AutoHelper.SPECIMEN_SAMPLE_2_POSE.position, Math.PI / 2)
                                // Push second sample to observation zone
                                .splineToConstantHeading(AutoHelper.SPECIMEN_SAMPLE_2_DEPOSIT_POSE.position, Math.PI / 2)
                                // Move to third sample
                                .setTangent(3 * Math.PI / 2)
                                .splineToConstantHeading(AutoHelper.SPECIMEN_SAMPLE_3_POSE.position, Math.PI / 2)
                                // Push third sample to observation zone
                                .splineToConstantHeading(AutoHelper.SPECIMEN_SAMPLE_3_DEPOSIT_POSE.position, Math.PI / 2)
                                // Move to observation zone and pick up second specimen
                                .splineToConstantHeading(AutoHelper.OBSERVATION_ZONE_POSE.position, Math.PI / 2)
                                .build()
                ),

                // Start retracting the output slides while we start moving
                AutoHelper.retractSlide(outputSlide),

                // Open specimen claw after sleeping for a short time
                new SequentialAction(
                        new SleepAction(0.5),
                        new InstantAction(() -> specimenClaw.setAction(false))
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
                            new SleepAction(0.05),
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
