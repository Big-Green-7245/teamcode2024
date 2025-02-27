package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@SuppressWarnings("FieldCanBeLocal")
@Autonomous(name = "AutoSpecimenPathOnly", group = "Big Green", preselectTeleOp = "TeleOpSpecimen")
public class AutoSpecimenPathOnly extends LinearOpMode {
    private TelemetryWrapper telemetryWrapper;
    private MecanumDrive drive;
    private ServoToggle intakeSlide;
    private ServoToggle intakePivot;
    private DoubleLinearSlides outputSlide;
    private ServoToggle outputBox;
    private ServoToggle specimenClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "Specimen Auto PATH ONLY\t Initializing");

        // Initialize robot modules
        drive = new PinpointDrive(hardwareMap, AutoHelper.SPECIMEN_INITIAL_POSE);
        intakeSlide = new DoubleServoToggle("intakeSlide", 0, 0.3, Servo.Direction.REVERSE, Servo.Direction.FORWARD);
        intakePivot = new DoubleServoToggle("intakePivot", 0, 0.66, Servo.Direction.FORWARD, Servo.Direction.REVERSE);
        outputSlide = new DoubleLinearSlides(
                List.of(Pair.create("outputSlideLeft", DcMotorSimple.Direction.REVERSE), Pair.create("outputSlideLeft2", DcMotorSimple.Direction.FORWARD)),
                List.of(Pair.create("outputSlideRight", DcMotorSimple.Direction.FORWARD), Pair.create("outputSlideRight2", DcMotorSimple.Direction.REVERSE)),
                1, (int) (0.1 * AutoHelper.OUTPUT_SLIDE_ENCODER), Integer.MAX_VALUE
        );
        outputBox = new ServoToggle("outputBox", 0, 0.4, true);
        specimenClaw = new ServoToggle("specimenClaw", 0, 0.2, false);

        intakeSlide.init(hardwareMap);
        intakePivot.init(hardwareMap);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap);
        specimenClaw.init(hardwareMap);

        specimenClaw.setAction(true);

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "Specimen Auto PATH ONLY\t Press start to start >");
        while (opModeInInit()) {
            outputSlide.tickBeforeStart();
        }

        // Begin autonomous program
        // See AutoSpecimenPathTest.java for a visualization

        // Move to submersible and deposit the preload specimen
        Actions.runBlocking(drive.actionBuilder(AutoHelper.SPECIMEN_INITIAL_POSE)
                .setTangent(3 * Math.PI / 2)
                .splineToConstantHeading(AutoHelper.INITIAL_SUBMERSIBLE_POSE.position, 3 * Math.PI / 2)
                .build()
        );

        // Move to first sample while resetting specimen claw and retracting slides
        Actions.runBlocking(drive.actionBuilder(AutoHelper.INITIAL_SUBMERSIBLE_POSE)
                .setTangent(3 * Math.PI / 4)
                .splineToLinearHeading(AutoHelper.SPECIMEN_SAMPLE_1_POSE, Math.PI)
                .build()
        );

        for (int i = 1; i < AutoHelper.SPECIMEN_SAMPLE_POSES.size(); i++) {
            Actions.runBlocking(drive.actionBuilder(AutoHelper.SPECIMEN_SAMPLE_POSES.get(i - 1))
                    // Intake and turn to observation zone
                    .turnTo(3 * Math.PI / 4)
                    // Spit out the sample and move to the next sample while starting intake
                    .setTangent(Math.PI)
                    .splineToLinearHeading(AutoHelper.SPECIMEN_SAMPLE_POSES.get(i), Math.PI)
                    .build()
            );
        }

        Actions.runBlocking(drive.actionBuilder(AutoHelper.SPECIMEN_SAMPLE_3_POSE)
                // Intake and turn to observation zone
                .setTangent(Math.PI / 4)
                .splineToLinearHeading(AutoHelper.SPECIMEN_SAMPLE_3_DEPOSIT_POSE, Math.PI / 4)
                // Move to observation zone to pick up specimen
                .setTangent(Math.PI / 4)
                .splineToLinearHeading(AutoHelper.OBSERVATION_ZONE_POSE, Math.PI / 2)
                .build()
        );

        for (Pose2d submersiblePose : AutoHelper.SPECIMEN_SUBMERSIBLE_POSES) {
            // Pick up specimen and move to submersible
            Actions.runBlocking(drive.actionBuilder(AutoHelper.OBSERVATION_ZONE_POSE)
                    .setTangent(7 * Math.PI / 4)
                    .splineToLinearHeading(submersiblePose, 7 * Math.PI / 4)
                    .build()
            );

            // Deposit the specimen and move to observation zone
            Actions.runBlocking(drive.actionBuilder(submersiblePose)
                    .setTangent(3 * Math.PI / 4)
                    .splineToLinearHeading(AutoHelper.OBSERVATION_ZONE_POSE, Math.PI / 2)
                    .build()
            );
        }
    }
}
