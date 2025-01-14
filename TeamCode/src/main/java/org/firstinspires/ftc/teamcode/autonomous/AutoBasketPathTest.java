package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.output.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@Autonomous(name = "AutoBasketPathTest", group = "Big Green", preselectTeleOp = "TeleOp")
public class AutoBasketPathTest extends LinearOpMode {
    // Define attributes
    private static final String PROGRAM_VERSION = "0.1.0";
    private static final double SPEED_MULTIPLIER = 0.9;
    private static final Pose2d INITIAL_POSE = new Pose2d(36, 63, 3 * Math.PI / 2);

    // Declare modules
    private ButtonHelper gp1, gp2;
    private ServoToggle intakeSlide1, intakeSlide2;
    private ServoToggle intakePivot;
    private Servo activeIntake;
    private LinearSlide outputSlide;
    private ServoToggle outputBox;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryWrapper.init(telemetry, 20);
        TelemetryWrapper.setLineAndRender(1, "TeleOp v" + PROGRAM_VERSION + "\t Initializing");

        // Initialize robot modules
        gp1 = new ButtonHelper(gamepad1);
        gp2 = new ButtonHelper(gamepad2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, INITIAL_POSE);
        intakeSlide1 = new ServoToggle();
        intakeSlide2 = new ServoToggle();
        intakePivot = new ServoToggle();
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        outputSlide = new LinearSlide("outputSlide", 0.5, DcMotorSimple.Direction.REVERSE);
        outputBox = new ServoToggle();

        intakeSlide1.init(hardwareMap, "intakeSlide1", 0, 0.2, false);
        intakeSlide2.init(hardwareMap, "intakeSlide2", 0, 0.2, true);
        intakePivot.init(hardwareMap, "intakePivot", 0, 0.66, false);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap, "outputBox", 0, 0.3, false);

        // Wait for start
        TelemetryWrapper.setLineAndRender(1, "TeleOp v" + PROGRAM_VERSION + "\t Press start to start >");
        while (opModeInInit()) {
            outputSlide.tickBeforeStart();
        }

        // Begin autonomous program
        // See BigGreenTest.java for a visualization

        // Start to move to the basket
        Actions.runBlocking(drive.actionBuilder(INITIAL_POSE)
                .splineTo(new Vector2d(36, 36), 5 * Math.PI / 4)
                .build()
        );

        // Move to basket
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(new Pose2d(36, 36, 5 * Math.PI / 4))
                        .splineToConstantHeading(new Vector2d(59, 59), Math.PI / 4)
                        .build()
        ));
        sleep(500);

        // Move to first sample
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(new Pose2d(59, 59, 5 * Math.PI / 4))
                        .setTangent(5 * Math.PI / 4)
                        .splineToSplineHeading(new Pose2d(36, 25, 0), 3 * Math.PI / 2)
                        .build()
        ));

        // Move to basket the second time
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(new Pose2d(36, 25, 0))
                        .setTangent(Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(59, 59, 5 * Math.PI / 4), Math.PI / 4)
                        .build()
        ));
        sleep(500);

        // Move to second sample
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(new Pose2d(59, 59, 5 * Math.PI / 4))
                        .setTangent(5 * Math.PI / 4)
                        .splineToSplineHeading(new Pose2d(46, 25, 0), 3 * Math.PI / 2)
                        .build()
        ));

        // Move to basket the third time
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(new Pose2d(46, 25, 0))
                        .setTangent(Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(59, 59, 5 * Math.PI / 4), Math.PI / 4)
                        .build()
        ));
        sleep(500);

        // Move to third sample
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(new Pose2d(59, 59, 5 * Math.PI / 4))
                        .setTangent(5 * Math.PI / 4)
                        .splineToSplineHeading(new Pose2d(56, 25, 0), 3 * Math.PI / 2)
                        .build()
        ));

        // Move to basket the fourth time
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(new Pose2d(56, 25, 0))
                        .setTangent(Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(59, 59, 5 * Math.PI / 4), Math.PI / 4)
                        .build()
        ));
        sleep(500);
    }
}
