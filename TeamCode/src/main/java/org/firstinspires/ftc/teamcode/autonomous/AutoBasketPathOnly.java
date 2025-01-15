package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

/**
 * This runs the path in {@link AutoBasket} without the input and output modules.
 */
@Autonomous(name = "AutoBasketPathOnly", group = "Big Green", preselectTeleOp = "TeleOp")
public class AutoBasketPathOnly extends LinearOpMode {
    // Define attributes
    private static final String PROGRAM_VERSION = "0.1.0";
    private static final double SPEED_MULTIPLIER = 0.9;
    private static final Pose2d INITIAL_POSE = new Pose2d(36, 63, 3 * Math.PI / 2);

    // Declare modules
    private TelemetryWrapper telemetryWrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryWrapper = new TelemetryWrapper(telemetry, 20);
        telemetryWrapper.setLineAndRender(1, "TeleOp v" + PROGRAM_VERSION + "\t Initializing");

        // Initialize robot modules
        MecanumDrive drive = new PinpointDrive(hardwareMap, INITIAL_POSE);

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "TeleOp v" + PROGRAM_VERSION + "\t Press start to start >");
        while (opModeInInit()) {}

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
