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

    // Declare modules
    private TelemetryWrapper telemetryWrapper;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryWrapper = new TelemetryWrapper(telemetry, 20);
        telemetryWrapper.setLineAndRender(1, "Basket Auto PATH ONLY v" + PROGRAM_VERSION + "\t Initializing");

        // Initialize robot modules
        drive = new PinpointDrive(hardwareMap, AutoHelper.BASKET_INITIAL_POSE);

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "Basket Auto PATH ONLY v" + PROGRAM_VERSION + "\t Press start to start >");
        while (opModeInInit()) {}

        // Begin autonomous program
        // See AutoBasketPathTest.java for a visualization

        // Start to move to the basket
        Actions.runBlocking(drive.actionBuilder(AutoHelper.BASKET_INITIAL_POSE)
                .splineTo(new Vector2d(36, 36), 5 * Math.PI / 4)
                .build()
        );

        // Move to basket
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(new Pose2d(36, 36, 5 * Math.PI / 4))
                        .splineToConstantHeading(AutoHelper.BASKET_POSE.position, Math.PI / 4)
                        .build()
        ));
        sleep(500);

        // Move to first sample
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineToSplineHeading(AutoHelper.SAMPLE_1_POSE, 3 * Math.PI / 2)
                        .build()
        ));

        // Move to basket the second time
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.SAMPLE_1_POSE)
                        .setTangent(Math.PI / 2)
                        .splineToSplineHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build()
        ));
        sleep(500);

        // Move to second sample
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineToSplineHeading(AutoHelper.SAMPLE_2_POSE, 3 * Math.PI / 2)
                        .build()
        ));

        // Move to basket the third time
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.SAMPLE_2_POSE)
                        .setTangent(Math.PI / 2)
                        .splineToSplineHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build()
        ));
        sleep(500);

        // Move to third sample
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineToSplineHeading(AutoHelper.SAMPLE_3_POSE, 3 * Math.PI / 2)
                        .build()
        ));

        // Move to basket the fourth time
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.SAMPLE_3_POSE)
                        .setTangent(Math.PI / 2)
                        .splineToSplineHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build()
        ));
        sleep(500);
    }
}
