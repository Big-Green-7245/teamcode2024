package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@Autonomous(name = "AutoSpecimenPathOnly", group = "Big Green", preselectTeleOp = "TeleOp")
public class AutoSpecimenPathOnly extends LinearOpMode {
    private TelemetryWrapper telemetryWrapper;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryWrapper = new TelemetryWrapper(telemetry, 20);
        telemetryWrapper.setLineAndRender(1, "Specimen Auto PATH ONLY\t Initializing");

        // Initialize robot modules
        drive = new PinpointDrive(hardwareMap, AutoHelper.SPECIMEN_INITIAL_POSE);

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "Specimen Auto PATH ONLY\t Press start to start >");
        while (opModeInInit()) {}

        // Begin autonomous program
        // See AutoSpecimenPathTest.java for a visualization

        // Move to submersible and deposit the preload specimen
        Actions.runBlocking(drive.actionBuilder(AutoHelper.SPECIMEN_INITIAL_POSE)
                .setTangent(3 * Math.PI / 2)
                .splineToConstantHeading(AutoHelper.INITIAL_SUBMERSIBLE_POSE.position, 3 * Math.PI / 2)
                .build()
        );

        Actions.runBlocking(drive.actionBuilder(AutoHelper.INITIAL_SUBMERSIBLE_POSE)
                // Move to first sample while resetting specimen claw and retracting slides
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(-36, 32), 3 * Math.PI / 2)
                .splineToSplineHeading(AutoHelper.SPECIMEN_SAMPLE_1_POSE, Math.PI / 2)
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
        );

        // Move to submersible to deposit second specimen
        Actions.runBlocking(drive.actionBuilder(AutoHelper.OBSERVATION_ZONE_POSE)
                .setTangent(3 * Math.PI / 2)
                .splineToLinearHeading(AutoHelper.SUBMERSIBLE_POSE, 3 * Math.PI / 2)
                .build()
        );

        for (int i = 0; i < 3; i++) {
            // Move to the side while depositing the specimen
            // Move to observation zone and pick up specimen
            Actions.runBlocking(drive.actionBuilder(AutoHelper.SUBMERSIBLE_POSE)
                    .setTangent(0)
                    .splineToLinearHeading(AutoHelper.OBSERVATION_ZONE_POSE, Math.PI / 2)
                    .build()
            );

            // Move to submersible to deposit specimen
            Actions.runBlocking(drive.actionBuilder(AutoHelper.OBSERVATION_ZONE_POSE)
                    .setTangent(3 * Math.PI / 2)
                    .splineToLinearHeading(AutoHelper.SUBMERSIBLE_POSE, 3 * Math.PI / 2)
                    .build()
            );
        }
    }
}
