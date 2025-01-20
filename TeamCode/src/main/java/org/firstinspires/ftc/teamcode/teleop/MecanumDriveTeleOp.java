package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name = "MecanumDriveTeleOp", group = "opmode")
public class MecanumDriveTeleOp extends LinearOpMode {
    // Declare modules
    private TelemetryWrapper telemetryWrapper;
    private ButtonHelper gp1, gp2;
    private MecanumDrive driveTrain;

    @Override
    public void runOpMode() {
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "Mecanum Drive TeleOp\t Initializing");

        // Robot modules initialization
        gp1 = new ButtonHelper(gamepad1);
        gp2 = new ButtonHelper(gamepad2);
        driveTrain = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "Mecanum Drive TeleOp\t Press start to start >");

        while (opModeInInit()) {
        }


        while (opModeIsActive()) {
            // Update ButtonHelper
            gp1.update();
            gp2.update();

            // DriveTrain wheels
            driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));
            driveTrain.updatePoseEstimate();
        }
    }
}
