package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.modules.output.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@TeleOp(name = "TeleOpBoydenBotsRi3W", group = "opmode")
public class TeleOpBoydenBots extends LinearOpMode {
    // Input helpers and hardware modules
    private TelemetryWrapper telemetryWrapper;
    private ButtonHelper gp1, gp2;
    private MecanumDrive driveTrain;
    private DcMotor pivot;
    private ServoToggle clawServo;  // Servo for the claw
    private LinearSlide outputSlide;

    @Override
    public void runOpMode() {
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "TeleOpBoydenBots \tInitializing");

        gp1 = new ButtonHelper(gamepad1);
        gp2 = new ButtonHelper(gamepad2);
        driveTrain = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        clawServo = new ServoToggle("clawServo", 0, 0.3, false);

        pivot = hardwareMap.get(DcMotor.class, "pivot");
        clawServo.init(hardwareMap);

        outputSlide = new LinearSlide("outputSlide", 0.5, DcMotorSimple.Direction.REVERSE);
        outputSlide.init(hardwareMap);

        telemetryWrapper.setLineAndRender(1, "TeleOpBoydenBots \tPress start to start >");

        while (opModeInInit()) {}

        while (opModeIsActive()) {
            telemetryWrapper.setLine(3, "OutputSlidePos: " + outputSlide.getCurrentPosition());
            telemetryWrapper.setLine(5, "PivotPos: " + pivot.getCurrentPosition());
            telemetryWrapper.setLine(6, "ClawPos: " + clawServo.getPosition());
            telemetryWrapper.render();

            gp1.update();
            gp2.update();

            // Drive movement: left stick (direction), right stick (turn)
            driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));
            driveTrain.updatePoseEstimate();

            pivot.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            if (gp2.pressing(ButtonHelper.dpad_up)) {
                clawServo.toggleAction();
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.0001 || outputSlide.isFinished()) {
                outputSlide.startMoveToRelativePos((int) -gamepad2.left_stick_y * 500);  // Adjust position based on joystick
            }
            if (gp2.pressing(ButtonHelper.b)) {
                outputSlide.startMoveToPos(1200);  // Middle position preset
            }
            if (gp2.pressing(ButtonHelper.y)) {
                outputSlide.startMoveToPos(1500);  // Top position preset
            }
            if (gp2.pressing(ButtonHelper.a)) {
                outputSlide.startRetraction();  // Retract slide fully
            }

            outputSlide.tick();
        }
    }
}


// Control hub: motors, Servo