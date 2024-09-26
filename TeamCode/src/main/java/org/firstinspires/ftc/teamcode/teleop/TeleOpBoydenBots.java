package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.output.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@TeleOp(name = "TeleOpBoydenBotsRi3W", group = "opmode")
public class TeleOpBoydenBots extends LinearOpMode {
    private final String PROGRAM_VERSION = "2.1";  // Version updated
    private final double SPEED_MULTIPLIER = 0.99;

    // Input helpers and hardware modules
    private ButtonHelper gp1, gp2;
    private DriveTrain driveTrain;
    private DcMotor pivot;
    private ServoToggle clawServo;  // Servo for the claw
    private LinearSlide outputSlide;

    @Override
    public void runOpMode() {
        TelemetryWrapper.init(telemetry, 20);
        TelemetryWrapper.setLine(1, "TeleOp v" + PROGRAM_VERSION + "\t Initializing");

        gp1 = new ButtonHelper(gamepad1);
        gp2 = new ButtonHelper(gamepad2);
        driveTrain = new DriveTrain(this);
        clawServo = new ServoToggle();
        outputSlide = new LinearSlide("outputSlide", 0.5);

        driveTrain.init(hardwareMap);
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        // Initializing the claw servo with min/max range and no reversed direction
        clawServo.init(hardwareMap, "clawServo", 0, 0.3, false);  // Assuming range is 0 to 1 for open and close
        outputSlide.init(hardwareMap);

        TelemetryWrapper.setLine(1, "TeleOp v" + PROGRAM_VERSION + "\t Press start to start >");

        while (opModeInInit()) {
            outputSlide.tickBeforeStart();
        }

        while (opModeIsActive()) {
            TelemetryWrapper.setLineNoRender(3, "LeftSlidePos" + outputSlide.getCurrentPosition()[0]);
            TelemetryWrapper.setLineNoRender(4, "RightSlidePos" + outputSlide.getCurrentPosition()[1]);
            TelemetryWrapper.setLineNoRender(5, "PivotPos" + pivot.getCurrentPosition());
            TelemetryWrapper.setLine(6, "ClawPos: " + clawServo.getPosition());

            // Updating button inputs from gamepads
            gp1.update();
            gp2.update();
            // Drive movement: left stick (direction), right stick (turn)
            driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, SPEED_MULTIPLIER);

            // right to increase power, left to decrease
            pivot.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            if (gp2.pressing(ButtonHelper.dpad_up)) {
                clawServo.toggleAction();
            }

            if (Math.abs(gamepad2.right_stick_y) > 0.0001 || outputSlide.isFinished()) {
                outputSlide.startMoveToRelativePos((int) -gamepad2.right_stick_y * 500);
            } else if (gp2.pressing(ButtonHelper.y)) {
                outputSlide.startMoveToPosSetBusy(1200);
            } else if (gp2.pressing(ButtonHelper.a)) {
                outputSlide.startRetraction();
            }
            outputSlide.tick(); // each loop iteration
        }
    }
}
