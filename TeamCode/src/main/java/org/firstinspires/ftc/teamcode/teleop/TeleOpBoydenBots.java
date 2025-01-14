package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        TelemetryWrapper.setLineAndRender(1, "TeleOp v" + PROGRAM_VERSION + "\t Initializing");

        gp1 = new ButtonHelper(gamepad1);
        gp2 = new ButtonHelper(gamepad2);
        driveTrain = new DriveTrain(this);
        clawServo = new ServoToggle();

        driveTrain.init(hardwareMap);
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        clawServo.init(hardwareMap, "clawServo", 0, 0.3, false);

        outputSlide = new LinearSlide("outputSlide", 0.5, DcMotorSimple.Direction.REVERSE);
        outputSlide.init(hardwareMap);

        TelemetryWrapper.setLineAndRender(1, "TeleOp v" + PROGRAM_VERSION + "\t Press start to start >");

        while (opModeInInit()) {}

        while (opModeIsActive()) {
            TelemetryWrapper.setLine(3, "OutputSlidePos: " + outputSlide.getCurrentPosition());
            TelemetryWrapper.setLine(5, "PivotPos: " + pivot.getCurrentPosition());
            TelemetryWrapper.setLine(6, "ClawPos: " + clawServo.getPosition());
            TelemetryWrapper.render();

            gp1.update();
            gp2.update();

            // Drive movement: left stick (direction), right stick (turn)
            driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, SPEED_MULTIPLIER);

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