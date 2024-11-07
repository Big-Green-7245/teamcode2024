package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.output.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@TeleOp(name = "TeleOpBigGreenRi3W", group = "Big Green")
public class TeleOpBigGreenRi3W extends LinearOpMode {
    // Define attributes
    private static final String PROGRAM_VERSION = "0.1.0";
    private static final double SPEED_MULTIPLIER = 0.9;

    // Declare modules
    private ButtonHelper gp1, gp2;
    private DriveTrain driveTrain;
    private ServoToggle intakeSlide1, intakeSlide2;
    private ServoToggle intakePivot;
    private Servo activeIntake;
    private LinearSlide outputSlide;
    private ServoToggle outputBox;

    @Override
    public void runOpMode() {
        TelemetryWrapper.init(telemetry, 20);
        TelemetryWrapper.setLine(1, "TeleOp v" + PROGRAM_VERSION + "\t Initializing");

        // Initialize robot modules
        gp1 = new ButtonHelper(gamepad1);
        gp2 = new ButtonHelper(gamepad2);
        driveTrain = new DriveTrain(this);
        intakeSlide1 = new ServoToggle();
        intakeSlide2 = new ServoToggle();
        intakePivot = new ServoToggle();
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        outputSlide = new LinearSlide("outputSlide", 0.5, DcMotorSimple.Direction.REVERSE);
        outputBox = new ServoToggle();

        driveTrain.init(hardwareMap);
        intakeSlide1.init(hardwareMap, "intakeSlide1", 0, 0.2, false);
        intakeSlide2.init(hardwareMap, "intakeSlide2", 0, 0.2, true);
        intakePivot.init(hardwareMap, "intakePivot", 0, 0.66, false);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap, "outputBox", 0, 0.3, false);

        // Wait for start
        TelemetryWrapper.setLine(1, "TeleOp v" + PROGRAM_VERSION + "\t Press start to start >");
        while (opModeInInit()) {
            outputSlide.tickBeforeStart();
        }

        // Main loop
        while (opModeIsActive()) {
            // Update gamepads
            gp1.update();
            gp2.update();

            // Move drive train
            driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, SPEED_MULTIPLIER);

            if (Math.abs(gamepad2.left_stick_y) > 0.0001) {
                // Move intake slides
                intakeSlide1.interpolateAction(intakeSlide1.getInterpolatedPos() - gamepad2.left_stick_y * 0.2);
                intakeSlide2.interpolateAction(intakeSlide2.getInterpolatedPos() - gamepad2.left_stick_y * 0.2);
            }
            if (gp2.pressing(ButtonHelper.dpad_up)) {
                // Extend slides
                intakeSlide1.setAction(true);
                intakeSlide2.setAction(true);
            } else if (gp2.pressing(ButtonHelper.dpad_down)) {
                // Retract slides
                intakeSlide1.setAction(false);
                intakeSlide2.setAction(false);
            }
            if (gp2.pressing(ButtonHelper.dpad_right)) {
                // Toggle intake pivot between up and down
                intakePivot.toggleAction();
            }
            activeIntake.setPosition((gamepad2.right_trigger - gamepad2.left_trigger) / 2 + 0.5);

            // Move output slide
            if (Math.abs(gamepad2.right_stick_y) > 0.0001 || outputSlide.isFinished()) {
                // Move output slide by the right stick y if it is not zero and the slide is not currently moving to a position
                outputSlide.startMoveToRelativePos((int) (-gamepad2.right_stick_y * 500));
            }
            if (gp2.pressing(ButtonHelper.TRIANGLE)) {
                // Move the slide to the output position
                outputSlide.startMoveToPosSetBusy(1350);
            }
            if (gp2.pressing(ButtonHelper.CROSS)) {
                // Move the output box back
                outputBox.setAction(false);
                // Retract the slide to the bottom
                outputSlide.startRetraction();
            }
            outputSlide.tick();
            if (gp2.pressing(ButtonHelper.SQUARE)) {
                outputBox.toggleAction();
            }

            // Update telemetry
            TelemetryWrapper.setLineNoRender(1, "TeleOp v" + PROGRAM_VERSION + "\t Running");
            TelemetryWrapper.setLineNoRender(2, "Gamepad2RightStickY: " + gamepad2.right_stick_y * 500);
            TelemetryWrapper.setLineNoRender(3, "OutputSlidePos: " + outputSlide.getCurrentPosition());
            TelemetryWrapper.setLineNoRender(4, "OutputSlideTargetPos: " + outputSlide.getTargetPosition());
            TelemetryWrapper.setLineNoRender(5, "OutputSlideButton: " + outputSlide.isElevatorBtnPressed());
            TelemetryWrapper.setLine(6, "OutputSlideCurrent: " + outputSlide.getCurrent() + "A");
        }
    }
}
