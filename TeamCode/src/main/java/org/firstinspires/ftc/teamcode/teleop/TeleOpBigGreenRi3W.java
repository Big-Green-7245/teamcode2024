package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private ServoToggle intakeSlide;
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
        intakeSlide = new ServoToggle();
        intakePivot = new ServoToggle();
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        outputSlide = new LinearSlide("outputSlide", 0.5);
        outputBox = new ServoToggle();
        driveTrain.init(hardwareMap);
        intakeSlide.init(hardwareMap, "intakeSlide", 0, 0.5, false);
        intakePivot.init(hardwareMap, "intakePivot", 0, 1, false);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap, "outputBox", 0, 0.5, false);

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

            if (gamepad2.left_stick_y > 0.0001) {
                // Move intake slides
                intakeSlide.setPosition(gamepad2.left_stick_y / 2 + 0.5);
            } else if (gp2.pressing(ButtonHelper.dpad_down)) {
                // Retract slides
                intakeSlide.setAction(false);
            }
            if (gp2.pressing(ButtonHelper.dpad_right)) {
                // Toggle intake pivot between up and down
                intakePivot.toggleAction();
            }
            activeIntake.setPosition((gamepad2.right_trigger - gamepad2.left_trigger) / 2 + 0.5);

            // Move output slide
            if (Math.abs(gamepad2.right_stick_y) > 0.0001 || outputSlide.isFinished()) {
                // Move output slide by the right stick y if it is not zero and the slide is not currently moving to a position
                outputSlide.startMoveToRelativePos((int) (gamepad2.right_stick_y * 500));
            } else if (gp2.pressing(ButtonHelper.y)) {
                // Move the slide to the output position
                outputSlide.startMoveToPosSetBusy(2000);
            } else if (gp2.pressing(ButtonHelper.a)) {
                // Retract the slide to the bottom
                outputSlide.startRetraction();
            }
            outputSlide.tick();
            if (gp2.pressing(ButtonHelper.x)) {
                outputBox.toggleAction();
            }
        }
    }
}
