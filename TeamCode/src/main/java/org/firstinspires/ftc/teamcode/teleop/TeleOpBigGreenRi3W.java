package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.output.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.util.List;

@TeleOp(name = "TeleOpBigGreenRi3W", group = "Big Green")
public class TeleOpBigGreenRi3W extends LinearOpMode {
    // Define attributes
    private static final String PROGRAM_VERSION = "0.1.0";
    private static final double SPEED_MULTIPLIER = 0.9;
    private final ElapsedTime timer = new ElapsedTime();

    // Declare modules
    private List<LynxModule> hubs;
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
        hubs = hardwareMap.getAll(LynxModule.class);
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

        // Manual bulk caching to ensure sensors only get read once per loop
        // This can save a lot of time in the execution loop
        // See https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Wait for start
        TelemetryWrapper.setLine(1, "TeleOp v" + PROGRAM_VERSION + "\t Press start to start >");
        while (opModeInInit()) {
            clearBulkCache();

            outputSlide.tickBeforeStart();

            // Debug loop times
            TelemetryWrapper.setLine(2, "TeleOp Init loop time: " + timer.milliseconds() + " ms");
            timer.reset();
        }

        // Main loop
        while (opModeIsActive()) {
            clearBulkCache();

            // Update gamepads
            gp1.update();
            gp2.update();

            // Move drive train
            driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, SPEED_MULTIPLIER);

            if (gp2.pressing(ButtonHelper.dpad_up)) {
                // Extend slides
                intakeSlide1.setAction(true);
                intakeSlide2.setAction(true);
            } else if (gp2.pressing(ButtonHelper.dpad_down)) {
                // Retract slides
                intakeSlide1.setAction(false);
                intakeSlide2.setAction(false);
            } else if (Math.abs(gamepad2.left_stick_y) > 0.0001) {
                // Move intake slides
                intakeSlide1.interpolateAction(intakeSlide1.getInterpolatedPos() - gamepad2.left_stick_y * 0.2);
                intakeSlide2.interpolateAction(intakeSlide2.getInterpolatedPos() - gamepad2.left_stick_y * 0.2);
            }
            if (gp2.pressing(ButtonHelper.dpad_right)) {
                // Toggle intake pivot between up and down
                intakePivot.toggleAction();
            }
            activeIntake.setPosition((gamepad2.right_trigger - gamepad2.left_trigger) / 2 + 0.5);

            // Move output slide
            if (gp2.pressing(ButtonHelper.TRIANGLE)) {
                // Move the slide to the output position
                outputSlide.startMoveToPosSetBusy(1350);
            } else if (gp2.pressing(ButtonHelper.CROSS)) {
                // Move the output box back
                outputBox.setAction(false);
                // Retract the slide to the bottom
                outputSlide.startRetraction();
            } else if (Math.abs(gamepad2.right_stick_y) > 0.0001) {
                // Move output slide by the right stick y if it is not zero and the slide is not currently moving to a position
                outputSlide.startMoveToRelativePos((int) (-gamepad2.right_stick_y * 500));
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

            // Debug loop times
            TelemetryWrapper.setLine(7, "TeleOp loop time: " + timer.milliseconds() + " ms");
            timer.reset();
        }
    }

    /**
     * Resets the cache every iteration to update the sensors once every loop.
     * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html">Bulk Reads</a>
     */
    private void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
