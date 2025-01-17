package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.autonomous.AutoHelper;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.output.DoubleLinearSlides;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Big Green")
public class TeleOp extends LinearOpMode {
    // Define attributes
    private static final String PROGRAM_VERSION = "0.1.0";
    private static final double SPEED_MULTIPLIER = 1;
    private final ElapsedTime timer = new ElapsedTime();

    // Declare modules
    private List<LynxModule> hubs;
    private TelemetryWrapper telemetryWrapper;
    private ButtonHelper gp1, gp2;
    private DriveTrain driveTrain;
    private ServoToggle intakeSlide1, intakeSlide2;
    private ServoToggle intakePivot;
    private Servo activeIntake;
    private DoubleLinearSlides outputSlide;
    private ServoToggle outputBox;
    private ServoToggle specimenClaw;

    @Override
    public void runOpMode() {
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "TeleOp v%s\t Initializing", PROGRAM_VERSION);

        // Initialize robot modules
        hubs = hardwareMap.getAll(LynxModule.class);
        gp1 = new ButtonHelper(gamepad1);
        gp2 = new ButtonHelper(gamepad2);
        driveTrain = new DriveTrain(this, telemetryWrapper);
        intakeSlide1 = new ServoToggle("intakeSlide1", 0, 0.2, true);
        intakeSlide2 = new ServoToggle("intakeSlide2", 0, 0.2, false);
        intakePivot = new ServoToggle("intakePivot", 0, 0.66, false);
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        outputSlide = new DoubleLinearSlides("outputSlide", 1, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, 3300);
        outputBox = new ServoToggle("outputBox", 0, 0.4, true);
        specimenClaw = new ServoToggle("specimenClaw", 0, 0.2, false);

        driveTrain.init(hardwareMap);
        intakeSlide1.init(hardwareMap);
        intakeSlide2.init(hardwareMap);
        intakePivot.init(hardwareMap);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap);
        specimenClaw.init(hardwareMap);

        // Manual bulk caching to ensure sensors only get read once per loop
        // This can save a lot of time in the execution loop
        // See https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "TeleOp v%s\t Press start to start >", PROGRAM_VERSION);
        while (opModeInInit()) {
            clearBulkCache();

            outputSlide.tickBeforeStart();

            // Debug loop times
            telemetryWrapper.setLineAndRender(2, "TeleOp Init loop time: %s ms", timer.milliseconds());
            timer.reset();
        }

        // Main loop
        while (opModeIsActive()) {
            clearBulkCache();

            double clearBulkCacheTime = timer.milliseconds();

            // Update gamepads
            gp1.update();
            gp2.update();

            double gamepadsTime = timer.milliseconds();

            // Move drive train
            driveTrain.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, SPEED_MULTIPLIER);

            double driveTrainTime = timer.milliseconds();

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
                intakeSlide1.setPosition(intakeSlide1.getPosition() - gamepad2.left_stick_y * 0.2);
                intakeSlide2.setPosition(intakeSlide2.getPosition() - gamepad2.left_stick_y * 0.2);
            }
            if (gp2.pressing(ButtonHelper.dpad_right)) {
                // Toggle intake pivot between up and down
                intakePivot.toggleAction();
            }
            activeIntake.setPosition((gamepad2.right_trigger - gamepad2.left_trigger) / 2 + 0.5);

            double intakeTime = timer.milliseconds();

            // Move output slide
            if (gp2.pressing(ButtonHelper.TRIANGLE)) {
                // Move the slide to the output position
                outputSlide.startMoveToPos(AutoHelper.BASKET_SLIDE_HIGH);
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
            } else if (gp1.pressing(ButtonHelper.right_bumper)) {
                outputBox.setAction(true);
            } else if (gp1.pressing(ButtonHelper.left_bumper)) {
                outputBox.setAction(false);
            }

            double outputTime = timer.milliseconds();

            // Specimen claw
            if (gp2.pressing(ButtonHelper.CIRCLE)) {
                specimenClaw.toggleAction();
            }

            double specimenClawTime = timer.milliseconds();

            // Update telemetry
            telemetryWrapper.setLine(1, "TeleOp v%s\t Running", PROGRAM_VERSION);
            telemetryWrapper.setLine(2, "Gamepad2RightStickY: %s", gamepad2.right_stick_y * 500);
            int[] currentPositions = outputSlide.getCurrentPositions();
            telemetryWrapper.setLine(3, "OutputSlidePos Left: %d; Right: %d; Diff: %d", currentPositions[0], currentPositions[1], currentPositions[1] - currentPositions[0]);
            double telemetryCurrentPositionsTime = timer.milliseconds();
            int[] targetPositions = outputSlide.getTargetPositions();
            telemetryWrapper.setLine(4, "OutputSlideTargetPos Left: %d; Right: %d; Diff: %d", targetPositions[0], targetPositions[1], targetPositions[1] - targetPositions[0]);
            double telemetryTargetPositionsTime = timer.milliseconds();
            boolean[] elevatorButtons = outputSlide.areElevatorButtonsPressed();
            telemetryWrapper.setLine(5, "OutputSlideButton Left: %s; Right: %s", elevatorButtons[0], elevatorButtons[1]);
            double telemetryElevatorButtonsTime = timer.milliseconds();
            telemetryWrapper.setLine(6, "OutputSlideCurrent: %sA", outputSlide.getCurrent());

            // Debug loop times
            double telemetryTime = timer.milliseconds();
            telemetryWrapper.setLine(7, "TeleOp loop time: %.2f ms; ClearBulkCacheTime: %.2f ms; Gamepads: %.2f ms; DriveTrain: %.2f ms; Intake: %.2f ms; Output: %.2f ms; SpecimenClaw: %.2f ms; TelemetryTime: %.2f ms", telemetryTime, clearBulkCacheTime, gamepadsTime - clearBulkCacheTime, driveTrainTime - gamepadsTime, intakeTime - driveTrainTime, outputTime - intakeTime, specimenClawTime - outputTime, telemetryTime - specimenClawTime);
            telemetryWrapper.setLine(8, "OutputSlideTelemetry CurrentPos: %.2f ms; TargetPos: %.2f ms; Buttons: %.2f ms; Current: %.2f ms", telemetryCurrentPositionsTime - specimenClawTime, telemetryTargetPositionsTime - telemetryCurrentPositionsTime, telemetryElevatorButtonsTime - telemetryTargetPositionsTime, telemetryTime - telemetryElevatorButtonsTime);
            telemetryWrapper.render();
            timer.reset();
        }
    }

    /**
     * Resets the cache every iteration to update the sensors once every loop.
     *
     * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html">Bulk Reads</a>
     */
    private void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
