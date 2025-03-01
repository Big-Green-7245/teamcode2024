package org.firstinspires.ftc.teamcode.teleop;

import android.util.Pair;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.autonomous.AutoHelper;
import org.firstinspires.ftc.teamcode.modules.DoubleServoToggle;
import org.firstinspires.ftc.teamcode.modules.ServoToggle;
import org.firstinspires.ftc.teamcode.modules.motor.DoubleLinearSlides;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
public class TeleOp extends LinearOpMode {
    // Define constants
    private final ElapsedTime timer = new ElapsedTime();
    private final Pose2d initialPose;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final List<Action> actions = new ArrayList<>();

    // Declare modules
    private List<LynxModule> hubs;
    private TelemetryWrapper telemetryWrapper;
    private ButtonHelper gp1, gp2;
    private MecanumDrive driveTrain;
    private ServoToggle intakeSlide;
    private ServoToggle intakePivot;
    private Servo activeIntake;
    private ServoToggle intakeSweeper;
    private DoubleLinearSlides outputSlide;
    private ServoToggle outputBox;
    private ServoToggle specimenClaw;

    public TeleOp(Pose2d initialPose) {
        this.initialPose = initialPose;
    }

    @Override
    public void runOpMode() {
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "TeleOp \tInitializing");

        // Initialize robot modules
        hubs = hardwareMap.getAll(LynxModule.class);
        gp1 = new ButtonHelper(gamepad1);
        gp2 = new ButtonHelper(gamepad2);
        driveTrain = new PinpointDrive(hardwareMap, initialPose);
        intakeSlide = new DoubleServoToggle("intakeSlide", 0, 0.3, Servo.Direction.REVERSE, Servo.Direction.FORWARD);
        intakePivot = new DoubleServoToggle("intakePivot", 0, 0.66, Servo.Direction.FORWARD, Servo.Direction.REVERSE);
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        intakeSweeper = new ServoToggle("intakeSweeper", 0, 0.5, false);
        outputSlide = new DoubleLinearSlides(
                List.of(Pair.create("outputSlideLeft", DcMotorSimple.Direction.REVERSE), Pair.create("outputSlideLeft2", DcMotorSimple.Direction.FORWARD)),
                List.of(Pair.create("outputSlideRight", DcMotorSimple.Direction.FORWARD), Pair.create("outputSlideRight2", DcMotorSimple.Direction.REVERSE)),
                1, (int) (0.1 * AutoHelper.OUTPUT_SLIDE_ENCODER), (int) (8.58 * AutoHelper.OUTPUT_SLIDE_ENCODER)
        );
        outputBox = new ServoToggle("outputBox", 0, 0.4, true);
        specimenClaw = new ServoToggle("specimenClaw", 0, 0.2, false);

        intakeSlide.init(hardwareMap);
        intakePivot.init(hardwareMap);
        intakeSweeper.init(hardwareMap);
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
        telemetryWrapper.setLineAndRender(1, "TeleOp \tPress start to start >");
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
            if (Math.abs(gamepad1.left_stick_y) > 0.0001 || Math.abs(gamepad1.left_stick_x) > 0.0001 || Math.abs(gamepad1.right_stick_x) > 0.0001) {
                actions.clear();
            }
            if (actions.isEmpty()) {
                driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));
            }
            driveTrain.updatePoseEstimate();

            double driveTrainTime = timer.milliseconds();

            if (gp2.pressing(ButtonHelper.dpad_up)) {
                // Extend slides
                intakeSlide.setAction(true);
            } else if (gp2.pressing(ButtonHelper.dpad_down)) {
                // Retract slides
                intakeSlide.setAction(false);
            } else if (Math.abs(gamepad2.left_stick_y) > 0.0001) {
                // Move intake slides
                intakeSlide.setPosition(intakeSlide.getPosition() - gamepad2.left_stick_y * 0.1);
            }
            if (gp2.pressing(ButtonHelper.dpad_right)) {
                // Toggle intake pivot between up and down
                intakePivot.toggleAction();
            }
            if (actions.isEmpty()) {
                activeIntake.setPosition((gamepad2.right_trigger - gamepad2.left_trigger) / 2 + 0.5);
            }
            if (gp2.pressing(ButtonHelper.dpad_left)) {
                // Toggle intake sweeper
                intakeSweeper.toggleAction();
            }

            double intakeTime = timer.milliseconds();

            // Move output slide
            if (gp1.pressing(ButtonHelper.left_bumper)) {
                // Automatically transfer, drive, and deposit sample in the high basket
                actions.add(new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        driveTrain.actionBuilder(driveTrain.pose)
                                                .setTangent(driveTrain.pose.heading.plus(Math.PI))
                                                .splineToLinearHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                                                .build()
                                ),
                                new SequentialAction(
                                        AutoHelper.retractIntake(intakeSlide, intakePivot, activeIntake),
                                        new SleepAction(0.1),
                                        AutoHelper.transferSample(activeIntake),
                                        AutoHelper.moveSlideToPos(outputSlide, AutoHelper.BASKET_SLIDE_HIGH, AutoHelper.BASKET_SLIDE_TOLERANCE)
                                )
                        ),
                        new InstantAction(() -> outputBox.setAction(true)),
                        new SleepAction(AutoHelper.BASKET_DEPOSIT_TIME / 1000d),
                        new ParallelAction(
                                driveTrain.actionBuilder(AutoHelper.BASKET_POSE)
                                        .setTangent(5 * Math.PI / 4)
                                        .splineToLinearHeading(driveTrain.pose, driveTrain.pose.heading)
                                        .build(),
                                new InstantAction(() -> outputBox.setAction(false)),
                                AutoHelper.retractSlide(outputSlide)
                        )
                ));
            } else if (gp2.pressing(ButtonHelper.right_bumper) || gp1.pressing(ButtonHelper.TRIANGLE)) {
                // Move the slide to the specimen output position
                outputSlide.startMoveToPos(AutoHelper.SPECIMEN_SLIDE_HIGH);
            } else if (gp2.pressing(ButtonHelper.TRIANGLE)) {
                // Move the slide to the basket output position
                outputSlide.startMoveToPos(AutoHelper.BASKET_SLIDE_HIGH);
            } else if (gp1.pressing(ButtonHelper.CIRCLE)) {
                // Hanging step 1, robot slightly off the ground
                outputSlide.startMoveToPos((int) (2.42 * AutoHelper.OUTPUT_SLIDE_ENCODER));
            } else if (gp2.pressing(ButtonHelper.CROSS) || gp1.pressing(ButtonHelper.CROSS)) {
                // Move the output box back
                outputBox.setAction(false);
                // Retract the slide to the bottom
                outputSlide.startRetraction();
            } else if (Math.abs(gamepad2.right_stick_y) > 0.0001) {
                // Move output slide by the right stick y if it is not zero and the slide is not currently moving to a position
                outputSlide.startMoveToRelativePos((int) (-gamepad2.right_stick_y * AutoHelper.OUTPUT_SLIDE_ENCODER));
            }
            outputSlide.tick();
            if (gp2.pressing(ButtonHelper.SQUARE) || gp1.pressing(ButtonHelper.right_bumper)) {
                outputBox.toggleAction();
            }

            double outputTime = timer.milliseconds();

            // Specimen claw
            if (gp2.pressing(ButtonHelper.CIRCLE)) {
                specimenClaw.toggleAction();
            }

            double specimenClawTime = timer.milliseconds();

            // Run actions
            if (!actions.isEmpty()) {
                TelemetryPacket packet = new TelemetryPacket();
                for (Iterator<Action> iterator = actions.iterator(); iterator.hasNext(); ) {
                    Action action = iterator.next();
                    action.preview(packet.fieldOverlay());
                    if (!action.run(packet)) {
                        iterator.remove();
                    }
                }
                dashboard.sendTelemetryPacket(packet);
            }

            double autoActionsTime = timer.milliseconds();

            // Update telemetry
//            telemetryWrapper.setLine(1, "TeleOp \tRunning");
//            telemetryWrapper.setLine(2, "Gamepad2RightStickY: %s", gamepad2.right_stick_y * 500);
//            int[] currentPositions = outputSlide.getCurrentPositions();
//            telemetryWrapper.setLine(3, "OutputSlidePos Left: %d; Right: %d; Diff: %d", currentPositions[0], currentPositions[1], currentPositions[1] - currentPositions[0]);
//            double telemetryCurrentPositionsTime = timer.milliseconds();
//            int[] targetPositions = outputSlide.getTargetPositions();
//            telemetryWrapper.setLine(4, "OutputSlideTargetPos Left: %d; Right: %d; Diff: %d", targetPositions[0], targetPositions[1], targetPositions[1] - targetPositions[0]);
//            double telemetryTargetPositionsTime = timer.milliseconds();
//            boolean[] elevatorButtons = outputSlide.areElevatorButtonsPressed();
//            telemetryWrapper.setLine(5, "OutputSlideButton Left: %s; Right: %s", elevatorButtons[0], elevatorButtons[1]);
//            double telemetryElevatorButtonsTime = timer.milliseconds();
//            telemetryWrapper.setLine(6, "OutputSlideCurrent: %sA", outputSlide.getCurrent());

            // Debug loop times
            double telemetryTime = timer.milliseconds();
            telemetryWrapper.setLine(8, "TeleOp loop time: %.2f ms; ClearBulkCacheTime: %.2f ms; Gamepads: %.2f ms; DriveTrain: %.2f ms; Intake: %.2f ms; Output: %.2f ms; SpecimenClaw: %.2f ms; AutoActionsTime: %.2f ms; TelemetryTime: %.2f ms", telemetryTime, clearBulkCacheTime, gamepadsTime - clearBulkCacheTime, driveTrainTime - gamepadsTime, intakeTime - driveTrainTime, outputTime - intakeTime, specimenClawTime - outputTime, autoActionsTime - specimenClawTime, telemetryTime - autoActionsTime);
//            telemetryWrapper.setLine(9, "OutputSlideTelemetry CurrentPos: %.2f ms; TargetPos: %.2f ms; Buttons: %.2f ms; Current: %.2f ms", telemetryCurrentPositionsTime - specimenClawTime, telemetryTargetPositionsTime - telemetryCurrentPositionsTime, telemetryElevatorButtonsTime - telemetryTargetPositionsTime, telemetryTime - telemetryElevatorButtonsTime);
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
