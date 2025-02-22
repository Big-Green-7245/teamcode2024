package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.modules.DoubleServoToggle;
import org.firstinspires.ftc.teamcode.modules.ServoToggle;
import org.firstinspires.ftc.teamcode.modules.motor.DoubleLinearSlides;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

/**
 * This runs the path in {@link AutoBasket} without the input and output modules.
 */
@Autonomous(name = "AutoBasketPathOnly", group = "Big Green", preselectTeleOp = "TeleOpBasket")
public class AutoBasketPathOnly extends LinearOpMode {
    // Declare modules
    private TelemetryWrapper telemetryWrapper;
    private MecanumDrive drive;
    private ServoToggle intakeSlide;
    private ServoToggle intakePivot;
    private DoubleLinearSlides outputSlide;
    private ServoToggle outputBox;
    private ServoToggle specimenClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryWrapper = new TelemetryWrapper(telemetry);
        telemetryWrapper.setLineAndRender(1, "Basket Auto PATH ONLY \tInitializing");

        // Initialize robot modules
        drive = new PinpointDrive(hardwareMap, AutoHelper.BASKET_INITIAL_POSE);
        intakeSlide = new DoubleServoToggle("intakeSlide", 0, 0.3, Servo.Direction.REVERSE, Servo.Direction.FORWARD);
        intakePivot = new DoubleServoToggle("intakePivot", 0, 0.66, Servo.Direction.FORWARD, Servo.Direction.REVERSE);
        outputSlide = new DoubleLinearSlides(
                List.of(Pair.create("outputSlideLeft", DcMotorSimple.Direction.REVERSE), Pair.create("outputSlideLeft2", DcMotorSimple.Direction.FORWARD)),
                List.of(Pair.create("outputSlideRight", DcMotorSimple.Direction.FORWARD), Pair.create("outputSlideRight2", DcMotorSimple.Direction.REVERSE)),
                1, (int) (0.1 * AutoHelper.OUTPUT_SLIDE.getPulsesPerRevolution()), Integer.MAX_VALUE
        );
        outputBox = new ServoToggle("outputBox", 0, 0.4, true);
        specimenClaw = new ServoToggle("specimenClaw", 0, 0.2, false);

        intakeSlide.init(hardwareMap);
        intakePivot.init(hardwareMap);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap);
        specimenClaw.init(hardwareMap);

        // Wait for start
        telemetryWrapper.setLineAndRender(1, "Basket Auto PATH ONLY \tPress start to start >");
        while (opModeInInit()) {
            outputSlide.tickBeforeStart();
        }

        // Begin autonomous program
        // See AutoBasketPathTest.java for a visualization

        // Move to basket
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_INITIAL_POSE)
                        .setTangent(3 * Math.PI / 2)
                        .splineToLinearHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build()
        ));
        sleep(500);

        for (Pose2d samplePose : AutoHelper.SAMPLE_POSES) {
            Actions.runBlocking(drive.actionBuilder(AutoHelper.BASKET_POSE)
                    // Move to sample
                    .setTangent(5 * Math.PI / 4)
                    .splineToSplineHeading(samplePose, samplePose.heading)
                    // Move to basket
                    .lineToX(samplePose.position.x + 4 * samplePose.heading.real)
                    .splineToSplineHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                    .build()
            );
            sleep(500);
        }

        // Move to ascent zone
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineTo(AutoHelper.ASCENT_ZONE_POSE.position, Math.PI)
                        .build()
        ));

        // Move to basket
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.ASCENT_ZONE_POSE)
                        .setTangent(0)
                        .splineToLinearHeading(AutoHelper.BASKET_POSE, Math.PI / 4)
                        .build()
        ));
        sleep(500);

        // Move to ascent zone
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(AutoHelper.BASKET_POSE)
                        .setTangent(5 * Math.PI / 4)
                        .splineToLinearHeading(AutoHelper.ASCENT_ZONE_POSE_2, Math.PI)
                        .build()
        ));
    }
}
