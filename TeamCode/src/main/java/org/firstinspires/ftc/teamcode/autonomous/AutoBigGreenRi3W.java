package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.output.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.output.ServoToggle;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

@Autonomous(name = "AutoBigGreenRi3W", group = "Big Green", preselectTeleOp = "TeleOpBigGreenRi3W")
public class AutoBigGreenRi3W extends LinearOpMode {
    // Define attributes
    private static final String PROGRAM_VERSION = "0.1.0";
    private static final double SPEED_MULTIPLIER = 0.9;

    // Declare modules
    private ButtonHelper gp1, gp2;
    private ServoToggle intakeSlide;
    private ServoToggle intakePivot;
    private Servo activeIntake;
    private LinearSlide outputSlide;
    private ServoToggle outputBox;
    private DcMotor hangingActuator;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryWrapper.init(telemetry, 20);
        TelemetryWrapper.setLine(1, "TeleOp v" + PROGRAM_VERSION + "\t Initializing");

        // Initialize robot modules
        gp1 = new ButtonHelper(gamepad1);
        gp2 = new ButtonHelper(gamepad2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeSlide = new ServoToggle();
        intakePivot = new ServoToggle();
        activeIntake = hardwareMap.get(Servo.class, "activeIntake");
        outputSlide = new LinearSlide("outputSlide", 0.5, DcMotorSimple.Direction.REVERSE);
        outputBox = new ServoToggle();
        hangingActuator = hardwareMap.get(DcMotor.class, "hangingActuator");

        intakeSlide.init(hardwareMap, "intakeSlide", 0, 0.2, false);
        intakePivot.init(hardwareMap, "intakePivot", 0, 0.66, false);
        outputSlide.init(hardwareMap);
        outputBox.init(hardwareMap, "outputBox", 0, 0.4, false);

        // Wait for start
        TelemetryWrapper.setLine(1, "TeleOp v" + PROGRAM_VERSION + "\t Press start to start >");
        while (opModeInInit()) {
            outputSlide.tickBeforeStart();
        }

        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(27, 0), 7 * Math.PI / 4)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(4, 23), 7 * Math.PI / 4)
                .build()
        );
        outputSlide.startMoveToPosSetBusy(1300);
        while (opModeIsActive() && !outputSlide.isFinished()) {
            outputSlide.tick();
        }
        outputBox.setAction(true);
        sleep(500);
        outputBox.setAction(false);
        outputSlide.startRetraction();
        while (opModeIsActive() && !outputSlide.isFinished()) {
            outputSlide.tick();
        }
    }
}
