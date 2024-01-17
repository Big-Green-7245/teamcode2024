package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Webcams.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.TelemetryWrapper;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Navigation {
    private double[] currentPos;
    private double currentBearing;

    private double[] lastDriveTrainPos;

    private double[] lastAprilTagPos;

    private double lastDriveTrainBearing;

    private DriveTrain driveTrain;

    private AprilTagWebcam tagCam;

    BNO055IMU               imu;
    private double initGyroReading;
    public Navigation (double[] initPos, double initBearing, LinearOpMode opMode, HardwareMap map){
        currentPos = initPos;
        lastAprilTagPos = initPos;
        lastDriveTrainPos = initPos;
        lastDriveTrainBearing = initBearing;
        currentBearing = initBearing;
        driveTrain = new DriveTrain(opMode);
        driveTrain.init(map);
        tagCam = new AprilTagWebcam();
        tagCam.init(map);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (opMode.opModeInInit() && !imu.isGyroCalibrated()){
            TelemetryWrapper.setLine(2, "Calibrating gyro...");
        }
        TelemetryWrapper.setLine(2, "Gyro calibrated");
        initGyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initBearing;
    }

    public double[] getCurrentPosition(){
        return currentPos;
    }

    public double getCurrentBearing() {
        return currentBearing;
    }

    public double getGyroBearing(){
        return ((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initGyroReading%360)+360)%360;
    }

    public double magnitude(double[] vector){
        double sum = 0;
        for (double component:vector){
            sum += Math.pow(component, 2);
        }
        return Math.sqrt(sum);
    }
    public double[] getCurrentPos(){
        lastAprilTagPos = tagCam.detectIter(currentBearing);
        if (tagCam.isDetecting){
            return lastAprilTagPos;
        }
        return lastDriveTrainPos;
    }

    public void setBearing(double targetBearing){
        targetBearing = (targetBearing%360)+360;
        double dir =((targetBearing-currentBearing < 180)? -1 : 1);
        driveTrain.move(0, 0, 0.1*dir, 1);
        while (Math.abs(targetBearing - currentBearing) > 0.5){
            TelemetryWrapper.setLine(6, "Target Bearing: " + targetBearing + " | " + "Current Bearing: " + currentBearing);
            currentBearing = getGyroBearing();
        }
        driveTrain.stopStayInPlace();
    }


    public void MoveToPosDirect(double[] targetPos){
        double targetBearing = Math.toDegrees(Math.atan2(targetPos[1]-currentPos[1], targetPos[0]-currentPos[0]));
        setBearing(targetBearing);
        double[] lastEncoderPos = driveTrain.getEncPos();
        double[] displacement = new double[]{targetPos[0]- currentPos[0], targetPos[1] - currentPos[1]};
        driveTrain.move(0,0.7, 0, 1);
        while (magnitude(displacement) > 2){
            TelemetryWrapper.setLine(5, "x: "+ currentPos[0] + "| y: " + currentPos[1] + "| theta: " + currentBearing);
            double[] currentEncPos = driveTrain.getEncPos();
            currentBearing = getGyroBearing();
            lastAprilTagPos = tagCam.detectIter(currentBearing);
            if (tagCam.isDetecting){
                currentPos = lastAprilTagPos;
                TelemetryWrapper.setLine(7, "CAM");
            }else {
                TelemetryWrapper.setLine(7, "ENC");
                double delta = 0;
                for (int i = 0; i < lastEncoderPos.length; i++) {
                    delta += Math.abs(currentEncPos[i] - lastEncoderPos[i]) / driveTrain.COUNTS_PER_INCH;
                }
                delta /= lastEncoderPos.length;
                currentPos = new double[]{currentPos[0] + Math.cos(Math.toRadians(currentBearing)) * delta, currentPos[1] + Math.sin(Math.toRadians(currentBearing)) * delta};
            }
            displacement = new double[]{targetPos[0]- currentPos[0], targetPos[1] - currentPos[1]};
            lastEncoderPos = currentEncPos;
        }
        driveTrain.stopStayInPlace();
        strafeToPos(targetPos);
    }

    public void strafeToPos(double[] targetPos){
        double[] displacement = new double[]{targetPos[0]- currentPos[0], targetPos[1] - currentPos[1]};
        double dx = Math.cos(Math.toRadians(currentBearing-90))*(displacement[0]) - Math.sin(Math.toRadians(currentBearing-90))*(displacement[1]);
        double dy = Math.sin(Math.toRadians(currentBearing-90))*(displacement[0]) + Math.cos(Math.toRadians(currentBearing-90))*(displacement[1]);

        driveTrain.translate(0.4, dx, 0, 0, 10);
        driveTrain.translate(0.4, 0, dy, 0, 10);
        lastAprilTagPos = tagCam.detectIter(currentBearing);
        if(tagCam.isDetecting){
            currentPos = lastAprilTagPos;
            TelemetryWrapper.setLine(7, "CAM");
        }else{
            currentPos = targetPos;
            TelemetryWrapper.setLine(7, "ENC");
        }
        driveTrain.stopStayInPlace();
    }

    public void MoveToPosAtAngle(double[] targetPos, double angle){
        double directAngle = Math.toDegrees(Math.atan2(targetPos[1]-currentPos[1], targetPos[0]-currentPos[0]));
        directAngle = (360+directAngle)%360;

    }

}