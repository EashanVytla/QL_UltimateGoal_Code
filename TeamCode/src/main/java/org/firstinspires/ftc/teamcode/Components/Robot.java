package org.firstinspires.ftc.teamcode.Components;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer;
import org.firstinspires.ftc.teamcode.Vision.CameraTester;
import org.firstinspires.ftc.teamcode.Vision.RingDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.RingDetectionPipelineV2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class Robot {
    public final Vector2d ULTIMATE_GOAL_POS = new Vector2d(-12, 130.5);
    public Mecanum_Drive drive;
    public static Robot robotS = null;
    public ExpansionHubEx hub1;
    public ExpansionHubEx hub2;

    // The IMU sensor object
    private BNO055IMU imu;
    // State used for updating telemetry
    private Orientation angles;
    public S4T_Localizer localizer;
    private S4T_Encoder encoderLY;
    private S4T_Encoder encoderLX;
    private S4T_Encoder encoderRY;
    private S4T_Encoder encoderRX;
    private RevBulkData data;
    private RevBulkData data2;
    private HardwareMap hardwareMap;
    private Pose2d speedLimits;

    private Telemetry telemetry;

    //Todo: Once all robot hardware is on the main robot, make these their own classes
    public WobbleGoal wobbleGoal;
    public Shooter shooter;
    public Intake intake;

    OpenCvCamera webcam;
    RingDetectionPipelineV2 detector;
    Pose2d startPos = new Pose2d(0, 0, 0);

    public Robot(HardwareMap map, Telemetry telemetry){
        robotS = null;

        this.hardwareMap = map;
        this.telemetry = telemetry;

        hub1 = map.get(ExpansionHubEx.class, "Expansion Hub 173");
        hub2 = map.get(ExpansionHubEx.class, "Expansion Hub 2");

        encoderLY = new S4T_Encoder(map, "back_left");
        encoderLX = new S4T_Encoder(map, "front_left");
        encoderRY = new S4T_Encoder(map, "front_right");
        encoderRX = new S4T_Encoder(map, "back_right");


        drive = new Mecanum_Drive(map, telemetry);
        wobbleGoal = new WobbleGoal(map, telemetry, 0.5);
        shooter = new Shooter(map, telemetry);

        updateBulkData();

        localizer = new S4T_Localizer(telemetry, hardwareMap, data);
        intake = new Intake(hardwareMap);
    }

    public double getVelocityXMetersPerSecond(){
        double LX_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "front_left"));
        double RX_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "back_right"));
        double avg = (LX_velo + RX_velo)/2;
        return (avg/localizer.TICKS_TO_INCHES_STRAFE)/39.37;
    }

    public double getVelocityYMetersPerSecond(){
        double LY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "back_left"));
        double RY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "front_right"));
        double avg = (LY_velo + RY_velo)/2;
        return (avg/localizer.TICKS_TO_INCHES_VERT)/39.37;
    }

    public double getVelocityX(){
        double LX_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "front_left"));
        double RX_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "back_right"));
        double avg = (LX_velo + RX_velo)/2;
        return (avg/localizer.TICKS_TO_INCHES_STRAFE);
    }

    public double getVelocityY(){
        double LY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "back_left"));
        double RY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "front_right"));
        double avg = (LY_velo + RY_velo)/2;
        return (avg/localizer.TICKS_TO_INCHES_VERT);
    }

    public void setStartPose(Pose2d startPos){
        this.startPos = startPos;
    }

    public void initializeWebcam(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detector = new RingDetectionPipelineV2();
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public RevBulkData getData(){
        return data;
    }

    public RevBulkData getData2(){
        return data2;
    }

    public int getRingStackCase(){
        return detector.getAnalysis();
    }

    public static Robot getInstance(HardwareMap map, Telemetry telemetry){
        if(robotS == null)
        {
            robotS = new Robot(map, telemetry);
        }

        return robotS;
    }

    public void stop(){
        robotS = null;
    }

    public void updateBulkData(){
        data = hub1.getBulkInputData();
        data2 = hub2.getBulkInputData();
        shooter.setData(data2, data);
    }

    public void updatePos(){
        encoderLX.update(data);
        encoderLY.update(data);
        encoderRX.update(data);
        encoderRY.update(data);
        localizer.update(getRawLeft_X_Dist(), getRawLeft_Y_Dist(), getRawRight_X_Dist(), getRawRight_Y_Dist(), getVelocityX(), getVelocityY(), data);
    }

    public double getLeft_X_Dist(){
        return encoderLX.getDist();
    }

    public double getRight_X_Dist(){
        return encoderRX.getDist();
    }

    public double getLeft_Y_Dist(){
        return encoderLY.getDist();
    }

    public double getRight_Y_Dist(){
        return encoderRY.getDist();
    }

    public double getRawLeft_X_Dist(){
        return encoderLX.distance;
    }

    public double getRawRight_X_Dist(){
        return encoderRX.distance;
    }

    public double getRawLeft_Y_Dist(){
        return encoderLY.distance;
    }

    public double getRawRight_Y_Dist(){
        return encoderRY.distance;
    }

    public Pose2d getPos(){
        return new Pose2d(localizer.getPose().getX() + startPos.getX(), localizer.getPose().getY() + startPos.getY(), localizer.getPose().getHeading() + startPos.getHeading());
    }

    public Pose2d getStartPos(){
        return startPos;
    }

    public double angleWrap(double angle){
        return (angle + (2 * Math.PI)) % (2 * Math.PI);
    }

    public Bitmap getWebcamImage(){
        return detector.getImage();
    }

    public void stopWebcam(){
        webcam.stopStreaming();
    }

    public void initGyro(){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void startGyro(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void GoTo(Pose2d pose, Pose2d speedLimits){
        updateGoTo(pose, speedLimits);
    }

    public void GoTo(double x, double y, double heading, double maxspeed_x, double maxspeed_y, double maxspeed_z){
        updateGoTo(new Pose2d(x, y, heading), new Pose2d(maxspeed_x, maxspeed_y, maxspeed_z));
    }

    public void setAngle(double heading){
        localizer.setHeading(heading);
    }

    private void updateGoTo(Pose2d pose, Pose2d speedLimits){
        drive.goToPoint(pose, getPos(), speedLimits.getX(), speedLimits.getY(), speedLimits.getHeading());
        telemetry.addData("Position: ", getPos());
        telemetry.addData("Target Position: ", pose);
        drive.write();
        updatePos();
    }

    public double getGyro(){
        return angles.firstAngle;
    }

    public double getGryoWrapped()
    {
        return angleWrap(angles.firstAngle);
    }

    public void updateGyro(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
