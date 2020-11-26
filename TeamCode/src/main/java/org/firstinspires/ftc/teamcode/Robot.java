package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class Robot{
    Mecanum_Drive drive;
    public static Robot robot = null;
    public ExpansionHubEx hub1;
    // The IMU sensor object
    private BNO055IMU imu;
    // State used for updating telemetry
    private Orientation angles;
    private S4T_Localizer localizer;
    private S4T_Encoder encoderLY;
    private S4T_Encoder encoderLX;
    private S4T_Encoder encoderRY;
    private S4T_Encoder encoderRX;
    private RevBulkData data;
    private HardwareMap hardwareMap;
    private Pose2d speedLimits;

    Pose2d setPoint;
    Telemetry telemetry;

    //Todo: Once all robot hardware is on the main robot, make these their own classes
    WobbleGoal wobbleGoal;
    Shooter shooter;
    Intake intake;


    private Robot(HardwareMap map, Telemetry telemetry){
        robot = null;

        this.hardwareMap = map;
        this.telemetry = telemetry;

        hub1 = map.get(ExpansionHubEx.class, "Expansion Hub 173");

        encoderLY = new S4T_Encoder(map, "back_left");
        encoderLX = new S4T_Encoder(map, "front_left");
        encoderRY = new S4T_Encoder(map, "front_right");
        encoderRX = new S4T_Encoder(map, "back_right");

        drive = new Mecanum_Drive(map, telemetry);
        wobbleGoal = new WobbleGoal(map, telemetry, 0.5);
        shooter = new Shooter(map, telemetry);

        setPoint  = new Pose2d(0, 0, 0);

        localizer = new S4T_Localizer(telemetry);
        intake = new Intake(hardwareMap);
    }

    public static Robot getInstance(HardwareMap map, Telemetry telemetry){
        if( robot == null)
        {
            robot = new Robot(map, telemetry);
        }

        return robot;
    }

    public void stop(){
        robot = null;
    }

    private void updateBulkData(){
        data = hub1.getBulkInputData();
    }

    public void updatePos(){
        updateBulkData();
        encoderLX.update(data);
        encoderLY.update(data);
        encoderRX.update(data);
        encoderRY.update(data);
        localizer.update(getLeft_X_Dist(), getLeft_Y_Dist(), getRight_X_Dist(), getRight_Y_Dist());
    }

    public double getLeft_X_Dist(){
        return -encoderLX.getDist();
    }

    public double getRight_X_Dist(){
        return -encoderRX.getDist();
    }

    public double getLeft_Y_Dist(){
        return encoderLY.getDist();
    }

    public double getRight_Y_Dist(){
        return encoderRY.getDist();
    }

    public Pose2d getPos(){
        return new Pose2d(localizer.getPose().getX(), localizer.getPose().getY(), localizer.getPose().getHeading());
    }

    public double angleWrap(double angle){
        return (angle + (2 * Math.PI)) % (2 * Math.PI);
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

    public void GoTo(Pose2d pose, Pose2d speedLimits){
        updateGoTo(pose, speedLimits);
    }

    public void GoTo(double x, double y, double heading, double maxspeed_x, double maxspeed_y, double maxspeed_z){
        updateGoTo(new Pose2d(x, y, heading), new Pose2d(maxspeed_x, maxspeed_y, maxspeed_z));
    }

    private void updateGoTo(Pose2d pose, Pose2d speedLimits){
        drive.goToPoint(setPoint, getPos(), speedLimits.getX(), speedLimits.getY(), speedLimits.getHeading());
        telemetry.addData("Position: ", getPos());
        telemetry.addData("Target Position: ", setPoint);
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
