package org.firstinspires.ftc.teamcode.Odometry;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.openftc.revextensions2.RevBulkData;

@Config
public class S4T_Localizer {
    //WORKED FOR VERY LONG TIME: 2739.9319227985241529292184283395;
    public static double TRACK_WIDTH1 = 2751.221712973584;//2754.2217129735839800225467700795;//2752.5847407743298321228446875276; //2748.4333315465458503161338711337;//2743.9424860098757;//2737.9424860098754612321073812974;//2747.2530501807513383745870814547;//2744.9453035059188560059382668858;//2739.9319227985241529292184283395;//2742.1772557701833430093304257463;//13.653342515840303278727731562382;//13.629789982152818111428120849052;//13.617612893489945808623743902362;//13.581490658183012723991930114595;

    public static double TRACK_WIDTH2 = 2441.750981253132; //1384.1705400702137351819820900508;//1381.7036384522893574775643917185;//1366.4406794097765947773284388111;//1384.0383147856;//1375.0191308424297533752712736568;//1375.2578632570675963789245993019;//1371.1198347366783176489336214542;//1362.4458903381700218495294563504;//1367.9367358748404109335559461868//6.8508849857360350014568370251882;//6.8125242936766372831876532920797;//6.8542971111369223086049488009311;

    public static double AUX_WIDTH = 3.4254424928680174;
    private double EPILSON = 0.00001;
    private static Pose2d mypose = new Pose2d(0, 0, 0);
    double prevheading = 0;

    double prevx = 0;
    double prevy = 0;

    double prevely = 0;
    double prevery = 0;
    double prevelx = 0;
    double preverx = 0;

    double prevelyRaw = 0;
    double preveryRaw = 0;
    double prevelxRaw = 0;
    double preverxRaw = 0;

    public static double heading = 0;
    Telemetry telemetry;
    public static double k_strafe = TRACK_WIDTH1/TRACK_WIDTH2;
    public static double k_vert = TRACK_WIDTH2/TRACK_WIDTH1;
    public double TICKS_TO_INCHES_VERT = 201.67339734597755609;
    public double TICKS_TO_INCHES_STRAFE = 335.381388888888888;//503.94791666666666666666666666667;

    public static double clipping_strafe = 0;
    public static double clipping_vert = 0;

    float CaseSwitchEPLSN = 0.3f;

    public Vector2d OFFSET_FROM_CENTER = new Vector2d(-48, -55);

    KalmanFilter filter;
    private static T265Camera slamra;
    private double odoCovariance = 0.01;
    HardwareMap hardwareMap;
    private Pose2d kalmanFilteredPos = new Pose2d(0, 0, 0);
    public AnalogGyro gyro;
    public OneDimensionlKalmanFilter oneDimensionlKalmanFilter;

    public S4T_Localizer(Telemetry telemetry, HardwareMap hardwareMap, RevBulkData data){
        this.telemetry = telemetry;
        filter = new KalmanFilter(telemetry);
        this.hardwareMap = hardwareMap;
        gyro = new AnalogGyro(hardwareMap);
        oneDimensionlKalmanFilter = new OneDimensionlKalmanFilter();
        gyro.update(data);
        gyro.reset();

        //todo: uncomment for kalman filter
        /*if(slamra == null){
            try{
                slamra = new T265Camera(new Transform2d(), odoCovariance, hardwareMap.appContext);
            }catch (Exception e){
                slamra = null;
                telemetry.addData("LOL","Couldn't find the camera... Trying again...");
            }
        }

        if(!slamra.isStarted()){
            slamra.start();
        }

        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0, 0)));*/
        //todo: uncomment for kalman filter
    }

    public void stopCamera(){
        slamra.stop();
    }

    public Pose2d getT265Pose(){
        Translation2d translation = new Translation2d(0, 0);
        Rotation2d rotation = new Rotation2d(0, 0);

        if(slamra == null){
            telemetry.addData("IT IS NULL", "IT IS NULL");
            try{
                slamra = new T265Camera(new Transform2d(), odoCovariance, hardwareMap.appContext);
                slamra.start();
            }catch (Exception e){
                slamra = null;
                telemetry.addData("ERROR","Couldn't find the camera... Trying again...");
            }
        }else{
            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if (up == null) return new Pose2d(0, 0, 0);

            translation = new Translation2d(up.pose.getTranslation().getX() * 39.37, up.pose.getTranslation().getY() * 39.37);
            rotation = up.pose.getRotation();
        }

        return new Pose2d(translation.getX(), translation.getY(), rotation.getRadians());
    }

    public Pose2d getKalmanFilteredPos(){
        return kalmanFilteredPos;
    }

    enum State{
        STRAFE,
        VERTICAL
    }
    State OdometryCase = State.VERTICAL;

    public double wf = 1;
    public double ws = 1;
    double dtheta = 0;
    public Pose2d dashboardPos = new Pose2d(0, 0, 0);
    Pose2d prevT265Pos = new Pose2d(0, 0, 0);
    private double prevTime = 0;

    public double omega = 0;
    public double prevHeadingGyro = 0;

    double k_heading = 0;

    public void startTime(RevBulkData data){
        gyro.update(data);
        prevHeadingGyro = gyro.getAngleCorrected();
        prevTime = SystemClock.uptimeMillis();
    }

    public void update(double elxRaw, double elyRaw, double erxRaw, double eryRaw, double xVelo, double yVelo, RevBulkData data){
        double y = ((elyRaw + eryRaw)/2) / TICKS_TO_INCHES_VERT;
        double x = ((elxRaw + erxRaw)/2) / TICKS_TO_INCHES_STRAFE;
        //double x = erx;
        double dy = y - prevy;
        double dx = x - prevx;
        //double dx = (erx - prevx) - (AUX_WIDTH * dtheta);

        prevx = x;
        prevy = y;

        double dElyRaw = elyRaw - prevelyRaw;
        double dEryRaw = eryRaw - preveryRaw;
        double dElxRaw = elxRaw - prevelxRaw;
        double dErxRaw = erxRaw - preverxRaw;

        prevelx = elxRaw;
        prevely = elyRaw;
        preverx = erxRaw;
        prevery = eryRaw;

        prevelxRaw = elxRaw;
        prevelyRaw = elyRaw;
        preverxRaw = erxRaw;
        preveryRaw = eryRaw;

        //double dthetastrafe = (dErx - dElx) / TRACK_WIDTH2;
        //double dthetavert = (dEry - dEly) / TRACK_WIDTH1;

        double dthetastrafe = (dErxRaw - dElxRaw) / TRACK_WIDTH2;
        double dthetavert = (dEryRaw - dElyRaw) / TRACK_WIDTH1;

        dtheta = weightedTheta(dx, dy, dthetavert, dthetastrafe);
        heading %= 2 * Math.PI;
        //double dtheta = nonweightedTheta(dx, dy, dthetavert, dthetastrafe);

        heading += dtheta;
        heading %= 2 * Math.PI;

        Vector2 myVec = ConstantVelo(dy, dx, prevheading, dtheta);
        prevheading = heading;

        mypose = mypose.plus(new Pose2d(myVec.x, myVec.y, dtheta));
        mypose = new Pose2d(mypose.getX(), mypose.getY(), (Math.toRadians(360) - heading) % Math.toRadians(360));

        //todo: uncomment for Kalman Filter
        /*Pose2d t265_pos = new Pose2d(getT265Pose().getY(), -getT265Pose().getX(), getT265Pose().getHeading());

        telemetry.addData("T265 Pos", t265_pos);
        telemetry.addData("My Position", mypose);

        double dt = (SystemClock.uptimeMillis() - prevTime)/1000.0;
        //Kalman Filter
        //_____________________________________________________________________________________
        //CORRECT
        filter.correct(new SimpleMatrix(3, 1, true, new double[]{t265_pos.getX(), t265_pos.getY(), t265_pos.getHeading()}));

        kalmanFilteredPos = filter.getEstimate();

        //PREDICT
        omega = slamra.getLastReceivedCameraUpdate().velocity.omegaRadiansPerSecond;
        telemetry.addData("Omega", omega);
        filter.predict(new SimpleMatrix(3, 1, true, new double[]{mypose.getX(), mypose.getY(), mypose.getHeading()}), new Pose2d(xVelo, yVelo, omega), dt, kalmanFilteredPos.getHeading());
        //______________________________________________________________________________________
        //End of Kalman Filter
        prevTime = SystemClock.uptimeMillis();

        telemetry.addData("Filtered Position", kalmanFilteredPos);*/
        //todo: uncomment for kalman filter

        gyro.update(data);
        telemetry.addData("Gyro", Math.toDegrees(gyro.getAngleCorrected()));

        ////////////////////////////////////////////////////////////////////
        oneDimensionlKalmanFilter.correct((Math.toRadians(360) - heading) % Math.toRadians(360));

        double FilteredHeading = oneDimensionlKalmanFilter.state;

        double dt = (SystemClock.uptimeMillis() - prevTime)/1000.0;
        oneDimensionlKalmanFilter.predict(gyro.getAngleCorrected(), dt, (gyro.getAngleCorrected() - prevHeadingGyro)/dt);
        prevHeadingGyro = gyro.getAngleCorrected();
        prevTime = SystemClock.uptimeMillis();

        telemetry.addData("Refresh Rate", 1/dt);

        telemetry.addData("Kalman Filtered Heading", Math.toDegrees(FilteredHeading));
        ////////////////////////////////////////////////////////////////////

        dashboardPos = new Pose2d(mypose.getY() + OFFSET_FROM_CENTER.getY(), -mypose.getX() + OFFSET_FROM_CENTER.getX(), (2 * Math.PI) - mypose.getHeading());

        telemetry.addData("Vertical Heading", Math.toDegrees(-(elyRaw - eryRaw)/TRACK_WIDTH1) % (360));
        telemetry.addData("Strafe Heading", Math.toDegrees(-(erxRaw - elxRaw)/TRACK_WIDTH2) % (360));
    }

    public void reset(){
        mypose = new Pose2d(0, 0, 0);
        heading = 0;
    }

    public void reset(Pose2d pos){
        mypose = pos;
        heading = pos.getHeading();
    }

    public double angleWrap(double angle){
        return ((2 * Math.PI) + angle) % (2 * Math.PI);
    }

    public double weightedTheta(double dx, double dy, double dthetavert, double dthetastrafe){
        determineWeights(dx, dy);

        if(Math.abs(wf) <= clipping_vert){
            wf = 0;
        }

        if(Math.abs(ws) <= clipping_strafe){
            ws = 0;
        }


        double value = 0;
        double total = wf + ws;
        if(total != 0){
            value = ((wf * dthetavert) + (ws * -dthetastrafe))/total;
        }else{
            value = (dthetavert - dthetastrafe)/2;
            //value = dthetavert;
        }

        return value;
    }

    public double nonweightedTheta(double dx, double dy, double dthetavert, double dthetastrafe){
        OdometryCase = determineCase(dx, dy, dthetavert, dthetastrafe);

        if(OdometryCase == State.STRAFE){
            return dthetastrafe;
        }else{
            return dthetavert;
        }
    }

    private double prevdx = 0;
    private double prevdy = 0;

    public void setHeading(double heading){
        this.heading = heading;
    }

    public static double normalizationFactor = 8;

    public void determineWeights(double dx, double dy){
        //wf = 1;
        //ws = 0;

        double highest = 0;

        /*if(Math.abs(dx) > Math.abs(dy)){
            highest = dx;
        }else{
            highest = dy;
        }*/

        //highest /= 8;

        //Todo: Test is weights and decision is actually working
        //Todo: If they are not working, then try getting rid of the highest logic, and just dividing by 8
        double total = dx + dy;
        /*if(highest != 0){
            dx /= highest;
            dy /= highest;
        }*/

        double mydx = (dx + prevdx)/2;
        double mydy = (dy + prevdy)/2;

        if(total != 0){
            mydx /= total;
            mydy /= total;
            mydx *= normalizationFactor;
            mydy *= normalizationFactor;
        }

        //If dx is higher, wf is lower and vice versa
        if(mydx != 0) {
            wf = Math.pow(Math.E, -k_strafe * Math.abs(mydx));
        }

        //If dy is high, ws is lower and vice versa
        if(mydy != 0) {
            ws = Math.pow(Math.E, -k_vert * Math.abs(mydy));
        }

        prevdx = dx;
        prevdy = dy;
    }

    public State determineCase(double dx, double dy, double dthethavert, double dthetastrafe){
        if((dthethavert/2) >= Math.toRadians(0.01) || dthetastrafe >= Math.toRadians(0.01)){
            return State.VERTICAL;
        }else{
            if(Math.abs(dx) > Math.abs(dy)){
                return State.STRAFE;
            }else{
                return State.VERTICAL;
            }
        }
    }

    public Pose2d getPose(){
        return mypose;
    }

    private Vector2 circleUpdate(double dr, double dx, double dy, double dtheta){
        if(dtheta <= EPILSON){
            double sineTerm = 1.0 - dtheta * dtheta / 6.0;
            double cosTerm = dtheta / 2.0;

            return new Vector2(
                    sineTerm * dx - cosTerm * dy,
                    cosTerm * dx + sineTerm * dy
            );
        }else{
            double radius = (TRACK_WIDTH1/2) + (dr/dtheta);
            double strafe_radius = dy/dtheta;
            return new Vector2(
                    (radius * (1 - Math.cos(dtheta))) + (strafe_radius * Math.sin(dtheta)),
                    (Math.sin(dtheta) * radius) + (strafe_radius * (1 - Math.cos(dtheta)))
            );
        }
    }

    public Vector2 ConstantVelo(double delta_y, double delta_x, double prev_heading, double delta_theta){
        Pose2d RawRobotDelta = new Pose2d(delta_x, delta_y, delta_theta);

        double sinterm = 0;
        double costerm = 0;

        if(delta_theta <= EPILSON){
            sinterm = 1.0 - delta_theta * delta_theta / 6.0;
            costerm = delta_theta / 2.0;
        }else{
            sinterm = Math.sin(delta_theta) / delta_theta;
            costerm = (1 - Math.cos(delta_theta)) / delta_theta;
        }

        Vector2 FeildCentricDelta = new Vector2((sinterm * RawRobotDelta.getX()) - (costerm * RawRobotDelta.getY()), (costerm * RawRobotDelta.getX()) + (sinterm * RawRobotDelta.getY()));
        FeildCentricDelta.rotate(prev_heading);

        return FeildCentricDelta;
    }
}
