package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.FtcEventLoop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.PurePusuit.RCOffset;

@Config
public class S4T_Localizer {
    public static double TRACK_WIDTH1 = 13.629789982152818111428120849052;//13.617612893489945808623743902362;//13.581490658183012723991930114595;
    public static double TRACK_WIDTH2 = 6.8514614115633455971196003213002;//6.8542971111369223086049488009311;
    private double EPILSON = 0.00001;
    private Pose2d mypose = new Pose2d(0, 0, 0);
    double prevheading = 0;
    double prevx = 0;
    double prevy = 0;
    double prevely = 0;
    double prevery = 0;
    double prevelx = 0;
    double preverx = 0;
    double heading = 0;
    Telemetry telemetry;
    float k_strafe = 0.8f;
    float k_vert = 1;

    float CaseSwitchEPLSN = 0.3f;

    public S4T_Localizer(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    enum State{
        STRAFE,
        VERTICAL
    }
    State OdometryCase = State.VERTICAL;

    double wf = 1;
    double ws = 1;

    public void update(double elx, double ely, double erx, double ery){
        double y = (ely + ery)/2;
        double x = (elx + erx)/2;
        double dy = y - prevy;
        double dx = x - prevx;

        prevx = x;
        prevy = y;

        double dEly = ely - prevely;
        double dEry = ery - prevery;
        double dElx = elx - prevelx;
        double dErx = erx - preverx;

        prevelx = elx;
        prevely = ely;
        preverx = erx;
        prevery = ery;

        double dthetastrafe = -(dErx - dElx) / TRACK_WIDTH2;
        double dthetavert = -(dEry - dEly) / TRACK_WIDTH1;

        double dtheta = weightedTheta(dx, dy, dthetavert, dthetastrafe);
        //double dtheta = nonweightedTheta(dx, dy, dthetavert, dthetastrafe);

        heading += dtheta;
        heading %= 2 * Math.PI;

        Vector2 myVec = ConstantVelo(dy, dx, prevheading, dtheta);
        prevheading = heading;

        mypose = mypose.plus(new Pose2d(myVec.x, myVec.y, dtheta));
        mypose = new Pose2d(mypose.getX(), mypose.getY(), heading);

        telemetry.addData("Vertical Heading: ", Math.toDegrees(-(ery - ely)/ TRACK_WIDTH1) % (360));
        telemetry.addData("Strafe Heading: ", Math.toDegrees(-(erx - elx)/ TRACK_WIDTH2) % (360));
    }

    public double weightedTheta(double dx, double dy, double dthetavert, double dthetastrafe){
        determineWeights(dx, dy);

        telemetry.addData("Case: ", OdometryCase);
        if(ws > wf){
            OdometryCase = State.STRAFE;
            return dthetastrafe;
        }else{
            OdometryCase = State.VERTICAL;
            return dthetavert;
        }
        //return ((wf * dthetavert) + (ws * dthetastrafe)) / (wf + ws);
    }

    public double nonweightedTheta(double dx, double dy, double dthetavert, double dthetastrafe){
        OdometryCase = determineCase(dx, dy, dthetavert, dthetastrafe);

        telemetry.addData("Case: ", OdometryCase);

        if(OdometryCase == State.STRAFE){
            return dthetastrafe;
        }else{
            return dthetavert;
        }
    }

    public void determineWeights(double dx, double dy){
        wf = 1;
        ws = 0;

        double highest = 0;

        if(Math.abs(dx) > Math.abs(dy)){
            highest = dx;
        }else{
            highest = dy;
        }

        highest /= 8;

        //Todo: Test is weights and decision is actually working
        //Todo: If they are not working, then try getting rid of the highest logic, and just dividing by 8
        if(highest != 0){
            dx /= highest;
            dy /= highest;
        }

        //If dx is higher, wf is lower and vice versa
        if(dx != 0) {
            wf = Math.pow(Math.E, -k_vert * Math.abs(dx));
        }
        //wf = 1;

        //If dy is high, ws is lower and vice versa
        if(dy != 0) {
            ws = Math.pow(Math.E, -k_strafe * Math.abs(dy));
        }
        //ws = 0;

        telemetry.addData("weight forward: ", wf);
        telemetry.addData("weight strafe: ", ws);
        telemetry.addData("weight total", wf + ws);
    }

    public State determineCase(double dx, double dy, double dthethavert, double dthetastrafe){
        if(dthethavert >= Math.toRadians(0.01) || dthetastrafe >= Math.toRadians(0.01)){
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

    private Vector2 circleUpdate(RCOffset offset){
        if(offset.dtheta <= EPILSON){
            double sineTerm = 1.0 - offset.dtheta * offset.dtheta / 6.0;
            double cosTerm = offset.dtheta / 2.0;

            return new Vector2(
                    sineTerm * offset.x - cosTerm * offset.y,
                    cosTerm * offset.x + sineTerm * offset.y
            );
        }else{
            double radius = (TRACK_WIDTH1/2) + (offset.right/offset.dtheta);
            double strafe_radius = offset.y/offset.dtheta;
            return new Vector2(
                    (radius * (1 - Math.cos(offset.dtheta))) + (strafe_radius * Math.sin(offset.dtheta)),
                    (Math.sin(offset.dtheta) * radius) + (strafe_radius * (1 - Math.cos(offset.dtheta)))
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
