package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.ftccommon.FtcEventLoop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.PurePusuit.RCOffset;

@Config
public class S4T_Localizer {
    //WORKED FOR VERY LONG TIME: 2739.9319227985241529292184283395;
    public static double TRACK_WIDTH1 = 2743.9424860098757;//2737.9424860098754612321073812974;//2747.2530501807513383745870814547;//2744.9453035059188560059382668858;//2739.9319227985241529292184283395;//2742.1772557701833430093304257463;//13.653342515840303278727731562382;//13.629789982152818111428120849052;//13.617612893489945808623743902362;//13.581490658183012723991930114595;

    //todo: This is theoretical trackwidth from old trackwidth... PLEASE TUNE
    public static double TRACK_WIDTH2 = 1384.0383147856;//1375.0191308424297533752712736568;//1375.2578632570675963789245993019;//1371.1198347366783176489336214542;//1362.4458903381700218495294563504;//1367.9367358748404109335559461868//6.8508849857360350014568370251882;//6.8125242936766372831876532920797;//6.8542971111369223086049488009311;

    public static double AUX_WIDTH = 3.4254424928680174;
    private double EPILSON = 0.00001;
    private Pose2d myposeTicks = new Pose2d(0, 0, 0);
    private Pose2d myposeIn = new Pose2d(0, 0, 0);
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

    double heading = 0;
    Telemetry telemetry;
    public static double k_strafe = 1.0;
    public static double k_vert = 0.75;
    public double TICKS_TO_INCHES = 200.33577083;

    float CaseSwitchEPLSN = 0.3f;


    private Vector2d OFFSET_FROM_CENTER = new Vector2d(-48, -55);

    public S4T_Localizer(Telemetry telemetry){
        this.telemetry = telemetry;
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


    public void update(double elxRaw, double elyRaw, double erxRaw, double eryRaw){
        double y = (elyRaw + eryRaw)/2;
        double x = (elxRaw + erxRaw)/2;
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
        //double dtheta = nonweightedTheta(dx, dy, dthetavert, dthetastrafe);

        heading += dtheta;
        heading %= 2 * Math.PI;

        Vector2 myVec = ConstantVelo(dy, dx, prevheading, dtheta);
        prevheading = heading;

        myposeTicks = myposeTicks.plus(new Pose2d(myVec.x, myVec.y, dtheta));
        myposeTicks = new Pose2d(myposeTicks.getX(), myposeTicks.getY(), (Math.toRadians(360) - heading) % Math.toRadians(360));

        myposeIn = new Pose2d(myposeTicks.getX() / TICKS_TO_INCHES, myposeTicks.getY() / TICKS_TO_INCHES, myposeTicks.getHeading());

        dashboardPos = new Pose2d(myposeIn.getY() + OFFSET_FROM_CENTER.getY(), -myposeIn.getX() + OFFSET_FROM_CENTER.getX(), (2 * Math.PI) - myposeIn.getHeading());

        telemetry.addData("Vertical Heading", Math.toDegrees(-(elyRaw - eryRaw)/TRACK_WIDTH1) % (360));
        telemetry.addData("Strafe Heading", Math.toDegrees(-(erxRaw - elxRaw)/TRACK_WIDTH2) % (360));

        /*DashboardUtil.drawRobot(fieldOverlay, dashboardPos);
        packet.put("pos", mypose);
        packet.put("Vertical Heading: ", Math.toDegrees(-(elyRaw - eryRaw)/TRACK_WIDTH1) % (360));
        packet.put("Strafe Heading: ", Math.toDegrees(-(erxRaw - elxRaw)/TRACK_WIDTH2) % (360));
        dashboard.sendTelemetryPacket(packet);*/
    }

    public double angleWrap(double angle){
        return (-(2 * Math.PI) - angle) % (2 * Math.PI);
    }

    public double weightedTheta(double dx, double dy, double dthetavert, double dthetastrafe){
        determineWeights(dx, dy);

        //The trash version
        /*double total = wf + ws;
        wf /= total;
        ws /= total;

        telemetry.addData("Case: ", OdometryCase);
        if(ws > wf){
            OdometryCase = State.STRAFE;
            return dthetastrafe;
        }else{
            OdometryCase = State.VERTICAL;
            return dthetavert;
        }*/

//        //todo: take this out!!!!
//        wf = 1;
//        ws = 0;

        if(wf <= 0.075){
            wf = 0;
        }

        if(ws <= 0.075){
            ws = 0;
        }


        double value = 0;
        double total = wf + ws;
        if(total != 0){
            value = ((wf * dthetavert) + (ws * -dthetastrafe))/total;
        }else{
            value = dthetavert;
        }

        telemetry.addData("Weight Forward", wf);
        telemetry.addData("Weight strafe", ws);

        return value;
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

        //dy *= 0.49818137023672532471239330234681;

        double mydx = (dx + prevdx)/2;
        double mydy = (dy + prevdy)/2;

        if(total != 0){
            mydx /= total;
            mydy /= total;
            mydx *= normalizationFactor;
            mydy *= normalizationFactor;
        }

        Vector2d vec = new Vector2d(dashboardPos.getX() + 15, dashboardPos.getY());
        vec.rotated(heading);
        vec = vec.rotated(Math.atan2(mydy, mydx));

        //fieldOverlay.strokeLine(dashboardPos.getX(), dashboardPos.getY(), vec.getX(), vec.getY());

        //packet.put("dx", dx);
        //packet.put("dy", dy);

        //If dx is higher, wf is lower and vice versa
        if(mydx != 0) {
            wf = Math.pow(Math.E, -k_strafe * Math.abs(mydx));
        }
        //wf = 1;

        //If dy is high, ws is lower and vice versa
        if(mydy != 0) {
            ws = Math.pow(Math.E, -k_vert * Math.abs(mydy));
        }
        //ws = 0;

        //packet.put("ws", ws);
        //packet.put("wf", wf);

        /*telemetry.addData("weight forward: ", wf);
        telemetry.addData("weight strafe: ", ws);
        telemetry.addData("weight total", wf + ws);*/
        prevdx = dx;
        prevdy = dy;
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
        return myposeIn;
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
