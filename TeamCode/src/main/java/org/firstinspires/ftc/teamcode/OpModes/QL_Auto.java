package org.firstinspires.ftc.teamcode.OpModes;

import android.os.PowerManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;
import org.opencv.core.Point;

import java.util.ArrayList;

@Autonomous
public class QL_Auto extends OpMode {
    Robot robot = null;
    int stage = 0;
    ElapsedTime time;
    int ring_case = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init(){
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();
    }

    @Override
    public void init_loop(){
        ring_case = robot.getRingStackCase();
        telemetry.addData("Ring Case", ring_case);
        telemetry.update();
    }

    @Override
    public void start(){
        time.startTime();
    }

    public void loop(){
        Pose2d CLEAR_STACK = new Pose2d(Positions.CLEAR_STACK.x, Positions.CLEAR_STACK.x, Positions.CLEAR_STACK_HEADING);
        Pose2d ZONE_1 = new Pose2d(Positions.ZONE_1.x, Positions.ZONE_1.x, Positions.ZONE_1_HEADING);
        Pose2d ZONE_2 = new Pose2d(Positions.ZONE_2.x, Positions.ZONE_2.x, Positions.ZONE_2_HEADING);
        Pose2d ZONE_3 = new Pose2d(Positions.ZONE_3.x, Positions.ZONE_3.x, Positions.ZONE_3_HEADING);
        Pose2d POWER_SHOTS = new Pose2d(Positions.POWER_SHOTS.x, Positions.POWER_SHOTS.x, Positions.POWER_SHOTS_HEADING);
        Pose2d WOBBLE_GOAL_2 = new Pose2d(Positions.WOBBLE_GOAL_2.x, Positions.CLEAR_STACK.x, Positions.WOBBLE_GOAL_2_HEADING);
        Pose2d CLEAR_STACK_2 = new Pose2d(Positions.CLEAR_STACK_2.x, Positions.CLEAR_STACK_2.x, Positions.CLEAR_STACK_2_HEADING);
        Pose2d PARK = new Pose2d(Positions.PARK.x, Positions.PARK.x, Positions.PARK_HEADING);

        robot.updateBulkData();
        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        switch (stage){
            case 0:
                //Grabbing the wobble goal

                if(time.time() >= 2.0){
                    robot.drive.setPower(0, 0, 0);
                    RobotMovement.resetIndex();
                    stage = 1;
                }else if(time.time() >= 1.5){
                    robot.GoTo(new Pose2d(0, 5, 0), new Pose2d(1.0, 0.3, 1.0));
                }else if(time.time() >= 0.5){
                    robot.wobbleGoal.clamp();
                }

                robot.wobbleGoal.down();

                if(stage == 1){
                    time.reset();
                }
                break;
            case 1:
                allPoints.add(new CurvePoint(0, 5, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(CLEAR_STACK, 1, 1, 15));
                if(ring_case == 0){
                    allPoints.add(new CurvePoint(ZONE_1, 1, 1, 15));

                    if(robot.getPos().vec().distTo(POWER_SHOTS.vec()) <= 0.8){
                        RobotMovement.resetIndex();
                        //stage = 2;
                    }else if(robot.getPos().vec().distTo(ZONE_1.vec()) <= 4){
                        robot.wobbleGoal.kick();
                    }
                }else if(ring_case == 1){
                    allPoints.add(new CurvePoint(ZONE_2, 1, 1, 15));

                    if(robot.getPos().vec().distTo(POWER_SHOTS.vec()) <= 0.8){
                        RobotMovement.resetIndex();
                        //stage = 2;
                    }else if(robot.getPos().vec().distTo(ZONE_2.vec()) <= 4){
                        robot.wobbleGoal.kick();
                    }
                }else{
                    allPoints.add(new CurvePoint(ZONE_3, 1, 1, 15));

                    if(robot.getPos().vec().distTo(POWER_SHOTS.vec()) <= 0.8){
                        RobotMovement.resetIndex();
                        //stage = 2;
                    }else if(robot.getPos().vec().distTo(ZONE_3.vec()) <= 4){
                        robot.wobbleGoal.kick();
                    }
                }
                allPoints.add(new CurvePoint(POWER_SHOTS, 1, 1, 15));
                break;
        }

        if(stage != 0){
            RobotMovement.followCurve(allPoints, robot, telemetry);
        }

        robot.wobbleGoal.write();

        dashboardTelemetry.addData("Stage", stage);
        dashboardTelemetry.addData("Time", time.time());
        dashboardTelemetry.update();

        telemetry.addData("Pos", robot.getPos());
        telemetry.addData("Error", robot.getPos().vec().distTo(robot.drive.target_pos.vec()));
        telemetry.update();
    }

}

@Config
class Positions {
    public static Point CLEAR_STACK = new Point(8, 24);
    public static Point ZONE_1 = new Point(13.464, 61.245);
    public static Point ZONE_2 = new Point(-5.923, 87.501);
    public static Point ZONE_3 = new Point(15.898, 106.822);
    public static Point POWER_SHOTS = new Point(-28.505, 46.955);
    public static Point WOBBLE_GOAL_2 = new Point(0, 0);
    public static Point CLEAR_STACK_2 = new Point(0, 0);
    public static Point PARK = new Point(0, 0);

    public static double CLEAR_STACK_HEADING = 0;
    public static double ZONE_1_HEADING = 0;
    public static double ZONE_2_HEADING = 0;
    public static double ZONE_3_HEADING = 0;
    public static double POWER_SHOTS_HEADING = Math.toRadians(180);
    public static double WOBBLE_GOAL_2_HEADING = 0;
    public static double CLEAR_STACK_2_HEADING = 0;
    public static double PARK_HEADING = 0;
}
