package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous(name = "PP Tester")
public class GoToPoint_Tester extends OpMode {
    Robot robot;
    int run;

    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        robot = Robot.getInstance(hardwareMap, telemetry);
        run = 0;
    }

    @Override
    public void stop() {
        robot.stop();
    }

    @Override
    public void loop() {
        telemetry.addData("Pose: ", robot.getPos().toString());

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        //telemetry.addData("Pose: ", Robot.getPose().toString());

        //telemetry.addData("Run: ", String.valueOf(run));

        if(run == 0){
            //telemetry.addData("dist", String.valueOf(Robot.getPose().distanceToVector(new Vector3(50, 55))));
            if(robot.getPos().vec().distTo(new Vector2d(-35, 55)) <= 2.0){
                //telemetry.addData("Time", String.valueOf(time.timeSeconds()));
                if(time.time() >= 1){
                    run = 1;
                    RobotMovement.resetIndex();
                }
            }else{
                allPoints.add(new CurvePoint(0, 0, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(-5, 105, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(-35, 55, 1, 1, 15, 0));

                time.reset();
            }
        }else if(run == 1){
            if(robot.getPos().vec().distTo(new Vector2d(-29, 2)) <= 2.0){
                telemetry.addData("Time", String.valueOf(time.time()));
                if(time.time() >= 1){
                    run = 2;
                    RobotMovement.resetIndex();
                }
            }else{
                allPoints.add(new CurvePoint(-35, 55, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(-48, 22, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(-29, 2, 1, 1, 15, 0));
                time.reset();
            }
        }else if(run == 2){
            if(robot.getPos().vec().distTo(new Vector2d(-5, 105)) <= 2.0){
                //telemetry.addData("Time", String.valueOf(time.timeSeconds()));
                if(time.time() >= 1){
                    run = 3;
                    RobotMovement.resetIndex();
                }
            }else{
                allPoints.add(new CurvePoint(-29, 2, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(-48, 22, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(-5, 105, 1, 1, 15, 0));
                time.reset();
            }
        }else{
            allPoints.add(new CurvePoint(-5, 105, 1, 1, 15, 0));
            allPoints.add(new CurvePoint(-10, 70, 1, 1, 15, 0));
        }


        RobotMovement.followCurve(allPoints, robot, telemetry);
    }
}
