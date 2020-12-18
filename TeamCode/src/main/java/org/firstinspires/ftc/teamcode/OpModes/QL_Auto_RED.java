package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Vision.CameraTester;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "QL_Auto_RED")
//@Disabled
public class QL_Auto_RED extends OpMode {
    Robot robot;
    int run;

    ElapsedTime time = new ElapsedTime();

    enum State{
        GRAB_WOBBLE,
        DRIVE_TO_ZONE,
        DRIVE_TO_WOBBLE2,
        DRIVE_TO_ZONE2,
        PARK
    }

    State RobotState = State.GRAB_WOBBLE;

    @Override
    public void init() {
        robot = Robot.getInstance(hardwareMap, telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
    }

    @Override
    public void loop() {
        robot.updateBulkData();
        telemetry.addData("Pose: ", robot.getPos().toString());

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        switch (RobotState){
            case GRAB_WOBBLE:
                //robot.wobbleGoal.clamp();
                if(time.time() >= 1.5){
                    //robot.shooter.lift_auto();
                }
                break;
            case DRIVE_TO_ZONE:
                if(robot.getPos().vec().distTo(new Vector2d(35, -55)) <= 2.0){
                    //telemetry.addData("Time", String.valueOf(time.timeSeconds()));
                    if(time.time() >= 1){
                        //run = 1;
                        RobotMovement.resetIndex();
                    }else{
                        //robot.shooter.drop();
                        if(time.time() >= 1.5){
                            //robot.wobbleGoal.release();
                        }
                    }
                }else{
                    allPoints.add(new CurvePoint(0, 0, 1, 1, 15, 0));
                    allPoints.add(new CurvePoint(8, 24, 1, 1, 15, 0));
                    allPoints.add(new CurvePoint(13, 61, 1, 1, 15, 0));

                    time.reset();
                }
                break;
            case DRIVE_TO_WOBBLE2:
                if(robot.getPos().vec().distTo(new Vector2d(29, -2)) <= 2.0){
                    telemetry.addData("Time", String.valueOf(time.time()));
                    if(time.time() >= 1){
                        run = 2;
                        RobotMovement.resetIndex();
                    }else{
                        //robot.wobbleGoal.clamp();
                        if(time.time() >= 1.5){
                            //robot.shooter.lift_auto();
                        }
                    }
                }else{
                    allPoints.add(new CurvePoint(35, -55, 1, 1, 15, 0));
                    allPoints.add(new CurvePoint(48, -22, 1, 1, 15, 0));
                    allPoints.add(new CurvePoint(29, -2, 1, 1, 15, 0));
                    time.reset();
                }
                break;
            case DRIVE_TO_ZONE2:
                if(robot.getPos().vec().distTo(new Vector2d(5, -105)) <= 2.0){
                    //telemetry.addData("Time", String.valueOf(time.timeSeconds()));
                    if(time.time() >= 1){
                        run = 3;
                        RobotMovement.resetIndex();
                    }else{
                        //robot.shooter.drop();
                        if(time.time() >= 1.5){
                            //robot.wobbleGoal.release();
                        }
                    }
                }else{
                    allPoints.add(new CurvePoint(29, -2, 1, 1, 15, 0));
                    allPoints.add(new CurvePoint(48, -22, 1, 1, 15, 0));
                    allPoints.add(new CurvePoint(5, -105, 1, 1, 15, 0));
                    time.reset();
                }
                break;
            case PARK:
                allPoints.add(new CurvePoint(5, -105, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(10, -70, 1, 1, 15, 0));
                break;
        }

        if(run == 0){
            //telemetry.addData("dist", String.valueOf(Robot.getPose().distanceToVector(new Vector3(50, 55))));
            if(robot.getPos().vec().distTo(new Vector2d(35, -55)) <= 2.0){
                //telemetry.addData("Time", String.valueOf(time.timeSeconds()));
                if(time.time() >= 1){
                    run = 1;
                    RobotMovement.resetIndex();
                }else{
                    //robot.shooter.drop();
                    if(time.time() >= 1.5){
                        //robot.wobbleGoal.release();
                    }
                }
            }else{
                allPoints.add(new CurvePoint(0, 0, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(5, -105, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(35, -55, 1, 1, 15, 0));

                time.reset();
            }
        }else if(run == 1){
            if(robot.getPos().vec().distTo(new Vector2d(29, -2)) <= 2.0){
                telemetry.addData("Time", String.valueOf(time.time()));
                if(time.time() >= 1){
                    run = 2;
                    RobotMovement.resetIndex();
                }else{
                    //robot.wobbleGoal.clamp();
                    if(time.time() >= 1.5){
                        //robot.shooter.lift_auto();
                    }
                }
            }else{
                allPoints.add(new CurvePoint(35, -55, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(48, -22, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(29, -2, 1, 1, 15, 0));
                time.reset();
            }
        }else if(run == 2){
            if(robot.getPos().vec().distTo(new Vector2d(5, -105)) <= 2.0){
                //telemetry.addData("Time", String.valueOf(time.timeSeconds()));
                if(time.time() >= 1){
                    run = 3;
                    RobotMovement.resetIndex();
                }else{
                    //robot.shooter.drop();
                    if(time.time() >= 1.5){
                        //robot.wobbleGoal.release();
                    }
                }
            }else{
                allPoints.add(new CurvePoint(29, -2, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(48, -22, 1, 1, 15, 0));
                allPoints.add(new CurvePoint(5, -105, 1, 1, 15, 0));
                time.reset();
            }
        }else{
            allPoints.add(new CurvePoint(5, -105, 1, 1, 15, 0));
            allPoints.add(new CurvePoint(10, -70, 1, 1, 15, 0));
        }


        RobotMovement.followCurve(allPoints, robot, telemetry);
    }
}
