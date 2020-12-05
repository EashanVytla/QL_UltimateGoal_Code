package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    Robot robot;
    Pose2d storedPos = new Pose2d(0, 0, 0);
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private Pose2d currentPoseSnapShot = new Pose2d(0, 0, 0);

    enum Drive_State{
        Driving,
        AutoDrivePos,
        AutoAllign
    }

    Drive_State mDriveState = Drive_State.Driving;


    public void init(){
        robot = Robot.getInstance(hardwareMap, telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public void loop(){
        switch (mDriveState){
            case Driving:
                robot.drive.driveCentric(gamepad1, 1.0, 1.0, robot.getPos().getHeading() + Math.toRadians(90));

                if(gamepad1.right_stick_button){
                    mDriveState = Drive_State.AutoDrivePos;
                }else if(gamepad1.a){
                    storedPos = robot.getPos();
                }

                if(gamepad1.left_stick_button){
                    mDriveState = Drive_State.AutoAllign;
                    currentPoseSnapShot = robot.getPos();
                }

                break;
            case AutoDrivePos:
                if(!gamepad1.atRest()){
                    mDriveState = Drive_State.Driving;
                }

                if(robot.getPos().vec().distTo(storedPos.vec()) >= 0.5){
                    robot.GoTo(storedPos, new Pose2d(1.0, 1.0, 1.0));
                }else{
                    robot.drive.setPower(0, 0, 0);
                    mDriveState = Drive_State.Driving;
                }

                break;
            case AutoAllign:
                if(!gamepad1.atRest()){
                    mDriveState = Drive_State.Driving;
                }

                double angle = Math.atan2(robot.getPos().getX(), robot.getPos().getY());

                if(Math.abs(robot.getPos().getHeading() - angle) <= Math.toRadians(1.0)){
                    robot.GoTo(currentPoseSnapShot, new Pose2d(1.0, 1.0, 1.0));
                }else{
                    robot.drive.setPower(0, 0, 0);
                    mDriveState = Drive_State.Driving;
                }

                break;
        }

        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;

        robot.updatePos();
        robot.drive.write();

        robot.shooter.operate(gamepad1, gamepad2, robot.getPos().vec().distTo(robot.ULTIMATE_GOAL_POS));
        robot.shooter.write();

        //robot.wobbleGoal.operate(gamepad2);
        //robot.wobbleGoal.write();

        robot.intake.operate(gamepad1, gamepad2);
        robot.intake.write();

        telemetry.addData("MA3 Readings: ", robot.getShooterAngle());

        telemetry.addData("stored pos:", storedPos);
        telemetry.addData("State: ", mDriveState);
        telemetry.addData("Pos: ", robot.getPos());
        telemetry.addData("Right X: ", robot.getRight_X_Dist());
        telemetry.addData("Left X: ", robot.getLeft_X_Dist());
        telemetry.addData("Right Y: ", robot.getRight_Y_Dist());
        telemetry.addData("Left Y: ", robot.getLeft_Y_Dist());
    }
}
