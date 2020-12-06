package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

@TeleOp
public class LinearTeleOp extends LinearOpMode {
    //Gamepad1:
    //Left Stick Button = align to goal
    //a/y = intake in/out

    //Gamepad2:
    //Dpad Up/Down = Wobble Goal Slides
    //b(toggle) = Wobble Goal Grabber

    Robot robot = null;
    Pose2d storedPos = new Pose2d(0, 0, 0);
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private Pose2d currentPoseSnapShot = new Pose2d(0, 0, 0);
    final double targetPosX = -12;
    final double targetPosY = 130;
    //private final double flickPosDown = 0.1139999;
    //private final double stopPosUp = 0.6;

    enum Drive_State{
        Driving,
        AutoAllign
    }

    Drive_State mDriveState = Drive_State.Driving;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot = new Robot(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.updateBulkData();

            //double angle = Math.atan2(robot.ULTIMATE_GOAL_POS.getX() - currentPoseSnapShot.getX(), robot.ULTIMATE_GOAL_POS.getY() - currentPoseSnapShot.getY());

            switch (mDriveState){
                case Driving:
                    robot.drive.driveCentric(gamepad1, robot.shooter.aToggle ? 0.5 : 1.0, robot.shooter.aToggle ? 0.3 : 1.0, robot.getPos().getHeading() + Math.toRadians(90));
                    robot.shooter.setShooterPower(0.2);
                    //robot.drive.driveWithHeading(gamepad1.left_stick_x, gamepad1.left_stick_y, (angle + Math.toRadians(180)) % Math.toRadians(360), robot.getPos().getHeading(), robot.getPos().getHeading() + Math.toRadians(90), 1.0);

                    if(gamepad1.right_stick_button){
                        mDriveState = Drive_State.AutoAllign;
                        currentPoseSnapShot = robot.getPos();
                    }

                    break;
                case AutoAllign:
                    if(!gamepad1.atRest()){
                        mDriveState = Drive_State.Driving;
                    }

                    double x = robot.getPos().getX();
                    double y = robot.getPos().getY();
                    double angle = Math.atan2(targetPosX - robot.getPos().getX(), targetPosY - robot.getPos().getY());
                    angle += Math.toRadians(180);

                    if(robot.getPos().getHeading() <= (angle - Math.toRadians(1)) || robot.getPos().getHeading() >= (angle + Math.toRadians(1))) {
                        robot.GoTo(x, y, angle, 1.0, 1.0, 1.0);
                        robot.shooter.setShooterPower(1.0);
                        robot.shooter.flicker.setPosition(robot.shooter.flickPosDown);
                        robot.shooter.stopper.setPosition(robot.shooter.stopPosUp);
                    } else {
                        robot.shooter.mStateTime.reset();
                        robot.shooter.mRobotState = Shooter.ShootState.PREPARE;
                        robot.drive.setPower(0,0,0);
                        mDriveState = Drive_State.Driving;
                    }
                    break;
/*
                    double angle = Math.atan2(robot.ULTIMATE_GOAL_POS.getX() - currentPoseSnapShot.getX(), robot.ULTIMATE_GOAL_POS.getY() - currentPoseSnapShot.getY());

                    if(Math.abs(robot.getPos().getHeading() - (angle + Math.toRadians(180)) % Math.toRadians(360)) >= Math.toRadians(1.0)){
                        robot.GoTo(new Pose2d(currentPoseSnapShot.getX(), currentPoseSnapShot.getY(), (angle + Math.toRadians(180)) % Math.toRadians(360)), new Pose2d(1.0, 1.0, 1.0));
                    }else{
                        robot.drive.setPower(0, 0, 0);
                        mDriveState = Drive_State.Driving;
                    }

 */
            }

            previousDpadUp = gamepad1.dpad_up;
            previousDpadDown = gamepad1.dpad_down;

            robot.updatePos();
            robot.drive.write();

            robot.shooter.operate(gamepad1, gamepad2, robot.getPos().vec().distTo(robot.ULTIMATE_GOAL_POS));
            robot.shooter.write();

            telemetry.addData("Shooter Angle", Math.toDegrees(robot.getShooterAngle()));

            //robot.wobbleGoal.operate(gamepad1);
            //robot.wobbleGoal.write();

            robot.intake.operate(gamepad1, gamepad2);
            robot.intake.write();

            telemetry.addData("stored pos:", storedPos);
            telemetry.addData("State: ", mDriveState);
            telemetry.addData("Pos: ", robot.getPos());
            telemetry.addData("------------", "------------");
            telemetry.addData("Target Angle", Math.atan2(targetPosX - robot.getPos().getX(), targetPosY - robot.getPos().getY()));
            telemetry.addData("Angle ", robot.getPos().getHeading());
            telemetry.addData("State", mDriveState);
            telemetry.addData("------------", "-----------");
            telemetry.addData("Right X: ", robot.getRight_X_Dist());
            telemetry.addData("Left X: ", robot.getLeft_X_Dist());
            telemetry.addData("Right Y: ", robot.getRight_Y_Dist());
            telemetry.addData("Left Y: ", robot.getLeft_Y_Dist());
            telemetry.update();
        }
    }
}
