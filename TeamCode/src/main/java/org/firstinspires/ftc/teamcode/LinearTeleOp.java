package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    enum Drive_State{
        Driving,
        AutoAllign
    }

    Drive_State mDriveState = Drive_State.Driving;

    @Override
    public void runOpMode() {
        telemetry.addData("Robot Status", Robot.robotS==null?"null":"filled");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot = Robot.getInstance(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            switch (mDriveState){
                case Driving:
                    robot.drive.driveCentric(gamepad1, robot.shooter.aToggle ? 0.5 : 1.0, robot.shooter.aToggle ? 0.3 : 1.0, robot.getPos().getHeading() + Math.toRadians(90));


                    if(gamepad1.left_stick_button){
                        mDriveState = Drive_State.AutoAllign;
                        currentPoseSnapShot = robot.getPos();
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

            if(gamepad2.dpad_up && !previousDpadUp){
                robot.shooter.lift_auto();
            }

            if(gamepad2.dpad_down && !previousDpadDown){
                robot.shooter.drop();
            }

            previousDpadUp = gamepad1.dpad_up;
            previousDpadDown = gamepad1.dpad_down;

            robot.updatePos();
            robot.drive.write();

            robot.shooter.operate(gamepad1, gamepad2);
            robot.shooter.write();

            //robot.wobbleGoal.operate(gamepad1);
            //robot.wobbleGoal.write();

            telemetry.addData("stored pos:", storedPos);
            telemetry.addData("State: ", mDriveState);
            telemetry.addData("Pos: ", robot.getPos());
            telemetry.addData("Right X: ", robot.getRight_X_Dist());
            telemetry.addData("Left X: ", robot.getLeft_X_Dist());
            telemetry.addData("Right Y: ", robot.getRight_Y_Dist());
            telemetry.addData("Left Y: ", robot.getLeft_Y_Dist());
            telemetry.update();

            if(isStopRequested()){
                robot.stop();
                Robot.robotS = null;
            }
        }

        robot.stop();
        Robot.robotS = null;
    }
}
