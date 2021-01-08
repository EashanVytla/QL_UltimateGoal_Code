package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Robot robot = null;
    Pose2d storedPos = new Pose2d(0, 0, 0);
    private Pose2d currentPoseSnapShot = new Pose2d(0, 0, 0);
    private boolean xToggle = false;
    //private boolean prevx = false;
    GamepadEx gamepad1ex;
    GamepadEx gamepad2ex;

    enum Drive_State{
        Driving,
        AutoAllign
    }

    Drive_State mDriveState = Drive_State.Driving;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot = new Robot(hardwareMap, telemetry);

        robot.shooter.init();
        robot.wobbleGoal.init();

        gamepad1ex = new GamepadEx(gamepad1);
        gamepad2ex = new GamepadEx(gamepad2);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        timer.startTime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.updateBulkData();

            double distFromGoal = robot.getPos().vec().distTo(robot.ULTIMATE_GOAL_POS);

            //double angle = Math.atan2(robot.ULTIMATE_GOAL_POS.getX() - currentPoseSnapShot.getX(), robot.ULTIMATE_GOAL_POS.getY() - currentPoseSnapShot.getY());

            switch (mDriveState){
                case Driving:
                    if(gamepad1ex.isPress(GamepadEx.Control.x)){
                        xToggle = !xToggle;
                    }

                    robot.drive.driveCentric(gamepad1, xToggle ? 0.5 : 1.0, xToggle ? 0.3 : 1.0, robot.getPos().getHeading() + Math.toRadians(90));

                    if(gamepad1ex.isPress(GamepadEx.Control.right_stick_button)){
                        timer.reset();
                        mDriveState = Drive_State.AutoAllign;
                        currentPoseSnapShot = robot.getPos();
                    }

                    break;
                case AutoAllign:
                    if(Math.abs(gamepad1ex.gamepad.left_stick_x) >= 0.3 || Math.abs(gamepad1ex.gamepad.right_stick_x) >= 0.3){
                        robot.shooter.reset = true;
                        mDriveState = Drive_State.Driving;
                    }

                    //Angle Based Auto Align Code
                    double angle = Math.atan2(robot.ULTIMATE_GOAL_POS.getX() - currentPoseSnapShot.getX(), robot.ULTIMATE_GOAL_POS.getY() - currentPoseSnapShot.getY());
                    angle += Math.toRadians(180);

                    if(Math.abs(robot.getPos().getHeading() - angle) <= Math.toRadians(0.5)){
                        robot.shooter.stopper.setPosition(robot.shooter.stopPosUp);
                        robot.shooter.mStateTime.reset();
                        robot.shooter.mRobotState = Shooter.ShootState.PREPARE;
                        mDriveState = Drive_State.Driving;
                        robot.shooter.PROTO_AlignSlides = false;
                    }else if(timer.time() >= 0.25){
                        robot.shooter.PROTO_AlignSlides = true;
                        robot.shooter.powerShotAngle = false;
                        robot.shooter.reset = false;
                    }
                    robot.shooter.shooter.setPower(1.0);
                    robot.shooter.flicker.setPosition(robot.shooter.flickPosDown);

                    robot.GoTo(new Pose2d(robot.getPos().getX(), robot.getPos().getY(), angle), new Pose2d(1.0, 1.0, 1.0));

                    //Position Based Auto Align Code
                    //Pose2d testPos = new Pose2d(-13, 48, Math.toRadians(180));
                    //Pose2d testPos = new Pose2d(robot.getPos().getX(), robot.getPos().getY(), Math.PI);
                    /*if(testPos.vec().distTo(robot.getPos().vec()) <= 0.5){
                        robot.shooter.mRobotState = Shooter.ShootState.PREPARE;
                    }*/
                    //robot.GoTo(testPos, new Pose2d(1.0, 1.0, 1.0));

                    break;
            }

            telemetry.addData("Is Rest? ", gamepad1ex.gamepad.atRest());

            robot.updatePos();
            robot.drive.write();

            robot.shooter.operate(gamepad1ex, gamepad2ex, distFromGoal);
            robot.shooter.write();

            robot.wobbleGoal.operate(gamepad2ex);
            robot.wobbleGoal.write();

            robot.intake.operate(gamepad1ex, gamepad2ex, telemetry);
            robot.intake.write();

            telemetry.addData("State: ", mDriveState);
            telemetry.addData("Pos: ", robot.getPos());
            telemetry.addData("Right X: ", robot.getRight_X_Dist());
            telemetry.addData("Left X: ", robot.getLeft_X_Dist());
            telemetry.addData("Right Y: ", robot.getRight_Y_Dist());
            telemetry.addData("Left Y: ", robot.getLeft_Y_Dist());
            telemetry.update();

            gamepad1ex.loop();
            gamepad2ex.loop();
        }
    }
}
