package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Odometry.DashboardUtil;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;
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
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public TelemetryPacket packet;
    Canvas fieldOverlay;

    enum Drive_State{
        Driving,
        AutoAllign,
        PowerShots
    }

    Drive_State mDriveState = Drive_State.Driving;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        packet = new TelemetryPacket();
        fieldOverlay = packet.fieldOverlay();

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
        int power_shots = 0;

        timer.startTime();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        timer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            packet = new TelemetryPacket();
            fieldOverlay = packet.fieldOverlay();

            robot.updateBulkData();

            double distFromGoal = robot.getPos().vec().distTo(robot.ULTIMATE_GOAL_POS);

            //double angle = Math.atan2(robot.ULTIMATE_GOAL_POS.getX() - currentPoseSnapShot.getX(), robot.ULTIMATE_GOAL_POS.getY() - currentPoseSnapShot.getY());

            switch (mDriveState){
                case Driving:
                    if(gamepad1ex.isPress(GamepadEx.Control.left_stick_button)){
                        xToggle = !xToggle;
                    }

                    robot.drive.driveCentric(gamepad1, xToggle ? 0.5 : 1.0, xToggle ? 0.3 : 1.0, robot.getPos().getHeading() + Math.toRadians(90));

                    if(gamepad1ex.isPress(GamepadEx.Control.right_stick_button)){
                        timer.reset();
                        mDriveState = Drive_State.AutoAllign;
                        currentPoseSnapShot = robot.getPos();
                    }

                    if(gamepad1ex.isPress(GamepadEx.Control.dpad_right)){
                        timer.reset();
                        mDriveState = Drive_State.PowerShots;
                    }

                    if(gamepad1ex.isPress(GamepadEx.Control.a)){
                        robot.setAngle(Math.PI);
                    }

                    break;
                case AutoAllign:
                    if(Math.abs(gamepad1ex.gamepad.left_stick_x) >= 0.3 || Math.abs(gamepad1ex.gamepad.right_stick_x) >= 0.3 || Math.abs(gamepad1ex.gamepad.left_stick_y) >= 0.3 || Math.abs(gamepad1ex.gamepad.right_stick_y) >= 0.3){
                        mDriveState = Drive_State.Driving;
                    }

                    if(gamepad1ex.isPress(GamepadEx.Control.dpad_right)){
                        timer.reset();
                        mDriveState = Drive_State.PowerShots;
                    }

                    //Angle Based Auto Align Code
                    /*double angle = Math.atan2(robot.ULTIMATE_GOAL_POS.getX() - currentPoseSnapShot.getX(), robot.ULTIMATE_GOAL_POS.getY() - currentPoseSnapShot.getY());
                    angle += Math.toRadians(180);

                    if(Math.abs(robot.getPos().getHeading() - angle) <= Math.toRadians(0.5)){
                        robot.shooter.stopper.setPosition(robot.shooter.stopPosUp);
                        //robot.shooter.mRobotState = Shooter.ShootState.PREPARE;
                        mDriveState = Drive_State.Driving;
                    }else{
                        robot.shooter.mStateTime.reset();
                    }

                    if(timer.time() >= 0.25){
                        robot.shooter.PROTO_AlignSlides = true;
                        robot.shooter.powerShotAngle = false;
                        robot.shooter.reset = false;
                    }
                    robot.shooter.shooter.setPower(1.0);
                    robot.shooter.flicker.setPosition(robot.shooter.flickPosDown);

                    robot.GoTo(new Pose2d(robot.getPos().getX(), robot.getPos().getY(), angle), new Pose2d(1.0, 1.0, 1.0));*/

                    //Position Based Auto Align Code
                    Pose2d testPos = new Pose2d(robot.getPos().getX(), robot.getPos().getY(), Math.toRadians(180));
                    //Pose2d testPos = new Pose2d(-10.636, 47.790, Math.PI);
                    /*if(testPos.vec().distTo(robot.getPos().vec()) <= 1.0 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(0.5)){
                        //robot.shooter.stopper.setPosition(robot.shooter.stopPosUp);

                        //robot.shooter.mRobotState = Shooter.ShootState.PREPARE;
                    }else{
                        robot.shooter.mStateTime.reset();
                    }*/

                    /*if(timer.time() >= 0.15){
                        if(gamepad2ex.isPress(GamepadEx.Control.dpad_down)){
                            robot.shooter.PROTO_AlignSlides = false;
                            robot.shooter.powerShotAngle = false;
                            robot.shooter.reset = true;
                            mDriveState = Drive_State.Driving;
                        }else{
                            robot.shooter.PROTO_AlignSlides = true;
                            robot.shooter.powerShotAngle = false;
                            robot.shooter.reset = false;
                        }
                    }*/

                    //robot.shooter.shooter.setPower(1.0);
                    //robot.shooter.flicker.setPosition(robot.shooter.flickPosDown);

                    robot.GoTo(testPos, new Pose2d(1.0, 1.0, 1.0));

                    break;
                case PowerShots:
                    robot.shooter.stopper.setPosition(robot.shooter.stopPosUp);
                    robot.shooter.setShooterAngle(Math.toRadians(25.0), robot.shooter.getShooterAngle(), 1.0);

                    if(Math.abs(gamepad1ex.gamepad.left_stick_x) >= 0.3 || Math.abs(gamepad1ex.gamepad.right_stick_x) >= 0.3){
                        mDriveState = Drive_State.Driving;
                    }

                    Pose2d POWER_SHOTS_1 = new Pose2d(Positions.POWER_SHOTS_1.x, Positions.POWER_SHOTS_1.y, Positions.POWER_SHOTS_HEADING);
                    Pose2d POWER_SHOTS_2 = new Pose2d(Positions.POWER_SHOTS_2.x, Positions.POWER_SHOTS_2.y, Positions.POWER_SHOTS_HEADING);
                    Pose2d POWER_SHOTS_3 = new Pose2d(Positions.POWER_SHOTS_3.x, Positions.POWER_SHOTS_3.y, Positions.POWER_SHOTS_HEADING);

                    double velo = robot.shooter.shooter.motor.getVelocity(AngleUnit.RADIANS);

                    switch (power_shots){
                        case 0:
                            robot.GoTo(POWER_SHOTS_1, new Pose2d(1.0, 1.0, 1.0));
                            if(robot.getPos().vec().distTo(POWER_SHOTS_1.vec()) <= 0.8 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(0.5) && velo >= 5.3){
                                if(timer.time() >= 2.0){
                                    power_shots = 1;
                                }else if(timer.time() >= 1.0){
                                    robot.shooter.powerShot(0);
                                }
                            }else{
                                timer.reset();
                            }
                            break;
                        case 1:
                            robot.GoTo(POWER_SHOTS_2, new Pose2d(1.0, 1.0, 1.0));
                            if(robot.getPos().vec().distTo(POWER_SHOTS_2.vec()) <= 0.8 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(0.5) && velo >= 5.3){
                                if(timer.time() >= 1.0){
                                    power_shots = 2;
                                }
                                robot.shooter.powerShot(1);
                            }else{
                                timer.reset();
                            }
                            break;
                        case 2:
                            robot.GoTo(POWER_SHOTS_3, new Pose2d(1.0, 1.0, 1.0));
                            if(robot.getPos().vec().distTo(POWER_SHOTS_3.vec()) <= 0.8 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(0.5) && velo >= 5.3){
                                if(timer.time() >= 1.0){
                                    mDriveState = Drive_State.Driving;
                                    robot.shooter.pushSlide.setPosition(robot.shooter.pushIdle);
                                    robot.shooter.shooter.setPower(0.2);
                                }
                                robot.shooter.powerShot(2);
                            }else{
                                timer.reset();
                            }
                            break;
                    }
                    break;
            }

            telemetry.addData("Is Rest? ", gamepad1ex.gamepad.atRest());

            robot.updatePos();
            robot.drive.write();

            robot.shooter.operate(gamepad1ex, gamepad2ex, distFromGoal, packet);
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

            packet.put("Pose", robot.getPos());
            packet.put("Vertical Heading: ", Math.toDegrees(-(robot.getRawLeft_Y_Dist() - robot.getRawRight_Y_Dist())/ S4T_Localizer.TRACK_WIDTH1) % (360));
            packet.put("Strafe Heading: ", Math.toDegrees(-(robot.getRawLeft_X_Dist() - robot.getRawRight_X_Dist())/S4T_Localizer.TRACK_WIDTH2) % (360));
            packet.put("wf", robot.localizer.wf);
            packet.put("ws", robot.localizer.ws);
            packet.put("LX", robot.getRawLeft_X_Dist());
            packet.put("RX", robot.getRawRight_X_Dist());
            packet.put("LY", robot.getRawLeft_Y_Dist());
            packet.put("RY", robot.getRawRight_Y_Dist());

            packet.put("gamepad x", gamepad1ex.gamepad.left_stick_x);

            DashboardUtil.drawRobot(fieldOverlay, robot.localizer.dashboardPos);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
