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
    private boolean first = false;

    @Override
    public void runOpMode(){
        packet = new TelemetryPacket();
        fieldOverlay = packet.fieldOverlay();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot = new Robot(hardwareMap, telemetry);

        robot.localizer.gyro.update(robot.getData());
        robot.localizer.gyro.reset();

        robot.shooter.init();
        robot.wobbleGoal.init();

        robot.intake.dropIntake();
        robot.intake.write();

        gamepad1ex = new GamepadEx(gamepad1);
        gamepad2ex = new GamepadEx(gamepad2);
        int power_shots = 0;

        timer.startTime();

        robot.setStartPose(new Pose2d(9.368, -5.198, 0));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        timer.reset();
        robot.localizer.startTime(robot.getData());
        robot.intake.startTimer();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            packet = new TelemetryPacket();
            fieldOverlay = packet.fieldOverlay();

            robot.updateBulkData();

            double distFromGoal = robot.getPos().vec().distTo(robot.ULTIMATE_GOAL_POS);
            telemetry.addData("Start pos", robot.getStartPos());

            telemetry.addData("Dist to Ultimate Goal: ", distFromGoal - 7.25);
            //double angle = Math.atan2(robot.ULTIMATE_GOAL_POS.getX() - currentPoseSnapShot.getX(), robot.ULTIMATE_GOAL_POS.getY() - currentPoseSnapShot.getY());

            switch (mDriveState){
                case Driving:

                    robot.drive.driveCentric(gamepad1, 1.0,  1.0, robot.getPos().getHeading() + Math.toRadians(90));

                    if(gamepad1ex.isPress(GamepadEx.Control.right_trigger)){
                        timer.reset();
                        mDriveState = Drive_State.AutoAllign;
                        currentPoseSnapShot = robot.getPos();
                    }

                    if(gamepad1ex.isPress(GamepadEx.Control.dpad_up) || gamepad2ex.isPress(GamepadEx.Control.dpad_left)){
                        robot.shooter.positionAutoAlign = !robot.shooter.positionAutoAlign;
                    }

                    if(gamepad1ex.isPress(GamepadEx.Control.dpad_left)){
                        timer.reset();
                        power_shots = 0;
                        robot.setStartPose(new Pose2d(0, 0, 0));
                        robot.localizer.reset(new Pose2d(-9.235, 58.835, Math.PI));
                        mDriveState = Drive_State.PowerShots;
                    }

                    if(gamepad1ex.isPress(GamepadEx.Control.x)){
                        robot.setAngle(Math.PI);
                    }

                    if(gamepad1ex.gamepad.share){
                        robot.localizer.reset();
                    }

                    if(gamepad1ex.isPress(GamepadEx.Control.right_bumper)){
                        xToggle = false;
                    }

                    if(gamepad1ex.isPress(GamepadEx.Control.left_trigger)){
                        xToggle = false;
                    }

                    break;
                case AutoAllign:
                    if(gamepad1ex.isPress(GamepadEx.Control.dpad_up) || gamepad2ex.isPress(GamepadEx.Control.dpad_left)){
                        robot.shooter.positionAutoAlign = !robot.shooter.positionAutoAlign;
                    }

                    if(Math.abs(gamepad1ex.gamepad.left_stick_x) >= 0.15 || Math.abs(gamepad1ex.gamepad.right_stick_x) >= 0.15 || Math.abs(gamepad1ex.gamepad.left_stick_y) >= 0.15 || Math.abs(gamepad1ex.gamepad.right_stick_y) >= 0.15){
                        //xToggle = true;
                        mDriveState = Drive_State.Driving;
                    }

                    if(gamepad1ex.isPress(GamepadEx.Control.dpad_left)){
                        timer.reset();
                        power_shots = 0;
                        robot.setStartPose(new Pose2d(0, 0, 0));
                        robot.localizer.reset(new Pose2d(-9.235, 58.835, Math.PI));
                        mDriveState = Drive_State.PowerShots;
                    }

                    if(!robot.shooter.positionAutoAlign){
                        double angle = Math.atan2(robot.ULTIMATE_GOAL_POS.getX() - robot.getPos().getX(), robot.ULTIMATE_GOAL_POS.getY() - robot.getPos().getY());
                        angle += Math.toRadians(180);
                        double myDistFromGoal = distFromGoal - 7.25;
                        if(myDistFromGoal > 65){
                            robot.shooter.kickOutEnabled = true;
                            robot.GoTo(new Pose2d(robot.getPos().getX(), robot.getPos().getY(), angle), new Pose2d(1.0, 1.0, 1.0));
                        }else{
                            robot.shooter.kickOutEnabled = false;
                            robot.GoTo(new Pose2d(-22, 50.5, Math.toRadians(188.123)), new Pose2d(1.0, 1.0, 1.0));
                        }
                    }else{
                        robot.GoTo(new Pose2d(-22, 50.5, Math.toRadians(188.123)), new Pose2d(1.0, 1.0, 1.0));
                    }

                    break;
                case PowerShots:
                    robot.shooter.stopper.setPosition(robot.shooter.stopPosUp);
                    robot.shooter.flicker.setPosition(robot.shooter.flickPosDown);
                    robot.shooter.shooter.setPower(1.0);
                    robot.shooter.PROTO_AlignSlides = true;
                    robot.shooter.powerShotAngle = true;

                    if(Math.abs(gamepad1ex.gamepad.left_stick_x) >= 0.3 || Math.abs(gamepad1ex.gamepad.right_stick_x) >= 0.3){
                        mDriveState = Drive_State.Driving;
                    }

                    Pose2d POWER_SHOTS_1 = new Pose2d(Positions.POWER_SHOTS_1.x, Positions.POWER_SHOTS_1.y, Positions.POWER_SHOTS_HEADING);
                    Pose2d POWER_SHOTS_2 = new Pose2d(Positions.POWER_SHOTS_2.x, Positions.POWER_SHOTS_2.y, Positions.POWER_SHOTS_HEADING);
                    Pose2d POWER_SHOTS_3 = new Pose2d(Positions.POWER_SHOTS_3.x, Positions.POWER_SHOTS_3.y, Positions.POWER_SHOTS_HEADING);

                    double velo = robot.shooter.getShooterVelocity();

                    switch (power_shots){
                        case 0:
                            robot.GoTo(POWER_SHOTS_1, new Pose2d(1.0, 1.0, 1.0));
                            if(robot.getPos().vec().distTo(POWER_SHOTS_1.vec()) <= 1.0 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(0.8) && velo > 2200){
                                if(timer.time() >= 0.4){
                                    timer.reset();
                                    power_shots = 1;
                                }else if(timer.time() >= 0.2){
                                    robot.shooter.powerShot(0);
                                }
                            }else{
                                timer.reset();
                            }
                            break;
                        case 1:
                            robot.GoTo(POWER_SHOTS_2, new Pose2d(1.0, 1.0, 1.0));
                            if(robot.getPos().vec().distTo(POWER_SHOTS_2.vec()) <= 1.0 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(0.8) && velo > 2200){
                                if(timer.time() >= 0.2){
                                    timer.reset();
                                    power_shots = 2;
                                }
                                robot.shooter.powerShot(1);
                            }else{
                                timer.reset();
                            }
                            break;
                        case 2:
                            robot.GoTo(POWER_SHOTS_3, new Pose2d(1.0, 1.0, 1.0));
                            if(robot.getPos().vec().distTo(POWER_SHOTS_3.vec()) <= 1.0 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(0.8) && velo > 2200){
                                if(timer.time() >= 0.2){
                                    timer.reset();
                                    power_shots = 0;
                                    robot.shooter.pushSlide.setPosition(robot.shooter.pushIdle);
                                    robot.shooter.shooter.setPower(0.2);
                                    mDriveState = Drive_State.Driving;
                                }else{
                                    robot.shooter.powerShot(2);
                                }
                            }else{
                                timer.reset();
                            }
                            break;
                    }
                    break;
            }

            robot.updatePos();

            robot.drive.write();

            robot.shooter.operate(gamepad1ex, gamepad2ex, distFromGoal, packet);
            robot.shooter.write();

            robot.wobbleGoal.operate(gamepad1ex);
            robot.wobbleGoal.write();

            robot.intake.operate(gamepad1ex, gamepad2ex, telemetry);
            robot.intake.write();

            telemetry.addData("AUTO-ALIGN MODE", robot.shooter.positionAutoAlign ? "POSITION BASED" : "FULL FIELD");
            telemetry.addData("State: ", mDriveState);
            telemetry.addData("Pos: ", robot.getPos());
            telemetry.update();

            gamepad1ex.loop();
            gamepad2ex.loop();

            packet.put("Pose", robot.getPos());
            //packet.put("Kalman Filtered Pos", robot.localizer.getKalmanFilteredPos());
            packet.put("Vertical Heading: ", Math.toDegrees(-(robot.getRawLeft_Y_Dist() - robot.getRawRight_Y_Dist())/ S4T_Localizer.TRACK_WIDTH1) % (360));
            packet.put("Strafe Heading: ", Math.toDegrees(-(robot.getRawLeft_X_Dist() - robot.getRawRight_X_Dist())/S4T_Localizer.TRACK_WIDTH2) % (360));

            DashboardUtil.drawRobot(fieldOverlay, robot.localizer.dashboardPos);
            dashboard.sendTelemetryPacket(packet);
        }


        //packet.put("stopped", "stopped");
        //dashboard.sendTelemetryPacket(packet);
        //robot.localizer.stopCamera();
    }
}
