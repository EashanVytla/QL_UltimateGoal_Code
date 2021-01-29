package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;
import org.opencv.core.Point;

import java.util.ArrayList;

@Autonomous
public class QL_Auto_Linear extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = null;
        int stage = 0;
        int ring_case = 0;
        boolean first = true;
        boolean pausePP = false;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        boolean error = false;
        int power_shots = 0;

        robot = new Robot(hardwareMap, telemetry);

        robot.setStartPose(new Pose2d(5.104, -5.293, 0));

        robot.localizer.reset();

        robot.shooter.flicker.setPosition(robot.shooter.flickPosDown);
        robot.shooter.stopper.setPosition(robot.shooter.stopPosDown);
        robot.shooter.pushSlide.setPosition(robot.shooter.pushIdle);
        robot.shooter.shooter.setPower(0.2);
        robot.shooter.write();

        robot.wobbleGoal.servo_grab.setPosition(robot.wobbleGoal.clamp_pos);
        robot.wobbleGoal.servo_lift.setPosition(0.95);
        robot.wobbleGoal.servo_lift.write();
        robot.wobbleGoal.servo_grab.write();

        robot.intake.closeIntake();
        robot.intake.write();

        time.startTime();
        robot.initializeWebcam();

        while(!isStarted() && !isStopRequested()){
            FtcDashboard.getInstance().sendImage(robot.getWebcamImage());
            ring_case = robot.getRingStackCase();
            //ring_case = 4;
            telemetry.addData("Ring Case", ring_case);
            telemetry.update();
            time.reset();
        }

        robot.stopWebcam();

        double targetAngle = 0;
        double currentAngle = 0;

        int intakeCase = 0;

        boolean slidesGood = false;

        waitForStart();
        time.reset();

        while(opModeIsActive()){
            Pose2d CLEAR_STACK = new Pose2d(Positions.CLEAR_STACK.x, Positions.CLEAR_STACK.y, Positions.CLEAR_STACK_HEADING);
            Pose2d ZONE_1 = new Pose2d(Positions.ZONE_1.x, Positions.ZONE_1.y, Positions.ZONE_1_HEADING);
            Pose2d ZONE_2 = new Pose2d(Positions.ZONE_2.x, Positions.ZONE_2.y, Positions.ZONE_2_HEADING);
            Pose2d ZONE_3 = new Pose2d(Positions.ZONE_3.x, Positions.ZONE_3.y, Positions.ZONE_3_HEADING);
            Pose2d ZONE_1_b = new Pose2d(Positions.ZONE_1_b.x, Positions.ZONE_1_b.y, Positions.ZONE_1_HEADING);
            Pose2d ZONE_2_b = new Pose2d(Positions.ZONE_2_b.x, Positions.ZONE_2_b.y, Positions.ZONE_2_HEADING);
            Pose2d ZONE_3_b = new Pose2d(Positions.ZONE_3_b.x, Positions.ZONE_3_b.y, Positions.ZONE_3_HEADING);
            Pose2d POWER_SHOTS_1 = new Pose2d(Positions.POWER_SHOTS_1.x, Positions.POWER_SHOTS_1.y, Positions.POWER_SHOTS_HEADING);
            Pose2d POWER_SHOTS_2 = new Pose2d(Positions.POWER_SHOTS_2.x, Positions.POWER_SHOTS_2.y, Positions.POWER_SHOTS_HEADING);
            Pose2d POWER_SHOTS_3 = new Pose2d(Positions.POWER_SHOTS_3.x, Positions.POWER_SHOTS_3.y, Positions.POWER_SHOTS_HEADING);
            Pose2d WOBBLE_GOAL_2 = new Pose2d(Positions.WOBBLE_GOAL_2.x, Positions.CLEAR_STACK.y, Positions.WOBBLE_GOAL_2_HEADING);
            Pose2d INTAKE_STACK_1 = new Pose2d(Positions.INTAKE_STACK_1.x, Positions.INTAKE_STACK_1.y, Math.PI);
            Pose2d INTAKE_STACK_2 = new Pose2d(Positions.INTAKE_STACK_2.x, Positions.INTAKE_STACK_2.y, Math.PI);
            Pose2d PREPARE_INTAKE = new Pose2d(Positions.PREPARE_INTAKE.x, Positions.PREPARE_INTAKE.y, Math.PI);
            Pose2d CLEAR_STACK_2 = new Pose2d(Positions.CLEAR_STACK_2.x, Positions.CLEAR_STACK_2.y, Positions.CLEAR_STACK_2_HEADING);
            Pose2d PARK = new Pose2d(Positions.PARK.x, Positions.PARK.y, Positions.PARK_HEADING);

            robot.updateBulkData();
            ArrayList<CurvePoint> allPoints = new ArrayList<>();

            if(stage <= 4) {
                robot.shooter.setShooterAngle(Math.toRadians(25.56), robot.shooter.getShooterAngle(), 1.0);
            }

            currentAngle = robot.shooter.getShooterAngle();

            switch (stage){
                case 0:
                    robot.intake.dropIntake();
                    if(first){
                        time.reset();
                        first = false;
                    }

                    if(time.time() >= 0.5){
                        robot.wobbleGoal.autoLift();
                        time.reset();
                        RobotMovement.resetIndex();
                        stage = 1;
                    }

                    break;
                case 1:
                    allPoints.add(new CurvePoint(robot.getStartPos().getX(), robot.getStartPos().getY(), 1, 1, 15, 0));
                    allPoints.add(new CurvePoint(CLEAR_STACK, 1, 1, 15));
                    if(ring_case == 0){
                        allPoints.add(new CurvePoint(ZONE_1, 1.0, 1, 15));
                        if(robot.getPos().vec().distTo(ZONE_1.vec()) <= 2){
                            robot.drive.setPower(0, 0, 0);
                            robot.drive.write();

                            if(time.time() >= 0.25){
                                robot.wobbleGoal.lift();
                                robot.wobbleGoal.kick();
                                RobotMovement.resetIndex();
                                stage = 2;
                            }
                        }else{
                            time.reset();
                        }
                    }else if(ring_case == 1){
                        if(robot.getPos().vec().distTo(ZONE_2.vec()) <= 20){
                            allPoints.add(new CurvePoint(ZONE_2, 0.1, 1, 15));
                        }else{
                            allPoints.add(new CurvePoint(ZONE_2, 1.0, 1, 15));
                        }
                        if(robot.getPos().vec().distTo(ZONE_2.vec()) <= 2){
                            robot.drive.setPower(0, 0, 0);
                            robot.drive.write();

                            if(time.time() >= 0.25){
                                robot.wobbleGoal.lift();
                                robot.wobbleGoal.kick();
                                RobotMovement.resetIndex();
                                stage = 2;
                            }
                        }else{
                            time.reset();
                        }
                    }else{
                        if(robot.getPos().vec().distTo(ZONE_3.vec()) <= 35){
                            allPoints.add(new CurvePoint(ZONE_3, 0.1, 1, 15));
                        }else{
                            allPoints.add(new CurvePoint(ZONE_3, 1.0, 1, 15));
                        }


                        if(robot.getPos().vec().distTo(ZONE_3.vec()) <= 2){
                            robot.drive.setPower(0, 0, 0);
                            robot.drive.write();

                            if(time.time() >= 0.25){
                                robot.wobbleGoal.lift();
                                robot.wobbleGoal.kick();
                                RobotMovement.resetIndex();
                                stage = 2;
                            }
                        }else{
                            time.reset();
                        }
                    }
                    break;
                case 2:
                    robot.GoTo(POWER_SHOTS_1, new Pose2d(1, 1,1));
                    if(robot.getPos().vec().distTo(POWER_SHOTS_1.vec()) <= 0.8){
                        RobotMovement.resetIndex();
                        time.reset();
                        stage = 3;
                    }else{
                        if(robot.getPos().vec().distTo(POWER_SHOTS_1.vec()) <= 6){
                            robot.shooter.flicker.setPosition(robot.shooter.flickPosDown);
                            robot.shooter.stopper.setPosition(robot.shooter.stopPosUp);
                        }
                        robot.shooter.shooter.setPower(1.0);
                    }
                    break;
                case 3:
                    //6 inches

                    double velo = robot.shooter.shooter.motor.getVelocity(AngleUnit.RADIANS);

                    switch (power_shots){
                        case 0:
                            robot.GoTo(POWER_SHOTS_1, new Pose2d(1.0, 1.0, 1.0));
                            if(robot.getPos().vec().distTo(POWER_SHOTS_1.vec()) <= 1.0 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(1.0) && velo >= 4.5){
                                if(time.time() >= 0.5){
                                    power_shots = 1;
                                }else if(time.time() >= 0.25){
                                    robot.shooter.powerShot(0);
                                }
                            }else{
                                time.reset();
                            }
                            break;
                        case 1:
                            robot.GoTo(POWER_SHOTS_2, new Pose2d(1.0, 1.0, 1.0));
                            if(robot.getPos().vec().distTo(POWER_SHOTS_2.vec()) <= 1.0 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(1.0) && velo >= 4.5){
                                if(time.time() >= 0.25){
                                    power_shots = 2;
                                }
                                robot.shooter.powerShot(1);
                            }else{
                                time.reset();
                            }
                            break;
                        case 2:
                            robot.GoTo(POWER_SHOTS_3, new Pose2d(1.0, 1.0, 1.0));
                            if(robot.getPos().vec().distTo(POWER_SHOTS_3.vec()) <= 1.0 && Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(1.0) && velo >= 4.5){
                                if(time.time() >= 0.25){
                                    RobotMovement.resetIndex();
                                    stage = 4;
                                    robot.shooter.pushSlide.setPosition(robot.shooter.pushIdle);
                                    robot.shooter.shooter.setPower(0.2);
                                }
                                robot.shooter.powerShot(2);
                            }else{
                                time.reset();
                            }
                            break;
                    }

                    break;
                case 4:
                    if(currentAngle <= Math.toRadians(21)){
                        robot.shooter.slideSetPower(-0.01);
                    }else{
                        robot.shooter.slideSetPower(-0.5);
                        robot.shooter.stopper.setPosition(robot.shooter.stopPosDown);

                        robot.shooter.pushSlide.setPosition(robot.shooter.pushIdle);
                    }

                    if(robot.getPos().vec().distTo(WOBBLE_GOAL_2.vec()) <= 0.8 && Math.abs(robot.getPos().getHeading() - WOBBLE_GOAL_2.getHeading()) <= Math.toRadians(0.8)){
                        robot.drive.setPower(0, 0, 0);
                        robot.drive.write();
                        if(time.time() >= 0.5){
                            RobotMovement.resetIndex();
                            robot.wobbleGoal.autoLift();
                            stage = 5;
                        }else{
                            robot.wobbleGoal.clamp();
                        }
                    }else{
                        robot.shooter.pushSlide.setPosition(robot.shooter.pushIdle);
                        robot.shooter.shooter.setPower(0.2);
                        robot.GoTo(WOBBLE_GOAL_2, new Pose2d(0.2, 0.2, 1.0));
                        robot.wobbleGoal.down();
                        time.reset();
                    }
                    break;
                case 5:
                    robot.intake.setPower(-1.0);
                    if(currentAngle <= Math.toRadians(21)){
                        robot.shooter.slideSetPower(-0.01);
                    }else{
                        robot.shooter.slideSetPower(-0.5);
                        robot.shooter.stopper.setPosition(robot.shooter.stopPosDown);

                        robot.shooter.pushSlide.setPosition(robot.shooter.pushIdle);
                    }

                    allPoints.add(new CurvePoint(WOBBLE_GOAL_2, 1, 1, 15));
                    allPoints.add(new CurvePoint(CLEAR_STACK_2, 1, 1, 15));
                    if(ring_case == 0){
                        allPoints.add(new CurvePoint(ZONE_1_b, 1, 1, 15));

                        if(robot.getPos().vec().distTo(ZONE_1_b.vec()) <= 4){
                            robot.wobbleGoal.lift();
                            robot.wobbleGoal.kick();
                            RobotMovement.resetIndex();
                            stage = 6;
                        }
                    }else if(ring_case == 1){
                        allPoints.add(new CurvePoint(ZONE_2_b, 1, 1, 15));

                        if(robot.getPos().vec().distTo(ZONE_2_b.vec()) <= 4){
                            robot.wobbleGoal.lift();
                            robot.wobbleGoal.kick();
                            RobotMovement.resetIndex();
                            stage = 6;
                        }
                    }else{
                        allPoints.add(new CurvePoint(ZONE_3_b, 1, 1, 15));

                        if(robot.getPos().vec().distTo(ZONE_3_b.vec()) <= 4){
                            robot.wobbleGoal.lift();
                            robot.wobbleGoal.kick();
                            RobotMovement.resetIndex();
                            time.reset();
                            stage = 6;
                        }
                    }
                    break;
                case 6:
                    robot.shooter.shooter.setPower(1.0);
                    if(ring_case == 0){
                        stage = 7;
                    }

                    if(intakeCase <= 1){
                        if(currentAngle <= Math.toRadians(21)){
                            robot.shooter.slideSetPower(-0.01);
                        }else{
                            robot.shooter.slideSetPower(-0.5);
                            robot.shooter.stopper.setPosition(robot.shooter.stopPosDown);

                            robot.shooter.pushSlide.setPosition(robot.shooter.pushIdle);
                        }
                    }

                    if(intakeCase < 1){
                        robot.shooter.flicker.setPosition(robot.shooter.flickPosUp);
                    }

                    switch (intakeCase){
                        case 0:
                            robot.GoTo(PREPARE_INTAKE, new Pose2d(1, 1, 1));
                            if(robot.getPos().vec().distTo(PREPARE_INTAKE.vec()) <= 4){
                                time.reset();
                                if(ring_case == 0){
                                    stage = 7;
                                }else if(ring_case == 1){
                                    intakeCase = 3;
                                }else if(ring_case == 4){
                                    intakeCase++;
                                }
                            }
                            break;
                        case 1:
                            robot.intake.setPower(-1.0);
                            robot.GoTo(INTAKE_STACK_1, new Pose2d(1, 0.2, 1));
                            if(robot.getPos().vec().distTo(INTAKE_STACK_1.vec()) <= 1.0){
                                robot.shooter.slideSetPower(0.0);
                                robot.shooter.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                robot.shooter.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                if(time.time() >= 0.5){
                                    time.reset();
                                    intakeCase++;
                                }
                            }else{
                                robot.shooter.slideSetPower(-0.01);
                                time.reset();
                            }
                            break;
                        case 2:
                            targetAngle = Math.toRadians(robot.shooter.calculateShooterAngle(robot.getPos().vec().distTo(robot.ULTIMATE_GOAL_POS)));

                            robot.GoTo(INTAKE_STACK_1, new Pose2d(1, 1, 1));
                            robot.intake.setPower(0.0);

                            if(Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(0.8)){
                                if(time.time() >= 2.1){
                                    robot.shooter.resetAuto();
                                    if(time.time() >= 2.6){
                                        time.reset();
                                        intakeCase++;
                                    }
                                }else if(time.time() >= 1.1){
                                    robot.shooter.slideSetPower(0.095);
                                    robot.shooter.pushSlide.setPosition(robot.shooter.pushForward);
                                }else if(time.time() >= 0.85){
                                    robot.shooter.setShooterAngle(targetAngle, currentAngle, 1.0);
                                    robot.shooter.stopper.setPosition(robot.shooter.stopPosUp);
                                }else if(time.time() >= 0.15){
                                    robot.shooter.setShooterAngle(targetAngle, currentAngle, 1.0);
                                }else{
                                    robot.shooter.shooter.setPower(1);
                                    robot.shooter.flicker.setPosition(robot.shooter.flickPosDown);
                                }
                            }else{
                                time.reset();
                            }
                            break;
                        case 3:
                            robot.shooter.resetAuto();
                            robot.GoTo(INTAKE_STACK_2, new Pose2d(1, 0.2, 1));
                            if(robot.getPos().vec().distTo(INTAKE_STACK_2.vec()) <= 1.0){
                                robot.shooter.slideSetPower(0.0);
                                robot.shooter.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                robot.shooter.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                                if(time.time() >= 0.5){
                                    time.reset();
                                    intakeCase++;
                                }
                            }else{
                                robot.shooter.slideSetPower(-0.01);
                                robot.intake.setPower(-1.0);
                                time.reset();
                            }
                            break;
                        case 4:
                            targetAngle = Math.toRadians(robot.shooter.calculateShooterAngle(robot.getPos().vec().distTo(robot.ULTIMATE_GOAL_POS)));

                            robot.GoTo(INTAKE_STACK_2, new Pose2d(1, 1, 1));
                            robot.intake.setPower(0.0);

                            if(Math.abs(robot.getPos().getHeading() - Math.PI) <= Math.toRadians(0.8)){
                                if(time.time() >= 2.1){
                                    robot.shooter.resetAuto();
                                    if(time.time() >= 2.6){
                                        RobotMovement.resetIndex();
                                        stage = 7;
                                    }
                                }else if(time.time() >= 1.1){
                                    robot.shooter.slideSetPower(0.095);
                                    robot.shooter.pushSlide.setPosition(robot.shooter.pushForward);
                                }else if(time.time() >= 0.85){
                                    robot.shooter.setShooterAngle(targetAngle, currentAngle, 1.0);
                                    robot.shooter.stopper.setPosition(robot.shooter.stopPosUp);
                                }else if(time.time() >= 0.15){
                                    robot.shooter.setShooterAngle(targetAngle, currentAngle, 1.0);
                                }else{
                                    robot.shooter.shooter.setPower(1);
                                    robot.shooter.flicker.setPosition(robot.shooter.flickPosDown);
                                }
                            }else{
                                time.reset();
                            }

                            break;
                    }
                    break;
                case 7:
                    if(currentAngle <= Math.toRadians(21)){
                        robot.shooter.slideSetPower(-0.01);
                    }else{
                        robot.shooter.slideSetPower(-0.5);
                        robot.shooter.stopper.setPosition(robot.shooter.stopPosDown);

                        robot.shooter.pushSlide.setPosition(robot.shooter.pushIdle);
                    }

                    robot.intake.setPower(0);
                    robot.shooter.reset();
                    if(robot.getPos().vec().distTo(PARK.vec()) <= 10 && ring_case > 0){
                        robot.GoTo(PARK, new Pose2d(0.1, 0.1, 0.1));
                    }else{
                        robot.GoTo(PARK, new Pose2d(1.0, 1.0, 1.0));
                    }

                    break;
                default:
                    robot.drive.setPower(0, 0, 0);
                    error = true;
                    robot.drive.write();
                    break;
            }

            if(stage != 0 && stage != 2 && !error && stage != 3 && stage != 4 && stage != 6 && stage != 7 && !pausePP){
                RobotMovement.followCurve(allPoints, robot, telemetry);
            }

            telemetry.addData("target height: ", targetAngle);
            telemetry.addData("target height: ", currentAngle);

            robot.wobbleGoal.write();
            robot.shooter.write();
            robot.intake.write();

            dashboardTelemetry.addData("Pos", robot.getPos());
            dashboardTelemetry.addData("Error", robot.getPos().vec().distTo(robot.drive.target_pos.vec()));
            dashboardTelemetry.update();

            telemetry.addData("Pos", robot.getPos());
            telemetry.addData("Error", robot.getPos().vec().distTo(robot.drive.target_pos.vec()));
            telemetry.update();
        }
    }
}

@Config
class Positions {
    public static Point CLEAR_STACK = new Point(8, 24);
    public static Point ZONE_1 = new Point(9.464, 45.245);
    public static Point ZONE_2 = new Point(-11.923, 76.501);
    public static Point ZONE_3 = new Point(9.898, 96.822);
    public static Point ZONE_1_b = new Point(3.464, 52.245);
    public static Point ZONE_2_b = new Point(-20.923, 67.501);
    public static Point ZONE_3_b = new Point(3.898, 89.822);
    public static Point POWER_SHOTS_1 = new Point(-28.255, 46.955);
    public static Point POWER_SHOTS_2 = new Point(-35.255, 46.955);
    public static Point POWER_SHOTS_3 = new Point(-43.505, 46.955);
    public static Point WOBBLE_GOAL_2 = new Point(-28, 32);
    public static Point PREPARE_INTAKE = new Point(-11, 51);
    public static Point INTAKE_STACK_1 = new Point(-11, 39);
    public static Point INTAKE_STACK_2 = new Point(-11, 25);
    public static Point PARK = new Point(-12, 67);
    public static Point CLEAR_STACK_2 = new Point(-40, 49);

    public static double CLEAR_STACK_HEADING = 0;
    public static double ZONE_1_HEADING = 0;
    public static double ZONE_2_HEADING = 0;
    public static double ZONE_3_HEADING = 0;
    public static double POWER_SHOTS_HEADING = Math.toRadians(180);
    public static double WOBBLE_GOAL_2_HEADING = Math.toRadians(144.441);
    public static double CLEAR_STACK_2_HEADING = Math.toRadians(90);
    public static double PARK_HEADING = Math.PI;
}
