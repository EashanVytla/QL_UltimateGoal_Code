package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@Autonomous
public class PID_Tuner extends LinearOpMode {
    Robot robot = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet;
    GamepadEx gamepadEx;
    public boolean PID = false;

    //PIDFController controller = new PIDFController(new PIDCoefficients(Shooter.kp_Flywheel, Shooter.ki_Flywheel, Shooter.kd_Flywheel));

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);

        waitForStart();

        while(opModeIsActive()){
            packet = new TelemetryPacket();

            double velo = robot.shooter.shooter.motor.getVelocity(AngleUnit.RADIANS);
            //robot.updateBulkData();

            //robot.GoTo(robot.drive.target_pos, new Pose2d(1.0, 1.0, 1.0));

            /*dashboardTelemetry.addData("Pos", robot.getPos());
            dashboardTelemetry.addData("Error", robot.getPos().vec().distTo(robot.drive.target_pos.vec()));
            dashboardTelemetry.update();

            telemetry.addData("Pos", robot.getPos());
            telemetry.addData("Error", robot.getPos().vec().distTo(robot.drive.target_pos.vec()));
            telemetry.update();*/



            if(gamepadEx.isPress(GamepadEx.Control.a)){
                PID = !PID;
            }

            if(PID){
                //controller.setTargetPosition(5.2);
                //robot.shooter.shooter.setPower(controller.update(velo));
                robot.shooter.shooter.write();
            }else{
                robot.shooter.shooter.setPower(0.0);
                robot.shooter.shooter.write();
            }

            packet.put("Velocity", velo);


            dashboard.sendTelemetryPacket(packet);
            gamepadEx.loop();
        }
    }
}
