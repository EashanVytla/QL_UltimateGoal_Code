package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

@Autonomous
public class Flywheel_PID_Tuner extends OpMode {
    Robot robot = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet;
    GamepadEx gamepadEx;

    private boolean pid = false;

    @Override
    public void init(){
        robot = new Robot(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);

        robot.shooter.shooter.motor.setVelocityPIDFCoefficients(Flywheel_Coefficients.kp, Flywheel_Coefficients.ki, Flywheel_Coefficients.kd, 0.0);
    }

    private int toggle;

    @Override
    public void loop(){
        robot.updateBulkData();
        packet = new TelemetryPacket();

        if(gamepadEx.isPress(GamepadEx.Control.a)){
            pid = !pid;
        }

        packet.put("pid", pid);

        if(pid){
            robot.shooter.setShooterVelocity(Flywheel_Coefficients.target);
        }else{
            robot.shooter.shooter.motor.setPower(0.0);
        }

        if(gamepadEx.isPress(GamepadEx.Control.b)){
            toggle++;
            toggle %= 4;
            robot.shooter.powerShot(toggle);
        }

        robot.shooter.slideSetPower(gamepadEx.gamepad.left_stick_y);
        telemetry.addData("SLIDE HEIGHT", robot.shooter.getShooterAngle());

        robot.shooter.leftSlide.write();
        robot.shooter.rightSlide.write();
        robot.shooter.pushSlide.write();

        packet.put("Error", Flywheel_Coefficients.target - robot.shooter.getShooterVelocity());
        packet.put("Velocity", robot.shooter.getShooterVelocity());

        dashboard.sendTelemetryPacket(packet);
        gamepadEx.loop();
    }

    @Override
    public void stop(){
        //robot.localizer.stopCamera();
    }
}

@Config
class Flywheel_Coefficients{
    public static double kp = 300;
    public static double ki = 25;
    public static double kd = 50;

    public static int target = 1500;
}
