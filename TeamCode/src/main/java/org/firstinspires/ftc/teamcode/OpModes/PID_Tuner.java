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
public class PID_Tuner extends OpMode {
    Robot robot = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet;
    GamepadEx gamepadEx;

    //PIDFController controller = new PIDFController(new PIDCoefficients(Shooter.kp_Flywheel, Shooter.ki_Flywheel, Shooter.kd_Flywheel));

    private Caching_Motor leftSlide;
    private Caching_Motor rightSlide;
    private boolean pid = false;

    private PIDFController slidesController;

    @Override
    public void init(){
        robot = new Robot(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);

        slidesController = new PIDFController(new PIDCoefficients(Coefficients.kp, Coefficients.ki, Coefficients.kd));

        rightSlide = new Caching_Motor(hardwareMap, "right_slide");
        leftSlide = new Caching_Motor(hardwareMap, "left_slide");
        leftSlide.motor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightSlide.motor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftSlide.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
        robot.updateBulkData();
        packet = new TelemetryPacket();

        double left = robot.getData2().getMotorCurrentPosition(leftSlide.motor);
        double right = robot.getData2().getMotorCurrentPosition(rightSlide.motor);

        packet.put("Slide Height Left: ", left);
        packet.put("Slide Height Right: ", right);

        telemetry.addData("Slide Height Left: ", left);
        telemetry.addData("Slide Height Right: ", right);

        if(gamepadEx.isPress(GamepadEx.Control.a)){
            pid = true;
        }

        if(gamepadEx.isPress(GamepadEx.Control.b)){
            pid = false;
        }

        packet.put("pid", pid);

        if(pid){
            slidesController.setTargetPosition(Coefficients.target);

            double power = slidesController.update((left+right)/2);
            packet.put("Error", Coefficients.target - ((left+right)/2));

            leftSlide.setPower(Range.clip(-power, -1.0, 1.0));
            rightSlide.setPower(Range.clip(-power, -1.0, 1.0));
        }else{
            leftSlide.setPower(gamepad1.left_stick_y * 0.5);
            rightSlide.setPower(gamepad1.left_stick_y * 0.5);
        }

        leftSlide.write();
        rightSlide.write();

        dashboard.sendTelemetryPacket(packet);
        gamepadEx.loop();
    }

    @Override
    public void stop(){
        //robot.localizer.stopCamera();
    }
}

class Coefficients{
    public static double kp = 0.01;
    public static double ki = 0.0;
    public static double kd = 0.0;

    public static int target = 604;
}
