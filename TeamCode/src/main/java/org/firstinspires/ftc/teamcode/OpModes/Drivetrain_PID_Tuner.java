package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;

@TeleOp
public class Drivetrain_PID_Tuner extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();

        waitForStart();

        while(opModeIsActive()){
            robot.updateBulkData();

            robot.GoTo(new Pose2d(Mecanum_Drive.x, Mecanum_Drive.y, Math.toRadians(Mecanum_Drive.angle)), new Pose2d(1.0, 1.0, 1.0));

            telemetry.update();
        }
    }
}
