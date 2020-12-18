package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;

@Autonomous
public class PID_Tuner extends LinearOpMode {
    Robot robot = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()){
            robot.updateBulkData();

            robot.GoTo(robot.drive.target_pos, new Pose2d(1.0, 1.0, 1.0));

            dashboardTelemetry.addData("Pos", robot.getPos());
            dashboardTelemetry.addData("Error", robot.getPos().vec().distTo(robot.drive.target_pos.vec()));
            dashboardTelemetry.update();

            telemetry.addData("Pos", robot.getPos());
            telemetry.addData("Error", robot.getPos().vec().distTo(robot.drive.target_pos.vec()));
            telemetry.update();
        }
    }
}
