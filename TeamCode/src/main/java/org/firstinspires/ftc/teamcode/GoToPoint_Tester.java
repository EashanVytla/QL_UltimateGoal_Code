package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous(name = "PP Tester")
public class GoToPoint_Tester extends OpMode {
    Robot robot;

    ArrayList<CurvePoint> allPoints = new ArrayList<>();

    @Override
    public void init() {
        robot = Robot.getInstance(hardwareMap, telemetry);

        allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 15, 0));
        allPoints.add(new CurvePoint(36, 36, 1.0, 1.0, 15, 0));
        allPoints.add(new CurvePoint(0, 72, 1.0, 1.0, 15, 0));
    }

    @Override
    public void stop() {
        robot.stop();
    }

    @Override
    public void loop() {
        telemetry.addData("Pose: ", robot.getPos().toString());

        //RobotMovement.followCurve(allPoints, robot, telemetry);
        robot.GoTo(new Pose2d(24, 24, 0), new Pose2d(1, 1,1));

        robot.updateGoTo();
    }
}
