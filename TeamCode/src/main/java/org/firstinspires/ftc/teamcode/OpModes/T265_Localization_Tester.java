package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.google.gson.internal.$Gson$Preconditions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class T265_Localization_Tester extends OpMode
{
    private static T265Camera slamra;
    public double move_power = 1.0;
    public double turn_power = 1.0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private Robot robot;
    private GamepadEx gamepadEx;

    @Override
    public void init() {
        gamepadEx = new GamepadEx(gamepad1);

        if(slamra == null){
            try{
                slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
            }catch (Exception e){
                telemetry.addData("LOL","Couldn't find the camera... Trying again...");
            }
        }

        //slamra.setPose(new Pose2d(0, 0, new Rotation2d(0, 0)));

        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        if(slamra == null){
            try{
                slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
                slamra.start();
            }catch (Exception e){
                telemetry.addData("LOL","Couldn't find the camera... Trying again...");
            }
        }
    }

    @Override
    public void loop() {
        while(slamra == null){
            try{
                slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
            }catch (Exception e){
                telemetry.addData("LOL","Couldn't find the camera... Trying again...");
            }
        }

        robot.updateBulkData();
        final int robotRadius = 9; // inches

        slamra.sendOdometry(robot.getVelocityXMetersPerSecond(), robot.getVelocityYMetersPerSecond());

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() * 39.37, up.pose.getTranslation().getY() * 39.37);
        Rotation2d rotation = up.pose.getRotation();

        packet.put("Pose", "(" + Math.round(translation.getX() * 100)/100.0 + ", " + Math.round(translation.getY() * 100)/100.0 + ", " + Math.round(rotation.getDegrees() * 100)/100.0 + ")");

        packet.put("start", gamepadEx.isPress(GamepadEx.Control.a));
        field.strokeCircle(translation.getX() + robot.localizer.OFFSET_FROM_CENTER.getY(), translation.getY() + robot.localizer.OFFSET_FROM_CENTER.getX(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1 + robot.localizer.OFFSET_FROM_CENTER.getY(), y1 + robot.localizer.OFFSET_FROM_CENTER.getX(), x2 + robot.localizer.OFFSET_FROM_CENTER.getY(), y2 + robot.localizer.OFFSET_FROM_CENTER.getX());

        dashboard.sendTelemetryPacket(packet);

        robot.drive.driveCentric(gamepad1, move_power, turn_power, robot.getPos().getHeading() + (Math.PI/2));
        robot.drive.write();

        robot.updatePos();
    }

    @Override
    public void stop() {
        slamra.stop();
    }

}
