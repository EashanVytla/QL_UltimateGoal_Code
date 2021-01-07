package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.OpModes.LinearTeleOp;

@TeleOp(name = "Track Width Tuner")
public class Track_Width_Tuner extends LinearOpMode {
    double maxTurn = 0.3;
    double maxMove = 1;
    Robot robot = null;
    boolean running = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive())
        {
            robot.updateBulkData();

            if(gamepad1.a){
                running = true;
            }

            if(gamepad1.b){
                running = false;
            }
            robot.updatePos();

            if(running){
                robot.drive.setPower(0, 0, 0.2);
                robot.drive.write();
            }else{
                robot.drive.setPower(0, 0, 0);
                robot.drive.write();
            }

            telemetry.addData("Pos: ", robot.getPos());
            telemetry.addData("Right X: ", robot.getRight_X_Dist());
            telemetry.addData("Left X: ", robot.getLeft_X_Dist());
            telemetry.addData("Right Y: ", robot.getRight_Y_Dist());
            telemetry.addData("Left Y: ", robot.getLeft_Y_Dist());

            telemetry.addData("Right X RAW", robot.getRawRight_X_Dist());
            telemetry.addData("Left X RAW", robot.getRawLeft_X_Dist());
            telemetry.addData("Right Y RAW", robot.getRawRight_Y_Dist());
            telemetry.addData("Left Y RAW", robot.getRawLeft_Y_Dist());

            telemetry.update();
        }
    }
}
