package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.OpModes.LinearTeleOp;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "Track Width Tuner")
public class Track_Width_Tuner extends LinearOpMode {
    double maxTurn = 0.3;
    double maxMove = 1;
    Robot robot = null;
    boolean running = false;
    boolean running2 = false;

    GamepadEx gamepad1ex;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        gamepad1ex = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive())
        {
            robot.updateBulkData();

            if(gamepad1ex.isPress(GamepadEx.Control.a)){
                running = !running;
            }

            if(gamepad1ex.isPress(GamepadEx.Control.b)){
                running2 = !running2;
            }
            robot.updatePos();

            if(running){
                //robot.drive.setPower(0, 0, 0.4);
                //robot.drive.write();
                robot.GoTo(0, 0, 14 * Math.PI, 1.0, 1.0, 0.3);
            }else if(running2){
                //robot.drive.setPower(0, 0, -0.4);
                //robot.drive.write();
                robot.GoTo(0, 0, -14 * Math.PI, 1.0, 1.0, 0.3);
            }else{
                robot.drive.setPower(0, 0, 0);
                robot.drive.write();
            }

            gamepad1ex.loop();

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
