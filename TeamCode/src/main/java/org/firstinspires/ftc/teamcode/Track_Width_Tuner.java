package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Track Width Tuner")
public class Track_Width_Tuner extends OpMode {
    double maxTurn = 0.3;
    double maxMove = 1;
    Robot robot;
    boolean running = false;

    public void init(){
        robot = Robot.getInstance(hardwareMap, telemetry);
    }

    public void stop(){
        robot.stop();
    }

    public void loop(){
        if(gamepad1.a){
            running = true;
        }

        if(gamepad1.b){
            running = false;
        }
        robot.updatePos();

        if(running){
            robot.drive.motors[2].setPower(-0.3);
            robot.drive.motors[1].setPower(0.3);
        }else{
            robot.drive.setPower(0, 0, 0);
        }

        robot.drive.write();

        telemetry.addData("Pos: ", robot.getPos());
        telemetry.addData("Right X: ", robot.getRight_X_Dist());
        telemetry.addData("Left X: ", robot.getLeft_X_Dist());
        telemetry.addData("Right Y: ", robot.getRight_Y_Dist());
        telemetry.addData("Left Y: ", robot.getLeft_Y_Dist());
    }
}
