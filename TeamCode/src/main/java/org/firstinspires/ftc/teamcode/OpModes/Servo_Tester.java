package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;

@TeleOp
public class Servo_Tester extends LinearOpMode {
    Robot robot;
    Caching_Servo servo;
    private double pos = 0.859;
    final String name = "push_slide";

    @Override
    public void runOpMode(){
        robot = Robot.getInstance(hardwareMap, telemetry);
        servo = new Caching_Servo(hardwareMap, name);

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                if(pos < 1){
                    pos += 0.00001;
                }
            } else if (gamepad1.dpad_down){
                if(pos > 0){
                    pos -= 0.00001;
                }
            }

            servo.setPosition(pos);

            servo.write();

            telemetry.addData("Position", servo.getPosition());
            telemetry.update();
        }
        robot.stop();
    }
}
