package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;

@TeleOp(name = "S4T Localizer Tester")
public class S4T_Localizer_Tester extends OpMode {
    Robot robot;

    public void init() {
        robot = Robot.getInstance(hardwareMap, telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public void loop(){
        robot.updatePos();

        telemetry.addData("Pos: ", robot.getPos());
    }
}
