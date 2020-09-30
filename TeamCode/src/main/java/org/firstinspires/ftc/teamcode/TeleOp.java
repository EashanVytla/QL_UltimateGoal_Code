package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;
    Pose2d storedPos = new Pose2d(0, 0, 0);

    long prevTime = 0;
    double refreshRate;

    enum Drive_State{
        Driving,
        AutoAllign
    }

    Drive_State mDriveState = Drive_State.Driving;

    public void init(){
        robot = Robot.getInstance(hardwareMap, telemetry);
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public void loop(){
        long dt = (System.currentTimeMillis() - prevTime);

        if(dt != 0){
            refreshRate = 1000/dt;
        }

        prevTime = System.currentTimeMillis();

        telemetry.addData("delta time: ", refreshRate);

        switch (mDriveState){
            case Driving:
                robot.drive.drive(gamepad1, 1.0, 0.5);

                if(gamepad1.left_stick_button){
                    mDriveState = Drive_State.AutoAllign;
                }else if(gamepad1.a){
                    storedPos = robot.getPos();
                }
                break;
            case AutoAllign:
                if(gamepad1.left_stick_x > 0.3 || gamepad1.right_stick_x > 0.3){
                    mDriveState = Drive_State.Driving;
                }

                if(robot.getPos().vec().distTo(storedPos.vec()) >= 0.5){
                    robot.GoTo(storedPos, new Pose2d(1.0, 1.0, 1.0));
                }else{
                    robot.drive.setPower(0, 0, 0);
                    mDriveState = Drive_State.Driving;
                }

                robot.updateGoTo();
                break;
        }

        robot.updatePos();
        robot.drive.write();
        telemetry.addData("stored pos:", storedPos);
        telemetry.addData("State: ", mDriveState);
        telemetry.addData("pos: ", robot.getPos());
    }
}
