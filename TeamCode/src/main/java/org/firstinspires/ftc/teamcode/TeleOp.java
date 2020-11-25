package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;
    Pose2d storedPos = new Pose2d(0, 0, 0);
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousA = false;
    private boolean grabberToggle = false;

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
        switch (mDriveState){
            case Driving:
                robot.drive.drive(gamepad1, 0.3, 0.3);

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

                break;
        }

        if(gamepad1.a && !previousA){
            grabberToggle = !grabberToggle;

            if(grabberToggle){
                robot.wobbleGoal.clamp();
            }else{
                robot.wobbleGoal.release();
            }
        }

        if(gamepad1.dpad_up && !previousDpadUp){
            robot.shooter.lift_auto();
        }

        if(gamepad1.dpad_down && !previousDpadDown){
            robot.shooter.drop();
        }

        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousA = gamepad1.a;

        robot.updatePos();
        robot.drive.write();
        telemetry.addData("stored pos:", storedPos);
        telemetry.addData("State: ", mDriveState);
        telemetry.addData("Pos: ", robot.getPos());
        telemetry.addData("Right X: ", robot.getRight_X_Dist());
        telemetry.addData("Left X: ", robot.getLeft_X_Dist());
        telemetry.addData("Right Y: ", robot.getRight_Y_Dist());
        telemetry.addData("Left Y: ", robot.getLeft_Y_Dist());
    }
}
