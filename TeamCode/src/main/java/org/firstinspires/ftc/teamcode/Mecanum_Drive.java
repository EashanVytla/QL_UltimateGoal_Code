package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector2;

import java.util.Arrays;


public class Mecanum_Drive{
    Caching_Motor[] motors = new Caching_Motor[4];

    Telemetry telemetry;

    PIDFController PID_X;
    PIDFController PID_Y;
    PIDFController PID_Z;

    float kp = 0.07f;
    float ki = 0;
    float kd = 0.0125f;

    float kpr = 0.86f;
    float kir = 0;
    float kdr = 0.06f;
    int currentWriteIndex = 0;

    private double scalePower(double speed, double min, double max){
        //One
        //double power = 0.5 * Math.pow(1.5 * (speed - 0.5), 3.0) + 0.5;

        /*double power = 0;
        //Two
        if(speed != 0.0){
            power = Math.pow(speed, 3.0)/Math.abs(speed);
        }*/

        //Three
        double power = speed;

        return Range.clip(power, min, max);
    }

    public Mecanum_Drive(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        motors[0] = new Caching_Motor(map, "front_left");
        motors[1] = new Caching_Motor(map, "front_right");
        motors[2] = new Caching_Motor(map, "back_left");
        motors[3] = new Caching_Motor(map, "back_right");

        motors[0].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[2].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[3].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PID_X = new PIDFController(new PIDCoefficients(kp, ki, kd));
        PID_Y = new PIDFController(new PIDCoefficients(kp, ki, kd));
        PID_Z = new PIDFController(new PIDCoefficients(kpr, kir, kdr));

        motors[1].motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].motor.setDirection(DcMotorSimple.Direction.REVERSE );
    }

    public void write(){
        motors[currentWriteIndex].write();
        currentWriteIndex = (currentWriteIndex + 1) % 4;
    }

    public void setPower(double y, double x, double rightX){
        double  FrontLeftVal = y - x + rightX;
        double FrontRightVal = y + x - rightX;
        double BackLeftVal = y + x + rightX;
        double BackRightVal = y - x - rightX;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {Math.abs(FrontRightVal), Math.abs(FrontLeftVal), Math.abs(BackLeftVal), Math.abs(BackRightVal)};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            FrontLeftVal /= wheelPowers[3];
            FrontRightVal /= wheelPowers[3];
            BackLeftVal /= wheelPowers[3];
            BackRightVal /= wheelPowers[3];
        }

        telemetry.addData("Front Left: ", motors[0].motor.getPower());
        telemetry.addData("Front Right: ", motors[1].motor.getPower());
        telemetry.addData("Back Left: ", motors[2].motor.getPower());
        telemetry.addData("Back Right: ", motors[3].motor.getPower());

        motors[0].setPower(FrontLeftVal);
        motors[1].setPower(FrontRightVal);
        motors[2].setPower(BackLeftVal);
        motors[3].setPower(BackRightVal);
    }

    public void setPower(Vector2 vec, double rot){
        setPower(vec.x, vec.y, rot);
    }

    public void setPowerCentic(double x, double y, double rot, double heading){
        setPower(new Vector2(x, y).rotated(-heading), rot);
    }

    public void drive(Gamepad gamepad, double maxMove, double maxTurn){
        setPower(scalePower(gamepad.left_stick_x, -maxMove, maxMove), scalePower(gamepad.left_stick_y, -maxMove, maxMove), scalePower(gamepad.right_stick_x, -maxTurn, maxTurn));
    }

    public void driveCentric(Gamepad gamepad, double maxMove, double maxTurn, double heading){
        setPowerCentic(scalePower(gamepad.left_stick_x, -maxMove, maxMove), scalePower(gamepad.left_stick_y, -maxMove, maxMove), scalePower(gamepad.right_stick_x, -maxTurn, maxTurn), heading);
    }

    public void goToPoint(Pose2d targetPos, Pose2d currentPos, double xspeed, double yspeed, double zspeed){
        PID_X.setOutputBounds(-xspeed, xspeed);
        PID_Y.setOutputBounds(-yspeed, yspeed);
        PID_Z.setOutputBounds(-zspeed, zspeed);

        double heading = 0;
        double target_heading = targetPos.getHeading();

        if(currentPos.getHeading() <= Math.PI){
            heading = currentPos.getHeading();
        }else{
            heading = -((2 * Math.PI ) - currentPos.getHeading());
        }

        if(Math.abs(targetPos.getHeading() - heading) >= Math.toRadians(180.0)){
            target_heading = -((2 * Math.PI) - targetPos.getHeading());
        }

        telemetry.addData("Target heading: ", target_heading);
        telemetry.addData("Current heading: ", heading);

        PID_X.setTargetPosition(targetPos.getX());
        PID_Y.setTargetPosition(targetPos.getY());
        PID_Z.setTargetPosition(target_heading);

        setPowerCentic(PID_X.update(currentPos.getX()), -PID_Y.update(currentPos.getY()), PID_Z.update(heading), currentPos.getHeading());
    }
}

