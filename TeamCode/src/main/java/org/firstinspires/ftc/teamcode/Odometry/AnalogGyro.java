package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PurePusuit.Math_Functions;

import java.util.ArrayList;

public class AnalogGyro {
    HardwareMap hardwareMap;
    AnalogInput out;
    double correctionCoeff = 1;
    double cumulativeAngle;
    private static double startHeading = 0;

    public AnalogGyro(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        out = hardwareMap.get(AnalogInput.class, "sangyro");
        cumulativeAngle = 0;
    }

    public double getAngleRaw(){
        return (out.getVoltage() / 3.25) * (2 * Math.PI);
    }

    public void reset(){
        startHeading = cumulativeAngle;
    }

    public void update(){
        double currentAngle = getAngleRaw();
        cumulativeAngle = currentAngle * correctionCoeff;
    }

    private double greatestVoltage = 0;
    public double findGreatestVoltage(){
        if(out.getVoltage() > greatestVoltage){
            greatestVoltage = out.getVoltage();
        }

        return greatestVoltage;
    }

    private double greatestAngle = 0;
    public double findGreatestAngle(){
        if(getAngleRaw() > greatestAngle){
            greatestAngle = getAngleRaw();
        }

        return greatestAngle;
    }

    public double getAngleCorrected(){
        return ((2 * Math.PI) + (cumulativeAngle - startHeading)) % (2 * Math.PI);
    }
}
