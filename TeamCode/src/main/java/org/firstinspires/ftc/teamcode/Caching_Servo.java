package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Caching_Servo {
    HardwareMap hardwareMap;
    String name;
    Servo servo = hardwareMap.servo.get(name);
    double prev_pos = 0.0;

    double query = -2.0;

    float pos = 0;

    double EPSILON = 0.001;

    double prev_write = 0;
    double current_write = 0;

    public void setPosition(double pos){
        if (Math.abs(pos - prev_pos) > EPSILON){
            query = pos;
        }
        else{
            query = -1.0;
        }
    }

    public double getPosition(){
        return prev_pos;
    }

    public void write(){
        if (query != -1.0) {
            servo.setPosition(query);
            prev_pos = query;
        }
    }

    public void write(double rate){
        current_write++;
        if (prev_write / current_write < rate){
            write();
            prev_write = current_write;
        }
    }
}