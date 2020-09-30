package org.firstinspires.ftc.teamcode.PurePusuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Math.Vector2;

import java.util.ArrayList;

public class Math_Functions {
    public static double angleWrap (double angle){
        if(angle > Math.PI){
            return -((2 * Math.PI ) - angle);
        }
        return angle;
    }

    public static double LineCenterOffset(Pose2d center, Pose2d linePoint1, Pose2d linePoint2){
        //Finding the slope
        double m1 = (linePoint2.getY() - linePoint1.getY()) / (linePoint2.getX() - linePoint1.getX());

        double offset = 0.0;

        //Removing the offset from the circles to normalize them to (0,0)
        double x1 = linePoint1.getX() - center.getX();
        double y1 = linePoint1.getY() - center.getY();

        double yc = center.getY();
        double xc = center.getX();

        try{
            double b = y1 - (m1 * x1);
            double bPerp = yc - ((1/m1) * xc);

            double xInt = ((m1 * (b - bPerp))/(Math.pow(m1, 2) - 1));
            double yInt = (m1 * xInt) + b;

            yInt += center.getY();
            xInt += center.getX();

            if(new Vector2(linePoint1.getX(), linePoint1.getY()).norm() <= 100){
                offset = Math.hypot(Math.abs(yInt - center.getY()), Math.abs((xInt - center.getX())))/2;
            }else{
                offset = 0.0;
            }

        }catch(Exception e){

        }
        return offset;
    }

    public static ArrayList<Vector2> lineCenterIntercection(Vector2 center, double radius, Vector2 linePoint1, Vector2 linePoint2){
        //Checking if the points are basically on the same line to make sure that no unecessary math is being taken place
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003){
            linePoint1.y = linePoint2.y + 0.003;
        }
        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }

        //Finding the slope
        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + Math.pow(m1, 2);

        //Removing the offset from the circles to normalize them to (0,0)
        double x1 = linePoint1.x - center.x;
        double y1 = linePoint1.y - center.y;

        //Finding the traditional A, B, and C variables in the Quadratic Equation
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2.0) * x1);
        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1,2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1,2) - Math.pow(radius, 2);

        ArrayList<Vector2> allPoints = new ArrayList<>();

        try{
            //This is finding the two x intersections and the two y intersections
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/ (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            //Reappllying the offset onto the circle
            xRoot1 += center.x;
            yRoot1 += center.y;

            //Checking for if any of the points are not on the line before adding them into the array
            double minX = linePoint1.x < linePoint2.x ?  linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ?  linePoint1.x : linePoint2.x;

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Vector2(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/ (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += center.x;
            yRoot2 += center.y;

            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Vector2(xRoot2, yRoot2));
            }
        }catch(Exception e){
            System.out.println(e);
        }
        return allPoints;
    }
}
