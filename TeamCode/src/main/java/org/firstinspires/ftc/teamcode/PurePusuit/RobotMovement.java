package org.firstinspires.ftc.teamcode.PurePusuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

public class RobotMovement {
    static int index = 0;
    static int previous_index = 0;
    static CurvePoint followMe = new CurvePoint(0, 0, 0, 0, 0, 0);


    public static void followCurve(ArrayList<CurvePoint> allPoints, Robot robot, Telemetry telemetry){
        if(index < allPoints.size() - 2){
            followMe = getFollowPointPath(allPoints, new Pose2d(robot.getPos().getX(), robot.getPos().getY(), robot.getPos().getHeading()), allPoints.get(index).followDistance);

            index = getCurrentLine(followMe.toVec(), allPoints);
            previous_index = index;

            telemetry.addData("current line: ", String.valueOf(index));
        }else{
            telemetry.addData("Last line!", "Lets goooo babbyyyy!");
            followMe = allPoints.get(allPoints.size() - 1);
        }

        robot.GoTo(followMe.x, followMe.y, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).heading, followMe.moveSpeed, followMe.moveSpeed, followMe.turnSpeed);
    }


    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Pose2d robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        double runningDistance = 0;
        double previous_dist = 0;

        for (int i = Math.max(Math.min(Math.max(0, index), pathPoints.size() - 3), 0); i < Math.min(index + 2, pathPoints.size() - 1); i++){
            CurvePoint start = pathPoints.get(i);
            CurvePoint end = pathPoints.get(i + 1);
            runningDistance += previous_dist;

            ArrayList<Vector2> intersections = Math_Functions.lineCenterIntercection(new Vector2(robotLocation.getX(), robotLocation.getY()), followRadius, start.toVec(), end.toVec());

            double closestDistance = Double.MAX_VALUE;

            for (Vector2 thisIntersection : intersections){
                double dist = Math.hypot(thisIntersection.x - pathPoints.get(i + 1).x, thisIntersection.y - pathPoints.get(i + 1).y);

                if (dist < closestDistance){
                    closestDistance = dist;
                    followMe.setPoint(thisIntersection);
                }
            }
            previous_dist = Math.hypot(end.x - start.x, end.y - start.y);
        }
        return followMe;
    }

    public static int getCurrentLine(Vector2 intersection, ArrayList<CurvePoint> allPoints){
        int currentline = 0;

        for (int j = 0; j < allPoints.size() - 1; j++){
            Vector2 start = new Vector2(allPoints.get(j).x, allPoints.get(j).y);
            Vector2 end = new Vector2(allPoints.get(j + 1).x, allPoints.get(j + 1).y);

            if (Math.abs(start.distanceToVector(end) - (start.distanceToVector(intersection) + end.distanceToVector(intersection))) <= 0.003){

                if (Math.abs(j - previous_index) < 2) {
                    currentline = j;
                    break;
                }
            }
        }
        return currentline;
    }
}