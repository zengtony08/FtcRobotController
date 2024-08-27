package org.firstinspires.ftc.teamcode.control_systems.PointPursuit;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;
import org.firstinspires.ftc.teamcode.utils.time.LibConstants;

import java.util.ArrayList;

public class PursuitMath implements LibConstants {
    private double previousTime = 0;
    private boolean resetTimer = false;
    private double deltaTime = 0;

    public ArrayList<CurvePoint> lineCircleIntersection(CurvePoint circleCenter, double radius, CurvePoint linePoint1, CurvePoint linePoint2){
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003){
            linePoint1.y = linePoint2.y + 0.003;
        }

        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y)/ (linePoint2.x - linePoint1.x);

        double quadraticA = Math.pow(m1,2) + 1.0;

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0*m1*y1) - (2.0*Math.pow(m1,2)*x1);

        double quadraticC = (Math.pow(m1,2) * Math.pow(x1,2)) - (2.0*y1*m1*x1) + Math.pow(y1,2) - Math.pow(radius,2);

        ArrayList<CurvePoint> points = new ArrayList<>();


        try{

            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB,2.0) - (4.0*quadraticA*quadraticC))) / (2.0*quadraticA);
            double yRoot1 = m1*(xRoot1 - x1) + y1;


            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;


            if(xRoot1 > minX && xRoot1 < maxX){
                points.add(new CurvePoint(xRoot1,yRoot1));
            }


            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB,2) - (4.0*quadraticA*quadraticC))) / (2.0*quadraticA);
            double yRoot2 = m1*(xRoot2-x1) + y1;


            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;


            if(xRoot2 > minX && xRoot2 < maxX){
                points.add(new CurvePoint(xRoot2,yRoot2));
            }



        }catch (Exception e){

        }

        return points;
    }


    public CurvePoint lookAheadPoint(PointPursuitPath path, CurvePoint robotLocation, CurvePoint currPointInPath){
        CurvePoint lookAheadPoint = new CurvePoint(path.get(currPointInPath.pointIndex));
        lookAheadPoint.setPoint(new CurvePoint(currPointInPath.x,currPointInPath.y));

        for(int i = 0; i < path.size()-1; i ++){
            CurvePoint startLine = path.get(i);
            CurvePoint endLine = path.get(i+1);

            ArrayList<CurvePoint> intersections =
                    lineCircleIntersection(robotLocation , currPointInPath.lookaheadDistance,
                            startLine ,endLine);

            double closestDistance = 1000000;
            for(int p = 0; p < intersections.size(); p ++){

                CurvePoint thisIntersection = intersections.get(p);

                double dist = Math.hypot(thisIntersection.x - path.get(path.size()-1).x,
                        thisIntersection.y - path.get(path.size()-1).y);

                if(dist < closestDistance){
                    closestDistance = dist;
                    lookAheadPoint.setPoint(thisIntersection);
                }
            }


            if(Math.sqrt(Math.pow(path.get(path.size() - 1).y - robotLocation.y , 2) + Math.pow(path.get(path.size() - 1).x - robotLocation.x , 2)) <= currPointInPath.lookaheadDistance + 2){
                return new CurvePoint(path.get(path.size() - 1));
            }

        }


        lookAheadPoint.setMovementSpeed(path.get(currPointInPath.pointIndex).movementSpeed);
        lookAheadPoint.setFaceTowardsAngle(path.get(currPointInPath.pointIndex).faceTowardsAngle);

        return lookAheadPoint;
    }

    public double lookaheadDampener(double initialLookahead , double currentLookahead , double targetLookahead , double dampenTime ){
        if(!resetTimer && initialLookahead != targetLookahead){
            previousTime = System.currentTimeMillis();
            resetTimer = true;
        }else if(initialLookahead == targetLookahead){
            resetTimer = false;
        }

        deltaTime = System.currentTimeMillis() - previousTime;
        previousTime = System.currentTimeMillis();

        double lookaheadDifference = targetLookahead - initialLookahead;
        double distanceToDecreasePerMilli = lookaheadDifference / (dampenTime * MILLISECS_PER_SEC);

        double lookaheadGain = distanceToDecreasePerMilli * deltaTime;

        double lowestRange = initialLookahead < targetLookahead ? initialLookahead : targetLookahead;
        double highestRange = initialLookahead > targetLookahead ? initialLookahead : targetLookahead;

        return Range.clip(currentLookahead + lookaheadGain , lowestRange , highestRange);
    }

}
