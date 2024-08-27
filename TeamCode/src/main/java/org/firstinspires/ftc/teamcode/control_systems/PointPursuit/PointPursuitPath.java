package org.firstinspires.ftc.teamcode.control_systems.PointPursuit;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;
import org.firstinspires.ftc.teamcode.utils.Points.Vector;

import java.util.ArrayList;
import java.util.Map;

public class PointPursuitPath {
    private ArrayList<CurvePoint> path;
    private Map<Double , CommandBase> markers;

    public PointPursuitPath(ArrayList<CurvePoint> path2){
        path = new ArrayList<>();
        for(CurvePoint c : path2){
            path.add(new CurvePoint(c));
        }
    }

    public PointPursuitPath(){
        path = new ArrayList<>();
    }

    public PointPursuitPath inject(double spacing){
        ArrayList<CurvePoint> newPath = new ArrayList<>();
        for(int i = 0 ; i < path.size() - 1; i++){
            Vector start = path.get(i).toVector();
            Vector endPoint = path.get(i + 1).toVector();
            Vector displacement = new Vector(endPoint.x - start.x , endPoint.y - start.y);
            double numOfPointsThatFit = (displacement.getMagnitude() / spacing);
            Vector unit = displacement.normalize(null);
            unit.mult(displacement.getMagnitude() / numOfPointsThatFit);

            for(int j = 0 ; j < numOfPointsThatFit ; j ++){
                Vector newVector = Vector.mult(unit , j , null);
                CurvePoint pointToAdd = Vector.add(start , newVector , null).toCurvePoint();
                pointToAdd.setMovementSpeed(path.get(i).movementSpeed);
                pointToAdd.setLookaheadDistance(path.get(i).lookaheadDistance);
                pointToAdd.setFaceTowardsAngle(path.get(i).faceTowardsAngle);
                newPath.add(pointToAdd);
            }
        }
        newPath.add(path.get(path.size() - 1));
        return setPath(newPath);
    }

    public PointPursuitPath smooth(double b , double tolerance){
        ArrayList<CurvePoint> smoothPath = this.path;
        double change = tolerance;
        while(change <= tolerance){
            change = 0.0;
            for(int i = 1 ; i < this.path.size() - 1 ; i ++){
                CurvePoint currPoint = smoothPath.get(i);
                CurvePoint prevPoint = smoothPath.get(i -1);
                CurvePoint nextPoint = smoothPath.get(i +1);
                CurvePoint currCopy = new CurvePoint(currPoint.x , currPoint.y);

                currPoint.x += b * (nextPoint.x + prevPoint.x - 2*currPoint.x);
                currPoint.y += b * (nextPoint.y + prevPoint.y - 2*currPoint.y);

                change += Math.abs((currPoint.x - currCopy.x));
                change += Math.abs((currPoint.y - currCopy.y));

            }

        }
        return setPath(smoothPath);
    }

    public PointPursuitPath extendPath(double distance){
        double pathAngle = Math.atan2(path.get(path.size() - 1).y - path.get(path.size() - 2).y , path.get(path.size() - 1).x - path.get(path.size() - 1).x);
        CurvePoint extendedPoint;
        double xExtended = Math.cos(pathAngle) * distance;
        double yExtended = Math.sin(pathAngle) * distance;


        extendedPoint = new CurvePoint(path.get(path.size() - 1).x + xExtended , path.get(path.size() - 1).y + yExtended);
        path.get(path.size() - 1).setPoint(extendedPoint);
        return this;
    }

    public PointPursuitPath curvePoint(CurvePoint c){
        this.path.add(new CurvePoint(c));
        return this;
    }


    public PointPursuitPath startingPoint(CurvePoint c){
        path.add(0 , c);
        return this;
    }

    public PointPursuitPath buildCurve(CurvePoint c){
        CurvePoint curve = this.path.get(path.size() - 1);
        curve.setFaceTowardsAngle(c.getFaceTowardsAngle());
        curve.setMovementSpeed(c.getMovementSpeed());
        curve.setLookaheadDistance(c.lookaheadDistance);
        return this;
    }


    public CurvePoint clipToPath(PointPursuitPath path , double xPosition , double yPosition){
        //start off the closest distance really high
        double closestPointDistance = Double.MAX_VALUE;
        //define the integer to a a random number
        int currLineIndex = 0;
        //define a point at a random index
        CurvePoint pointOnPath = path.get(0);

        //intersect perpendicular lines through each segment and see which one is the closest to
        //current robot position. Get the closest point and see which segment it is in
        //set pointOnPath x and y as the intersection closest to robot
        //change the index to be the index of which the line segment is on the path
        for(int i = 1 ; i < path.size(); i ++){
            //intersect the perpendicular line
            CurvePoint clippedToLine = clipToLine(path.get(i - 1) , path.get(i) , new CurvePoint(xPosition , yPosition));

            //find distance between the intersection and the robot pos
            double distanceToPoint = Math.hypot(xPosition - clippedToLine.x , yPosition - clippedToLine.y);

            //if the distance is less than closestPointDistance. save the closestPointDistance as
            //distance and set currIndex to i and pointOnPath to the clippedLinePoint
            if(distanceToPoint < closestPointDistance){
                closestPointDistance = distanceToPoint;
                pointOnPath = clippedToLine;
                currLineIndex = i;
            }
            //go through loop again
        }
        //return a new curve point with currLineIndex as the index
        return new CurvePoint(pointOnPath.x , pointOnPath.y , currLineIndex , path.get(currLineIndex).lookaheadDistance);
    }


    //method to intersect a perpendicular line to robot pos to line segment
    public CurvePoint clipToLine(CurvePoint linePoint1 , CurvePoint linePoint2 , CurvePoint currPos){
        //make sure the line segments don't have same x or y
        //if they have same y or x , they can have undefined or 0 slope
        if(linePoint1.x == linePoint2.x){
            linePoint1.x = linePoint2.x + 0.0001;
        }

        if(linePoint1.y == linePoint2.y){
            linePoint1.y = linePoint2.y + 0.0001;
        }

        //slope of the line segment
        double slope1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        //slope of the perpendicular line
        double perpendicularSlope = (linePoint1.x - linePoint2.x) / (linePoint2.y - linePoint1.y);

        //equations for the two lines in form mx + b
        double perpendicularLine = (-perpendicularSlope * currPos.x);
        double segmentLine = (slope1 * linePoint1.x);

        //find clipped x using this equation
        double clippedx = (perpendicularLine + currPos.y + segmentLine - linePoint1.y) / (slope1 - perpendicularSlope);
        //find error between the starting point of the line segment and the xclipped
        double xError = (clippedx - linePoint1.x);
        double clippedy = (slope1 * (xError) + linePoint1.y);

        return new CurvePoint(clippedx , clippedy);
    }


    public CurvePoint clipToPathPercent(PointPursuitPath path ,CurvePoint clippedToPath , double percent){
        CurvePoint currPathIndexPoint = path.get(clippedToPath.pointIndex);
        double clippedDistance = Math.sqrt(Math.pow(currPathIndexPoint.y - clippedToPath.y , 2) +
                Math.pow(currPathIndexPoint.x - clippedToPath.x , 2));

        CurvePoint previousPathIndexPoint = path.get(clippedToPath.pointIndex - 1);
        double lineDistance = Math.sqrt(Math.pow(currPathIndexPoint.y - previousPathIndexPoint.y , 2) +
                Math.pow(currPathIndexPoint.x - previousPathIndexPoint.x , 2));

        if(lineDistance - clippedDistance/lineDistance >= percent ){
            return path.get(clippedToPath.pointIndex - 1);
        }else{
            return clippedToPath;
        }
    }

    public double getPathLength(){
        double length = 0;
        for(int i = 0; i < path.size() - 1; i ++){
            length += Math.sqrt(Math.pow(path.get(i + 1).y - path.get(i).y , 2) + Math.pow(path.get(i + 1).x - path.get(i).x , 2));
        }
        return length;
    }


    public void add(CurvePoint c){
        path.add(c);
    }

    public CurvePoint get(int index){
        return path.get(index);
    }

    public int size(){
        return path.size();
    }

    public ArrayList<CurvePoint> getPath(){
        return path;
    }

    public PointPursuitPath setPath(ArrayList<CurvePoint> path){
        this.path = path;
        return this;
    }

    public PointPursuitPath copyOf(){
        return new PointPursuitPath(this.path);
    }

    public void addProgressMarker(double progress , CommandBase command){
        this.markers.put(progress , command);
    }

    public Map<Double , CommandBase> getMarkers(){
        return this.markers;
    }
}