package org.firstinspires.ftc.teamcode.utils.Points;


import android.graphics.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double lookaheadDistance;
    public double distanceAlongPath = 0;
    public int pointIndex = 0;
    public double movementSpeed;
    public double faceTowardsAngle;
    public double stopTime;
    public double stopStartDistance;

    public CurvePoint(double faceTowardsAngle) {
        this.faceTowardsAngle = faceTowardsAngle;
    }

    public CurvePoint(double x , double y){
        this.x = x;
        this.y = y;
    }

    public void setPoint(CurvePoint curvePoint) {
        this.x = curvePoint.x;
        this.y = curvePoint.y;
    }

    public CurvePoint(double x , double y , int pointIndex , double lookaheadDistance){
        this.x = x;
        this.y = y;
        this.pointIndex = pointIndex;
    }


    public CurvePoint(double x , double y , double faceTowardsAngle , int pointIndex ){
        this.x = x;
        this.y = y;
        this.faceTowardsAngle = faceTowardsAngle;
        this.pointIndex = pointIndex;
    }

    public CurvePoint(CurvePoint curvePoint){
        this.x = curvePoint.x;
        this.y = curvePoint.y;
        this.movementSpeed = curvePoint.movementSpeed;
        this.faceTowardsAngle = curvePoint.faceTowardsAngle;
        this.lookaheadDistance = curvePoint.lookaheadDistance;
    }

    public CurvePoint(Point point, double movementSpeed , double faceTowardsAngle , double lookaheadDistance){
        this.x = point.x;
        this.y = point.y;
        this.movementSpeed = movementSpeed;
        this.faceTowardsAngle = faceTowardsAngle;
        this.lookaheadDistance = lookaheadDistance;
    }


    public CurvePoint(Point point, double movementSpeed , double faceTowardsAngle, double stopTime , double stopStartDistance){
        this.x = point.x;
        this.y = point.y;
        this.movementSpeed = movementSpeed;
        this.faceTowardsAngle = faceTowardsAngle;
        this.stopTime = stopTime;
        this.stopStartDistance = stopStartDistance;
    }


    public CurvePoint(Point point, double movementSpeed , double faceTowardsAngle, double lookaheadDistance , double stopTime , double stopStartDistance){
        this.x = point.x;
        this.y = point.y;
        this.movementSpeed = movementSpeed;
        this.faceTowardsAngle = faceTowardsAngle;
        this.lookaheadDistance = lookaheadDistance;
        this.stopTime = stopTime;
        this.stopStartDistance = stopStartDistance;
    }

    public CurvePoint(){

    }



    public CurvePoint setX(double x){
        this.x = x;
        return this;
    }

    public CurvePoint setY(double y){
        this.y = y;
        return this;
    }

    public CurvePoint setMovementSpeed(double movementSpeed){
        this.movementSpeed = movementSpeed;
        return this;
    }

    public CurvePoint setFaceTowardsAngle(double faceTowardsAngle){
        this.faceTowardsAngle = faceTowardsAngle;
        return this;
    }

    public CurvePoint setLookaheadDistance(double lookaheadDistance){
        this.lookaheadDistance = lookaheadDistance;
        return this;
    }

    public CurvePoint setStartStopDistance(double stopStartDistance){
        this.stopStartDistance = stopStartDistance;
        return this;
    }

    public CurvePoint setStopTime(double stopTime){
        this.stopTime = stopTime;
        return this;
    }

    public void setDistanceAlongPath(double dist){
        this.distanceAlongPath = dist;
    }

    public double getDistanceAlongPath(){
        return distanceAlongPath;
    }

    public Vector toVector(){
        return new Vector(this.x , this.y);
    }

    public double getMovementSpeed(){
        return movementSpeed;
    }

    public double getFaceTowardsAngle(){
        return faceTowardsAngle;
    }


}
