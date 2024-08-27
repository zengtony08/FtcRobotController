package org.firstinspires.ftc.teamcode.control_systems.Odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Util;
import org.firstinspires.ftc.teamcode.utils.robot_hardware.Motor;

import java.util.ArrayList;

public class ThreeWheelTracker {
    public Motor LEncoder;
    public Motor REncoder;
    public Motor HEncoder;

    private double prevLeftE = 0;
    private double prevRightE = 0;
    private double prevHorizontalE = 0;

    private double x;
    private double y;
    private double heading;

    public double trackWidth;
    public double auxWidth;

    private final double parallelScaleFactor = 0;
    private final double auxScaleFactor = 0;
    private final double turnScaleFactor = 0;

    private double lookahead = 0;
    private final Telemetry telemetry;

    public ThreeWheelTracker(String LeftEncoderName , String RightEncoderName , String HorizontalEncoderName , double trackWidth, double auxWidth , HardwareMap hwmap , Telemetry telemetry){
        LEncoder = new Motor("leftOdo" , 2000 , 1.8896, hwmap);
        REncoder = new Motor("rightOdo" , 2000 , 1.8896 , hwmap);
        HEncoder = new Motor("horizontalOdo" ,  2000 , 1.8896, hwmap);

        LEncoder.reverse();
//        HEncoder.reverse();
        this.trackWidth = 13.6282;
        this.auxWidth = -4.24;
        this.telemetry = telemetry;

        reset();
    }

    public void updateTracker() {
        double leftPosition = LEncoder.getCurrPosInches();
        double rightPosition = REncoder.getCurrPosInches();
        double horizontalPosition = HEncoder.getCurrPosInches();

        double deltaLeft = leftPosition - prevLeftE;
        prevLeftE = leftPosition;
        double deltaRight = rightPosition - prevRightE;
        prevRightE = rightPosition;
        double deltaHorizontal = horizontalPosition - prevHorizontalE;
        prevHorizontalE = horizontalPosition;

        double deltaHeading = ((deltaLeft - deltaRight) / trackWidth);

        heading = Util.angleWrap((leftPosition - rightPosition) / trackWidth);

        double angularComponent = auxWidth * deltaHeading;
        double relativeXPosition = deltaHorizontal - angularComponent;
        double relativeYPosition = (deltaLeft + deltaRight) / 2.0;

        x += (Math.cos(heading) * relativeXPosition) - (Math.sin(heading) * relativeYPosition);
        y += (Math.sin(heading) * relativeXPosition) + (Math.cos(heading) * relativeYPosition);
    }

    public double getX(){
        return this.x;
    }

    public double getY(){
        return this.y;
    }

    public double getLeftInches(){
        return LEncoder.getCurrPosInches();
    }

    public double getRightInches(){
        return REncoder.getCurrPosInches();
    }

    public double getHorizontalInches(){
        return HEncoder.getCurrPosInches();
    }

    public void reset(){
        prevLeftE = 0;
        prevRightE = 0;
        prevHorizontalE = 0;
        LEncoder.resetMotor();
        REncoder.resetMotor();
        HEncoder.resetMotor();
    }

    public void reverseLeftEncoder(){
        LEncoder.reverse();
    }

    public void reverseRightEncoder(){
        REncoder.reverse();
    }

    public void reverseHorizontalEncoder(){
        HEncoder.reverse();
    }

    public double getHeadingRAD() {
        return heading;
    }

    public double getHeading(){
        return Math.toDegrees(heading);
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }

    public double getHeadingRadians(){
        return heading;
    }

    public double angleWrap(double angle){
        while(angle <= -Math.PI){
            angle += (Math.PI * 2.0);
        }
        while(angle >= Math.PI){
            angle -= (Math.PI * 2.0);
        }

        return angle;
    }

    public void setLookahead(double lookahead){
        this.lookahead = lookahead;
    }

    public double getLookahead(){
        return this.lookahead;
    }

    public ArrayList<Motor> getEncoders(){
        ArrayList<Motor> encoders = new ArrayList<>();
        encoders.add(LEncoder);
        encoders.add(REncoder);
        encoders.add(HEncoder);
        return encoders;
    }


}
