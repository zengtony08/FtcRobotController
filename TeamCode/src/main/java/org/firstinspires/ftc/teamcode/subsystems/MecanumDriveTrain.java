package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Point;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control_systems.Odometry.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.control_systems.Odometry.ThreeWheelTracker;
import org.firstinspires.ftc.teamcode.control_systems.PIDController.PIDTriplet;
import org.firstinspires.ftc.teamcode.control_systems.PointPursuit.LookaheadMath;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;
import org.firstinspires.ftc.teamcode.utils.Util;
import org.firstinspires.ftc.teamcode.utils.time.TDTTimer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MecanumDriveTrain extends DriveTrain {
    private Telemetry telemetry;
    private Logger _logger;

    private final TDTTimer timer;
    private final boolean resetTimer = false;

    public PIDController tempForward = new PIDController(0.04, 0,0);//(0.1, 0, 0)
    public PIDController tempStrafe = new PIDController(0.04, 0, 0);//(0.06, 0, 0)
    public PIDController tempTurn = new PIDController(0.007, 0, 0); //(0.04, 0, 0)

    public ThreeWheelTracker odometry;

    public StandardTrackingWheelLocalizer localizer;
    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();

    public MecanumDriveTrain(String tlName, String blName, String trName, String brName, PIDTriplet DrivePID , PIDTriplet StrafePID , PIDTriplet turnPID , double trackWidth , double auxWidth , HardwareMap hardwareMap , Telemetry telemetry) {
        super(tlName, blName, trName, brName, DrivePID , StrafePID , turnPID ,  hardwareMap);
        odometry = new ThreeWheelTracker(brName , blName , tlName , trackWidth , auxWidth , hardwareMap , telemetry);

        timer = new TDTTimer();
        this.telemetry = telemetry;
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        _logger = new Logger("Mecanum", 3);
    }

//    public void
//    fieldCentric(double movement_x , double movement_y , double movement_turn){
//        // We can rotate the field by getting the angle to the point and offsetting it by the robot heading
//        double newAngle = Math.atan2(movement_y , movement_x) - Math.toRadians(odometry.getHeading());
//
//        // Calulate magnitude of movement vector
//        double magnitude = Math.hypot(movement_y , movement_x);
//
//        // Calculate the individual components of the vector
//        double fieldCentric_movement_x = Math.cos(newAngle) * magnitude;
//        double fieldCentric_movement_y = Math.sin(newAngle) * magnitude;
//
//        double topLeftPower = fieldCentric_movement_y + fieldCentric_movement_x + movement_turn;
//        double bottomLeftPower = fieldCentric_movement_y - fieldCentric_movement_x + movement_turn;
//        double topRightPower = fieldCentric_movement_y - fieldCentric_movement_x - movement_turn;
//        double bottomRightPower = fieldCentric_movement_y + fieldCentric_movement_x - movement_turn;
//
//        // Now we can just set the power to the motors
//        tl.setPower(topLeftPower);
//        bl.setPower(bottomLeftPower);
//        tr.setPower(topRightPower);
//        br.setPower(bottomRightPower);
//    }


    public void fieldCentric(double xSpeed, double ySpeed, double turn , double movementSpeed){
        double angle = Math.atan2(ySpeed , xSpeed) - localizer.getPose().getHeading();

        double speed = Math.hypot(ySpeed , xSpeed);

        double move_x = speed * Math.cos(angle);
        double move_y = speed * Math.sin(angle);

        move_x = Range.clip(move_x , -movementSpeed , movementSpeed);
        move_y = Range.clip(move_y , -movementSpeed , movementSpeed);

        robotCentric(move_x , move_y , turn);
    }


    public void robotCentric(double x , double y , double turn){
        double tlPower = (y ) + (x )  + (turn ); //(y ) + (x )  - (turn )
        double blPower = (y) - (x)  + (turn ); //(y) - (x)  - (turn )
        double trPower = (y ) - (x)  - (turn ); //(y ) - (x)  + (turn )
        double brPower = (y ) + (x)  - (turn ); //(y ) + (x)  + (turn )

        telemetry.addData("Front Left", tlPower);
        telemetry.addData("Back Left", blPower);
        telemetry.addData("Front Right", trPower);
        telemetry.addData("Back Right", brPower);
        _logger.info("Directional Powers : %.3f, %.3f, %.3f", x, y, turn);
        _logger.info("Motor Powers : tl: %.3f, bl: %.3f, tr: %.3f, br: %.3f", tlPower, blPower, trPower, brPower);

        List<Double> powerList = Arrays.asList(tlPower, blPower, trPower, brPower);
        normalizeMecanumPowers(powerList);

        tl.setPower(powerList.get(0));
        bl.setPower(powerList.get(1));
        tr.setPower(powerList.get(2));
        br.setPower(powerList.get(3));
    }


    public void resetDtMotors(){
        tl.resetMotor();
        tr.resetMotor();
        bl.resetMotor();
        br.resetMotor();
    }


    public void normalizeMecanumPowers(List<Double> powerList){
        double max = 0;

        for(double d : powerList){
            if(Math.abs(d) > max){
                max = Math.abs(d);
            }
        }

        if (max > 1) {
            for (int i = 0; i < powerList.size(); i++) {
                double current = powerList.get(i);
                powerList.set(i, current / max);
            }
        }
    }

    // Parameter targetPoint : The target point (x , y , targetHeading , movementSpeed , lookaheadDistance)
    public void goToPosition(CurvePoint targetPoint , CurvePoint headingPoint){ //TODO: save new pid to driveTrain
//        odometry.updateTracker();
        localizer.update();

        double movement_x = tempStrafe.calculate(localizer.getPose().getX() , targetPoint.x);
        double movement_y = tempForward.calculate(localizer.getPose().getY() , targetPoint.y);
        double movement_turn = -tempTurn.calculate(Math.toDegrees(localizer.getPose().getHeading()), headingPoint.faceTowardsAngle); //degrees

        telemetry.addData("PID X", movement_x);
        telemetry.addData("PID Y", movement_y);
        telemetry.addData("PID TURN", movement_turn);
//        _logger.info("goToPosition X Y H: %.1f, %.1f, %.1f", targetPoint.x, targetPoint.y, headingPoint.faceTowardsAngle);
        _logger.info("PID X Y H : %.3f, %.3f, %.3f", movement_x, movement_y, movement_turn);
        fieldCentric(movement_x , movement_y , movement_turn, 0.3);

    }

    public void followPath(ArrayList<CurvePoint> path){
        double stopState = 0;
        double startTime = 0;

        // Get the new extended path to use for calculating heading point
        // Extend the path 20 inches above the lookahead distance just to be safe
        ArrayList<CurvePoint> extendedPath = LookaheadMath.extendedPath(path , path.get(path.size()-1).lookaheadDistance + 20);

        while(stopState < 500){
            localizer.update();
            CurvePoint robotLocation = new CurvePoint(localizer.getPose().getX() , localizer.getPose().getY());

            CurvePoint lastPointOfPath = path.get(path.size() - 1);


            // Get the current index we are on the path
            CurvePoint currIndexInPath = LookaheadMath.clipToPath(path , robotLocation);

            // The end point of the line segment we are currently on in the path
            CurvePoint currPointInPath = path.get(currIndexInPath.pointIndex);

            CurvePoint lookahead = LookaheadMath.getLookaheadPoint(path , robotLocation , currIndexInPath);
            CurvePoint headingLookahead = LookaheadMath.getLookaheadPoint(extendedPath , robotLocation , currIndexInPath);

            /* The relative point angle will use the faceTowardsAngle of the line segment
            to find the angle offset to face the desired direction.Subtracting 90 degrees
            since the forwards angle of a unit circle is 90 degrees while the forwards angle
            of the robot is 0 degrees */
            double relativePointAngle = (Math.toRadians(currPointInPath.faceTowardsAngle) - Math.toRadians(90));

            // Get the absolute angle to the lookahead point
            double angleToPoint = Math.atan2(lookahead.y - localizer.getPose().getY() , lookahead.x - localizer.getPose().getX());

            // Change the absolute angle , so we can add in our preference to it (if we should face towards the lookahead, sideways , etc)
            double targetHeading = angleToPoint + relativePointAngle;

            /* Angle wrap the target heading and convert it to degrees since the turnController
            in the goToPosition() method works in degrees */

            targetHeading = Util.angleWrap(targetHeading);
            targetHeading = Math.toDegrees(targetHeading);

            /* Set the faceTowardsAngle of the lookahead point to the target heading.
            This value will be called and used as the target position for the turnController in goToPosition() */

            headingLookahead.faceTowardsAngle = targetHeading;

            goToPosition(lookahead , headingLookahead);

            if(Math.sqrt(Math.pow(lastPointOfPath.x - robotLocation.x , 2) + Math.pow(lastPointOfPath.y - robotLocation.y , 2)) < 4){
                stopState = System.currentTimeMillis() - startTime;
            }else{
                startTime = System.currentTimeMillis();
            }
        }

        // Stop the robot when it reaches the end
        stop();
    }



    public void setTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public double getLEncoderInches(){return this.odometry.getLeftInches();}

    public double getREncoderInches(){
        return this.odometry.getRightInches();
    }

    public double getHEncoderInches(){
        return this.odometry.getHorizontalInches();
    }

}
