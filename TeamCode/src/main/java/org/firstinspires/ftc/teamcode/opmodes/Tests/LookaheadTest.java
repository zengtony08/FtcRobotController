package org.firstinspires.ftc.teamcode.opmodes.Tests;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control_systems.PointPursuit.LookaheadMath;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.utils.Points.CurvePoint;

import java.util.ArrayList;

@TeleOp
public class LookaheadTest extends OpMode {
    private Robot robot;
//    MecanumDriveTrain driveTrain;
    ArrayList<CurvePoint> path;

    @Override
    public void init(){
        robot = new Robot(hardwareMap, telemetry);

        path = new ArrayList<>();

        // Add points to the path (x , y , movementSpeed , faceTowardsAngle , lookaheadDistance)
        path.add(new CurvePoint(new Point(0, 0) , 0.75 , 0 , 5.0 ));
        path.add(new CurvePoint(new Point(0 , 25) , 0.5 , 0 , 7.5 ));
        path.add(new CurvePoint(new Point(25 , 25) , 0.65 , 0 , 6.5 ));
    }

    @Override
    public void loop(){
        robot.driveTrain.localizer.update();

        // Store the robot's x and y location as a Point
        CurvePoint robotLocation = new CurvePoint(robot.driveTrain.localizer.getPose().getX() , robot.driveTrain.localizer.getPose().getY());

        // Get the current index of the path the robot is currently on , pass in (Path , Robot Location)
        CurvePoint currIndexInPath = LookaheadMath.clipToPath(path , robotLocation);

        // Get the lookahead point , pass in (Path , Robot Location , Current index in the path)
        CurvePoint lookaheadPoint = LookaheadMath.getLookaheadPoint(path , robotLocation , currIndexInPath);

        // Output everything to telemetry

        telemetry.addData("x position : " , robot.driveTrain.localizer.getPose().getX());
        telemetry.addData("y position : " , robot.driveTrain.localizer.getPose().getY());
        telemetry.addData("heading : " , robot.driveTrain.localizer.getPose().getHeading());
        telemetry.addData("Lookahead x : " , lookaheadPoint.x);
        telemetry.addData("Lookahead y : " , lookaheadPoint.y);
        telemetry.addData("Lookahead movementSpeed : " , lookaheadPoint.movementSpeed);
        telemetry.addData("Lookahead faceTowardsAngle : " , lookaheadPoint.faceTowardsAngle);
        telemetry.addData("Lookahead lookaheadDistance : " , lookaheadPoint.lookaheadDistance);
        telemetry.update();
    }

}

